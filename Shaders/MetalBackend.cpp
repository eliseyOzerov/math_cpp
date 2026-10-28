#include "MetalBackend.h"

#include "ShaderCompiler.h"
#include "ShaderLibrary.h"

namespace editor::shaders::metalBackend {

std::string typeName(SocketType type) {
    switch (type) {
        case SocketType::number:
        case SocketType::any: return "float";
        case SocketType::vector2: return "float2";
        case SocketType::vector3: return "float3";
        case SocketType::vector4:
        case SocketType::color: return "float4";
        case SocketType::texture: return "texture2d<float>";
    }
    return "float";
}

namespace {

/// Uniforms, the interpolated vertex output and the full-screen triangle's vertex function. The
/// Uniforms layout must match ShaderUniforms.h.
std::string prelude() {
    return std::string(R"metal(#include <metal_stdlib>
using namespace metal;

struct Uniforms {
    float2 resolution;
    float time;
    float deltaTime;
    float4 pointer;
    float frame;
    float frameRate;
    float2 pointerDelta;
    float clickCount;
    float seed;
    float4 date;
    float4 audio;
};

struct VertexOut {
    float4 position [[position]];
    float2 uv;
};

)metal") + "vertex VertexOut " + ShaderCompiler::vertexFunction() + R"metal((uint id [[vertex_id]]) {
    float2 corner = float2((id << 1) & 2, id & 2);
    VertexOut out;
    out.position = float4(corner * 2.0f - 1.0f, 0.0f, 1.0f);
    out.uv = corner;
    return out;
}

)metal";
}

/// Helpers that take a function, which kernels may not (Docs/Foundations.md), so they are
/// Metal text. The Loop node's body arrives as a lambda of the step and the last value.
const char* functionHelpers() {
    return R"metal(// Runs `body` up to 64 times, each step taking the step's index and the last value.
template<typename Body> float4 loopAccumulate(Body body, float4 initial, float count) {
    float4 value = initial;
    int steps = clamp(int(floor(count + 0.5f)), 0, 64);
    for (int stepIndex = 0; stepIndex < steps; stepIndex++) { value = body(float(stepIndex), value); }
    return value;
}

)metal";
}


/// Reading textures, which kernels cannot do (they have no texture type), so each function here makes
/// the taps and hands the numbers to the kernel that holds the arithmetic (graphics_metal's
/// Texture.metal; Color.metal's `lookupColor`). UV starts at the bottom left, as in GLSL, and a
/// texture's rows start at the top, so every read flips v.
const char* textureHelpers() {
    return R"metal(// The texture coordinate of a UV that starts at the bottom left.
float2 textureCoordinate(float2 uv) { return float2(uv.x, 1.0f - uv.y); }

float4 sampleImage(texture2d<float> image, sampler imageSampler, float2 uv) {
    return image.sample(imageSampler, textureCoordinate(uv));
}

// Samples with a chosen wrap (0 clamp, 1 repeat, 2 mirror), filter (0 linear, 1 nearest) and mip level.
float4 sampleImageWith(texture2d<float> image, float2 uv, int wrap, int filtering, float lod) {
    constexpr sampler clampLinear(filter::linear, mip_filter::linear, address::clamp_to_edge);
    constexpr sampler repeatLinear(filter::linear, mip_filter::linear, address::repeat);
    constexpr sampler mirrorLinear(filter::linear, mip_filter::linear, address::mirrored_repeat);
    constexpr sampler clampNearest(filter::nearest, address::clamp_to_edge);
    constexpr sampler repeatNearest(filter::nearest, address::repeat);
    constexpr sampler mirrorNearest(filter::nearest, address::mirrored_repeat);
    float2 coordinate = textureCoordinate(uv);
    if (filtering == 1) {
        if (wrap == 1) return image.sample(repeatNearest, coordinate, level(lod));
        if (wrap == 2) return image.sample(mirrorNearest, coordinate, level(lod));
        return image.sample(clampNearest, coordinate, level(lod));
    }
    if (wrap == 1) return image.sample(repeatLinear, coordinate, level(lod));
    if (wrap == 2) return image.sample(mirrorLinear, coordinate, level(lod));
    return image.sample(clampLinear, coordinate, level(lod));
}

// The texture projected along each axis, blended by the normal. The kernels have no triplanar
// blend yet, so its weights (the normal's components to the sharpness, scaled to sum to 1) are here.
float4 triplanar(texture2d<float> image, float3 position, float3 normal, float scale, float sharpness) {
    constexpr sampler wrap(filter::linear, address::repeat);
    float3 weights = pow(abs(normal), float3(max(sharpness, 1.0f)));
    weights /= max(weights.x + weights.y + weights.z, 0.000001f);
    float3 p = position * scale;
    return image.sample(wrap, p.yz) * weights.x + image.sample(wrap, p.xz) * weights.y + image.sample(wrap, p.xy) * weights.z;
}

// A colour through a lookup table laid out as a strip of N slices, each N by N, blue across slices.
// The two slices either side of the blue value are read at the red and green position and blended.
// The index arithmetic is texture addressing (half a texel in from each edge), so it stays here.
float3 lookupColor(texture2d<float> table, float3 c) {
    constexpr sampler tableSampler(filter::linear, address::clamp_to_edge);
    float n = float(table.get_height());
    float3 q = clamp(c, 0.0f, 1.0f);
    float slice = q.b * (n - 1.0f);
    float s0 = floor(slice);
    float s1 = min(s0 + 1.0f, n - 1.0f);
    float2 inside = float2(q.r * (n - 1.0f) + 0.5f, q.g * (n - 1.0f) + 0.5f);
    float3 a = table.sample(tableSampler, float2((s0 * n + inside.x) / (n * n), inside.y / n)).rgb;
    float3 b = table.sample(tableSampler, float2((s1 * n + inside.x) / (n * n), inside.y / n)).rgb;
    return mix(a, b, slice - s0);
}

)metal";
}

/// The image effects (graphics_metal's Effects.metal). Every one takes a bounded number of taps, with
/// the edge clamped; the offsets, weights and the colour made of the taps are ShaderEffects.h's.
const char* imageEffectHelpers() {
    return R"metal(float4 imageTap(texture2d<float> image, float2 uv) {
    constexpr sampler tapSampler(filter::linear, address::clamp_to_edge);
    return image.sample(tapSampler, textureCoordinate(uv));
}

float2 texelOf(texture2d<float> image) { return 1.0f / float2(image.get_width(), image.get_height()); }

float4 blurImage(texture2d<float> image, float2 uv, float radius, int mode, float2 middle) {
    float2 texel = texelOf(image);
    float4 total = float4(0.0f);
    float weights = 0.0f;
    if (mode <= 1) {
        for (int y = -3; y <= 3; y++) {
            for (int x = -3; x <= 3; x++) {
                float2 offset = blurTapOffset(x, y);
                float weight = blurTapWeight(offset, mode);
                total += imageTap(image, uv + offset * radius * texel) * weight;
                weights += weight;
            }
        }
        return total / weights;
    }
    for (int tap = 0; tap < 16; tap++) {
        float fraction = blurTapFraction(tap);
        float2 point = mode == 2 ? radialBlurPoint(uv, middle, fraction, radius) : zoomBlurPoint(uv, middle, fraction, radius);
        total += imageTap(image, point);
    }
    return total / 16.0f;
}

float4 sharpenImage(texture2d<float> image, float2 uv, float amount) {
    float2 texel = texelOf(image);
    float4 aroundSum = imageTap(image, uv + float2(texel.x, 0.0f)) + imageTap(image, uv - float2(texel.x, 0.0f))
                     + imageTap(image, uv + float2(0.0f, texel.y)) + imageTap(image, uv - float2(0.0f, texel.y));
    return sharpenedColor(imageTap(image, uv), aroundSum, amount);
}

float2 sobelOf(texture2d<float> image, float2 uv, float2 texel) {
    float topLeft = luminance(imageTap(image, uv + texel * float2(-1.0f, 1.0f)));
    float top = luminance(imageTap(image, uv + texel * float2(0.0f, 1.0f)));
    float topRight = luminance(imageTap(image, uv + texel * float2(1.0f, 1.0f)));
    float left = luminance(imageTap(image, uv + texel * float2(-1.0f, 0.0f)));
    float right = luminance(imageTap(image, uv + texel * float2(1.0f, 0.0f)));
    float bottomLeft = luminance(imageTap(image, uv + texel * float2(-1.0f, -1.0f)));
    float bottom = luminance(imageTap(image, uv + texel * float2(0.0f, -1.0f)));
    float bottomRight = luminance(imageTap(image, uv + texel * float2(1.0f, -1.0f)));
    return sobelGradient(topLeft, top, topRight, left, right, bottomLeft, bottom, bottomRight);
}

float edgeStrength(texture2d<float> image, float2 uv, float strength) {
    return edgeStrengthOf(sobelOf(image, uv, texelOf(image)), strength);
}

float4 embossImage(texture2d<float> image, float2 uv, float amount, float angle) {
    float2 direction = embossOffset(angle, texelOf(image));
    return embossColor(luminance(imageTap(image, uv + direction)), luminance(imageTap(image, uv - direction)), amount);
}

float4 chromaticAberration(texture2d<float> image, float2 uv, float amount, float2 middle) {
    float2 offset = chromaticOffset(uv, amount, middle);
    return chromaticColor(imageTap(image, uv + offset), imageTap(image, uv), imageTap(image, uv - offset));
}

float4 scanlinesCrt(texture2d<float> image, float2 uv, float2 pixel, float spacing, float intensity, float curvature) {
    float2 centred = crtCentredWarp(uv, curvature);
    float2 warped = crtWarpedUv(centred);
    if (isOutsideUnitSquare(warped)) return float4(0.0f, 0.0f, 0.0f, 1.0f);
    return crtShadedColor(imageTap(image, warped), centred, pixel, spacing, intensity, curvature);
}

float4 bloomImage(texture2d<float> image, float2 uv, float cutoff, float radius, float intensity) {
    float2 texel = texelOf(image);
    float4 centre = imageTap(image, uv);
    float3 glow = float3(0.0f);
    float weights = 0.0f;
    for (int ring = 1; ring <= 3; ring++) {
        for (int tap = 0; tap < 8; tap++) {
            float3 color = imageTap(image, uv + bloomRingOffset(ring, tap, radius, texel)).rgb;
            float weight = bloomRingWeight(ring);
            glow += bloomBrightPart(color, cutoff) * weight;
            weights += weight;
        }
    }
    glow += bloomBrightPart(centre.rgb, cutoff);
    return bloomCombined(centre, glow, weights, intensity);
}

float4 pixelateImage(texture2d<float> image, float2 uv, float size) {
    return imageTap(image, pixelateTapUv(uv, size, float2(image.get_width(), image.get_height())));
}

float4 mosaicImage(texture2d<float> image, float2 uv, float2 aspectScale, float tiles, float gap, float variation) {
    float2 id = mosaicTileId(uv, aspectScale, tiles);
    float4 color = imageTap(image, mosaicTileCentreUv(id, aspectScale, tiles));
    return mosaicTileColor(color, uv, aspectScale, tiles, gap, variation);
}

// Moves each pixel by a map: mode 0 reads the map's red and green, mode 1 the slope of its brightness.
float4 displaceImage(texture2d<float> image, texture2d<float> map, float2 uv, float amount, int mode) {
    float2 offset;
    if (mode == 0) {
        offset = displacementFromMap(imageTap(map, uv).rg);
    } else {
        float2 texel = texelOf(map);
        offset = luminanceSlope(luminance(imageTap(map, uv + float2(texel.x, 0.0f))), luminance(imageTap(map, uv - float2(texel.x, 0.0f))),
                                luminance(imageTap(map, uv + float2(0.0f, texel.y))), luminance(imageTap(map, uv - float2(0.0f, texel.y))));
    }
    return imageTap(image, displacedUv(uv, offset, amount));
}

// The centre tap is read for the lit look only, but reading it always keeps the taps free of branches.
float4 normalFromHeight(texture2d<float> image, float2 uv, float strength, int mode, float lightAngle) {
    float2 texel = texelOf(image);
    float2 slope = luminanceSlope(luminance(imageTap(image, uv + float2(texel.x, 0.0f))), luminance(imageTap(image, uv - float2(texel.x, 0.0f))),
                                  luminance(imageTap(image, uv + float2(0.0f, texel.y))), luminance(imageTap(image, uv - float2(0.0f, texel.y))));
    return normalFromHeightColor(slope, strength, mode, lightAngle, imageTap(image, uv));
}

float4 lensImage(texture2d<float> image, float2 uv, float strength, float zoom) {
    float2 warped = lensUv(uv, strength, zoom);
    if (isOutsideUnitSquare(warped)) return float4(0.0f, 0.0f, 0.0f, 1.0f);
    return imageTap(image, warped);
}

// Painterly smoothing: each quadrant around the pixel is summed over (radius + 1)^2 taps and the
// kernel picks the mean of the one that varies least.
float4 kuwahara(texture2d<float> image, float2 uv, float radiusInput) {
    float2 texel = texelOf(image);
    int radius = kuwaharaRadius(radiusInput);
    float4 statistics[4];
    for (int quadrant = 0; quadrant < 4; quadrant++) {
        float2 windowSign = kuwaharaQuadrantSign(quadrant);
        float3 sum = float3(0.0f);
        float squares = 0.0f;
        for (int y = 0; y <= 4; y++) {
            for (int x = 0; x <= 4; x++) {
                if (x > radius || y > radius) continue;
                float3 color = imageTap(image, uv + float2(x, y) * windowSign * texel).rgb;
                sum += color;
                squares += dot(color, color);
            }
        }
        statistics[quadrant] = kuwaharaQuadrantStatistics(sum, squares, radius);
    }
    return float4(kuwaharaSelectedMean(statistics[0], statistics[1], statistics[2], statistics[3]), imageTap(image, uv).a);
}

)metal";
}

/// The simulations (graphics_metal's Simulation.metal): each step reads the state from Previous
/// Frame, where an unwritten texture has alpha 0 and means "begin", and gives the next state. The
/// taps and the neighbour loops are here; the rules are ShaderSimulation.h's. `particleField` reads
/// no texture and is the kernel itself.
const char* simulationHelpers() {
    return R"metal(float4 reactionDiffusionStep(texture2d<float> state, float2 uv, float feed, float kill, float diffusionA, float diffusionB,
                             float rate, float2 brush, float brushing) {
    float2 texel = texelOf(state);
    float4 centre = imageTap(state, uv);
    if (centre.a < 0.5f) return grayScottStartState(uv, texel);
    float2 edgeSum = float2(0.0f);
    float2 cornerSum = float2(0.0f);
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            if (x == 0 && y == 0) continue;
            float2 neighbour = imageTap(state, uv + float2(x, y) * texel).rg;
            if (x == 0 || y == 0) { edgeSum += neighbour; } else { cornerSum += neighbour; }
        }
    }
    float2 laplacian = grayScottLaplacian(centre.rg, edgeSum, cornerSum);
    float2 next = grayScottReaction(centre.rg, laplacian, feed, kill, diffusionA, diffusionB, rate);
    next = grayScottBrushed(next, uv, brush, brushing, texel);
    return float4(next, 0.0f, 1.0f);
}


float4 lifeStep(texture2d<float> state, float2 uv, float cell, float interval, float frame, float density, float fade) {
    float2 step = lifeCellStep(cell, float2(state.get_width(), state.get_height()));
    float2 centreUv = lifeCellCentreUv(uv, step);
    float4 centre = imageTap(state, centreUv);
    if (centre.a < 0.5f) return lifeStartColor(uv, step, density);
    if (!lifeAdvances(frame, interval)) return centre;
    int neighbours = 0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            if (x == 0 && y == 0) continue;
            if (lifeCellIsAlive(imageTap(state, lifeNeighbourPoint(centreUv, x, y, step)).r)) neighbours += 1;
        }
    }
    return lifeNextState(centre, neighbours, fade);
}


float2 fluidVelocity(texture2d<float> state, float2 uv, float2 texel, float2 offset) {
    return fluidVelocityOf(imageTap(state, uv + offset * texel).rg);
}

float fluidDivergence(texture2d<float> state, float2 uv, float2 texel, float2 offset) {
    float2 at = uv + offset * texel;
    return fluidDivergenceOf(fluidVelocity(state, at, texel, float2(1.0f, 0.0f)), fluidVelocity(state, at, texel, float2(-1.0f, 0.0f)),
                             fluidVelocity(state, at, texel, float2(0.0f, 1.0f)), fluidVelocity(state, at, texel, float2(0.0f, -1.0f)));
}

float4 fluidStep(texture2d<float> state, float2 uv, float2 source, float2 push, float force, float radius,
                 float viscosity, float damping, float dissipation) {
    float2 texel = texelOf(state);
    float4 here = imageTap(state, uv);
    if (here.a < 0.5f) return fluidStartState();
    float2 aspect = fluidAspect(float(state.get_width()), float(state.get_height()));
    float2 origin = fluidAdvectionOrigin(uv, fluidVelocityOf(here.rg), texel);
    float4 moved = imageTap(state, origin);
    float2 moving = fluidViscousVelocity(fluidVelocityOf(moved.rg),
                                         fluidVelocity(state, origin, texel, float2(1.0f, 0.0f)), fluidVelocity(state, origin, texel, float2(-1.0f, 0.0f)),
                                         fluidVelocity(state, origin, texel, float2(0.0f, 1.0f)), fluidVelocity(state, origin, texel, float2(0.0f, -1.0f)),
                                         viscosity);
    moving = fluidDampedVelocity(moving,
                                 fluidDivergence(state, origin, texel, float2(1.0f, 0.0f)), fluidDivergence(state, origin, texel, float2(-1.0f, 0.0f)),
                                 fluidDivergence(state, origin, texel, float2(0.0f, 1.0f)), fluidDivergence(state, origin, texel, float2(0.0f, -1.0f)),
                                 damping);
    return fluidNextState(moving, moved.b, push, force, fluidFalloff(uv, source, aspect, radius), dissipation);
}


)metal";
}

} // namespace

std::string shaderSource(const std::vector<std::string>& body, const std::string& color,
                         const std::optional<std::string>& feedback, int textureCount) {
    std::string textureArguments;
    for (int index = 0; index < textureCount; index += 1) {
        textureArguments += ",\n    texture2d<float> uTexture" + std::to_string(index) + " [[texture("
                          + std::to_string(index) + ")]]";
    }
    std::string statements;
    for (const std::string& line : body) statements += "    " + line + "\n";
    std::string resultType = feedback ? "GraphOutput" : "float4";
    std::string resultValue = feedback ? "GraphOutput { clamp(" + color + ", 0.0f, 1.0f), " + *feedback + " }"
                                       : "clamp(" + color + ", 0.0f, 1.0f)";
    return prelude() + shaderLibrarySource() + textureHelpers() + imageEffectHelpers() + simulationHelpers()
                         + functionHelpers() + R"metal(
struct GraphOutput {
    float4 color [[color(0)]];
    float4 feedback [[color(1)]];
};

)metal" + "fragment " + resultType + " " + ShaderCompiler::fragmentFunction() + R"metal((
    VertexOut in [[stage_in]],
    constant Uniforms& u [[buffer(0)]])metal" + textureArguments + R"metal(
) {
    constexpr sampler linearSampler(filter::linear, address::clamp_to_edge);
    constexpr sampler repeatSampler(filter::linear, address::repeat);
    constexpr sampler nearestSampler(filter::nearest, address::clamp_to_edge);
    float2 vUv = in.uv;
    float2 fragCoord = float2(in.position.x, u.resolution.y - in.position.y);
    float2 centredUv = (vUv - float2(0.5f)) * float2(u.resolution.x / max(u.resolution.y, 1.0f), 1.0f);
    (void)linearSampler; (void)repeatSampler; (void)nearestSampler; (void)fragCoord; (void)centredUv;
)metal" + statements + "    return " + resultValue + ";\n}\n";
}

} // namespace editor::shaders::metalBackend
