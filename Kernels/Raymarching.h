// The shader library's ray marching, ported from graphics_metal's Raymarch.metal: sphere tracing
// against a scene function (a point to the distance to the nearest surface), and what is built on
// it: normals, shading, soft shadows, ambient occlusion, fog, sky, a look-at camera and a march
// through a density field.
//
// The scene arrives as a callable, so every function that evaluates it is a template over its type
// `Scene` (`F` in graphics_metal): any object with `float operator()(Vector3Of<float>) const`. That
// is allowed in kernels: a template parameter on a callable is resolved at compile time, it is not a
// function pointer. On the CPU the callable may be a C++ lambda. Metal's compiler (metal3.0, as
// `make kernels` runs it) rejects lambda expressions ("lambda expressions are not supported in
// Metal"), so on the GPU the callable is a small struct with an `operator()`; the header cares
// about neither. The callable is passed by copy, and must be cheap to copy (capture little).
//
// Name map from graphics_metal (each function's Tags line repeats its old name):
//   MarchResult → SphereTraceResult, marchRay → sphereTrace, marchNormal → surfaceNormal,
//   marchShade → shadedSurface, marchLambert → lambertDiffuse, marchPhong → phongSpecular,
//   marchBlinnPhong → blinnPhongSpecular, marchFresnel → fresnelRim, marchOcclusion →
//   ambientOcclusion, marchShadow → softShadow, marchFog → distanceFog, marchSky → skyColor,
//   marchCameraRay → lookAtCameraRay, marchOrbitOrigin → orbitCameraOrigin, marchDensity →
//   densityMarch.
//
// Differences from graphics_metal, all on purpose: `mix`, `clamp`, `normalize`, `dot`, `cross`,
// `pow`, `exp` and the others are the prelude's and KernelVectors.h's names (`linearBlend`,
// `clamped`, `normalized`, `dotProduct`, `crossProduct`, `power`, `exponential`), colour
// swizzles (`.rgb`) are written out, the struct is filled field by field, and `reflect` is written
// as `light direction − 2 (n · l) n` in `phongSpecular`. Nothing was left out.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// What a sphere trace found: the point reached, the distance travelled, how much of the step
/// budget was used and whether a surface was hit.
///
/// Use when: reading the result of `sphereTrace`
/// Tags: MarchResult, ray hit, trace result
/// Node: none (shader library)
struct SphereTraceResult {
    Vector3Of<float> position;
    float travel;
    float stepsUsed;
    bool hit;
};

/// Sphere tracing: walks a ray forward by the scene's distance at each point until it is within a
/// small tolerance of a surface, or runs out of steps or distance.
///
/// Source: John C. Hart, "Sphere tracing: a geometric method for the antialiased ray tracing of
/// implicit surfaces", 1996.
///
/// How: ray = normalize(direction + (0, 0, 10⁻⁷)); steps = clamp(round-half-up(stepCount), 1, 96);
/// t starts at 0; each step p = origin + ray t, d = scene(p); a hit when d < tolerance (1 + 0.2 t)
/// with tolerance = max(epsilon, 0.00001) (the tolerance grows with distance, as a pixel does); else t += d,
/// and the march stops when t > max(maxDistance, 0.001).
///
/// Conventions: the scene is a signed distance (negative inside); a hit returns the point, t, and
/// i / steps (the share of steps spent) with `hit` true; a miss returns the origin, the limit
/// distance, 1 and `hit` false. A zero direction becomes +z. The 10⁻⁷ nudge only keeps
/// normalisation finite; it moves a unit direction by under 10⁻⁷.
///
/// Use when: rendering implicit surfaces and SDF scenes; picking along a ray
/// Tags: marchRay, ray march, sphere tracing, SDF render, raymarching
/// Node: none (shader library)
template <typename Scene>
inline SphereTraceResult sphereTrace(Scene scene, Vector3Of<float> origin, Vector3Of<float> direction, float maxDistance, float stepCount, float epsilon) {
    Vector3Of<float> ray = normalized(direction + Vector3Of<float>(0.0f, 0.0f, 0.0000001f));
    int steps = clamped(static_cast<int>(roundedDown(stepCount + 0.5f)), 1, 96);
    float limit = largerOf(maxDistance, 0.001f);
    float tolerance = largerOf(epsilon, 0.00001f);
    float t = 0.0f;
    SphereTraceResult result;
    result.position = origin;
    result.travel = limit;
    result.stepsUsed = 1.0f;
    result.hit = false;
    for (int i = 0; i < steps; i++) {
        Vector3Of<float> p = origin + ray * t;
        float d = scene(p);
        if (d < tolerance * (1.0f + 0.2f * t)) {
            result.position = p;
            result.travel = t;
            result.stepsUsed = static_cast<float>(i) / static_cast<float>(steps);
            result.hit = true;
            return result;
        }
        t += d;
        if (t > limit) { break; }
    }
    return result;
}

/// The surface's direction at `point`, from central differences of the scene.
///
/// How: e = max(epsilon, 0.0001); n = (scene(p + e x) − scene(p − e x), scene(p + e y) − scene(p − e y),
/// scene(p + e z) − scene(p − e z)); the result is normalize(n + (0, 10⁻⁷, 0)).
///
/// Conventions: points out of the surface where the scene is a signed distance (positive outside);
/// the 10⁻⁷ nudge makes a zero gradient give +y instead of NaN.
///
/// Use when: lighting a hit point; reflecting and refracting rays
/// Tags: marchNormal, SDF normal, gradient normal, central differences
/// Node: none (shader library)
template <typename Scene>
inline Vector3Of<float> surfaceNormal(Scene scene, Vector3Of<float> point, float epsilon) {
    float e = largerOf(epsilon, 0.0001f);
    Vector3Of<float> n(scene(point + Vector3Of<float>(e, 0.0f, 0.0f)) - scene(point - Vector3Of<float>(e, 0.0f, 0.0f)),
                       scene(point + Vector3Of<float>(0.0f, e, 0.0f)) - scene(point - Vector3Of<float>(0.0f, e, 0.0f)),
                       scene(point + Vector3Of<float>(0.0f, 0.0f, e)) - scene(point - Vector3Of<float>(0.0f, 0.0f, e)));
    return normalized(n + Vector3Of<float>(0.0f, 0.0000001f, 0.0f));
}

/// Albedo lit by a directional light with ambient, or `background` where the ray missed.
///
/// How: n = surfaceNormal at the hit; diffuse = max(n · normalize(light), 0); the colour is albedo.rgb
/// × clamp(ambient + (1 − ambient) diffuse, 0, 1) with albedo's alpha.
///
/// Conventions: `light` points towards the light; `ambient` in 0…1.
///
/// Use when: the final colour of a ray-marched pixel
/// Tags: marchShade, shading, Lambert shading, ambient light
/// Node: none (shader library)
template <typename Scene>
inline Vector4Of<float> shadedSurface(Scene scene, SphereTraceResult hit, Vector4Of<float> albedo, Vector3Of<float> light, float ambient, Vector4Of<float> background, float epsilon) {
    if (!hit.hit) { return background; }
    Vector3Of<float> normal = surfaceNormal(scene, hit.position, epsilon);
    float diffuse = largerOf(dotProduct(normal, normalized(light)), 0.0f);
    float lit = clamped(ambient + (1.0f - ambient) * diffuse, 0.0f, 1.0f);
    return Vector4Of<float>(albedo.x * lit, albedo.y * lit, albedo.z * lit, albedo.w);
}

/// Lambert's diffuse term: how directly a surface faces the light, 0 to 1.
///
/// Source: Johann Heinrich Lambert, Photometria, 1760 (the cosine law).
///
/// How: max(normalize(normal) · normalize(light), 0).
///
/// Conventions: `light` points towards the light; a surface facing away gives 0. A zero vector
/// normalises to a non-finite result on the CPU (as in Metal); callers pass real directions.
///
/// Use when: diffuse lighting; masks by facing direction
/// Tags: marchLambert, Lambert, diffuse, N dot L, cosine law
/// Node: none (shader library)
inline float lambertDiffuse(Vector3Of<float> normal, Vector3Of<float> light) {
    return largerOf(dotProduct(normalized(normal), normalized(light)), 0.0f);
}

/// Phong's specular highlight.
///
/// Source: Bui Tuong Phong, "Illumination for computer generated pictures", 1975.
///
/// How: n, l normalised; r = reflect(−l, n) = −l − 2 (n · −l) n; where n · l > 0 the result is
/// max(r · −normalize(direction), 0)^max(shininess, 1), else 0.
///
/// Conventions: `direction` is the way the viewing ray travels (from the eye to the surface);
/// `light` points towards the light; the result is 0 to 1.
///
/// Use when: shiny highlights on plastic and metal
/// Tags: marchPhong, Phong, specular, highlight, shininess
/// Node: none (shader library)
inline float phongSpecular(Vector3Of<float> normal, Vector3Of<float> direction, Vector3Of<float> light, float shininess) {
    Vector3Of<float> n = normalized(normal);
    Vector3Of<float> l = normalized(light);
    Vector3Of<float> reflected = reflectedAbout(-l, n);
    return dotProduct(n, l) > 0.0f ? power(largerOf(dotProduct(reflected, -normalized(direction)), 0.0f), largerOf(shininess, 1.0f)) : 0.0f;
}

/// The Blinn–Phong specular highlight, cheaper and broader than Phong's.
///
/// Source: James F. Blinn, "Models of light reflection for computer synthesized pictures", 1977.
///
/// How: n, l normalised; halfway = normalize(l − normalize(direction)); where n · l > 0 the result
/// is max(n · halfway, 0)^max(shininess, 1), else 0.
///
/// Conventions: as `phongSpecular`.
///
/// Use when: shiny highlights where Phong's is too narrow
/// Tags: marchBlinnPhong, Blinn Phong, half vector, specular
/// Node: none (shader library)
inline float blinnPhongSpecular(Vector3Of<float> normal, Vector3Of<float> direction, Vector3Of<float> light, float shininess) {
    Vector3Of<float> n = normalized(normal);
    Vector3Of<float> l = normalized(light);
    Vector3Of<float> halfway = normalized(l - normalized(direction));
    return dotProduct(n, l) > 0.0f ? power(largerOf(dotProduct(n, halfway), 0.0f), largerOf(shininess, 1.0f)) : 0.0f;
}

/// A rim term that grows as a surface turns edge-on to the viewer: 0 facing the viewer, 1 at the
/// silhouette.
///
/// Source: the Fresnel effect, with the exponent as a shaping power as in Schlick's approximation
/// (Christophe Schlick, 1994); no refractive index is involved.
///
/// How: (1 − clamp(normalize(normal) · −normalize(direction), 0, 1))^max(power, 0).
///
/// Conventions: `direction` is the way the viewing ray travels; the result is 0 to 1.
///
/// Use when: rim lights, glassy and watery edges
/// Tags: marchFresnel, fresnel, rim light, edge glow, silhouette
/// Node: none (shader library)
inline float fresnelRim(Vector3Of<float> normal, Vector3Of<float> direction, float rimPower) {
    return power(1.0f - clamped(dotProduct(normalized(normal), -normalized(direction)), 0.0f, 1.0f), largerOf(rimPower, 0.0f));
}

/// Five samples along the normal: surfaces with nearby geometry get darker. 1 is open.
///
/// Source: Iñigo Quilez, "Ambient occlusion for raymarching" (the five-sample estimate).
///
/// How: for i = 1…5, h = radius i / 5; the sum of (h − scene(p + n h)) × 0.7^(i−1) measures how far
/// the scene is closer than the sample distance; the result is clamp(1 − strength × sum / max(radius,
/// 0.0001) × 0.6, 0, 1).
///
/// Conventions: `normal` is normalised inside; in the open the scene's distance equals h, so the sum
/// is 0 and the result 1.
///
/// Use when: contact shadows in corners and crevices of ray-marched scenes
/// Tags: marchOcclusion, ambient occlusion, AO, crevice darkening
/// Node: none (shader library)
template <typename Scene>
inline float ambientOcclusion(Scene scene, Vector3Of<float> point, Vector3Of<float> normal, float radius, float strength) {
    float occlusion = 0.0f;
    float falloff = 1.0f;
    Vector3Of<float> n = normalized(normal);
    for (int i = 1; i <= 5; i++) {
        float h = radius * static_cast<float>(i) / 5.0f;
        occlusion += (h - scene(point + n * h)) * falloff;
        falloff *= 0.7f;
    }
    return clamped(1.0f - strength * occlusion / largerOf(radius, 0.0001f) * 0.6f, 0.0f, 1.0f);
}

/// How much of the light reaches `point`: 1 in the open, 0 in the shade, soft at the edge.
///
/// Source: Iñigo Quilez, "Soft shadows in raymarched SDFs" (the minimum of k d / t along the ray).
///
/// How: origin = point + normalize(normal) 0.01; t starts at 0.02; for up to 48 steps result =
/// min(result, softness scene(origin + l t) / t), returning 0 once it is under 0.001; t advances by
/// clamp(d, 0.01, 0.5) and stops past max(maxDistance, 0.1). The result s = clamp(result, 0, 1) is
/// smoothed as s² (3 − 2s).
///
/// Conventions: `light` points towards the light; a larger `softness` gives sharper shadows.
///
/// Use when: shadows in ray-marched scenes
/// Tags: marchShadow, soft shadow, penumbra, SDF shadow
/// Node: none (shader library)
template <typename Scene>
inline float softShadow(Scene scene, Vector3Of<float> point, Vector3Of<float> normal, Vector3Of<float> light, float softness, float maxDistance) {
    Vector3Of<float> l = normalized(light);
    Vector3Of<float> origin = point + normalized(normal) * 0.01f;
    float result = 1.0f;
    float t = 0.02f;
    float limit = largerOf(maxDistance, 0.1f);
    for (int i = 0; i < 48; i++) {
        float d = scene(origin + l * t);
        result = smallerOf(result, softness * d / t);
        if (result < 0.001f) { return 0.0f; }
        t += clamped(d, 0.01f, 0.5f);
        if (t > limit) { break; }
    }
    float s = clamped(result, 0.0f, 1.0f);
    return s * s * (3.0f - 2.0f * s);
}

/// Fog thickening with `travel`: exponential (mode 0), exponential squared (1) or linear (any other).
///
/// How: d = max(density, 0); amount = 1 − e^(−d travel) (mode 0), 1 − e^(−d² travel²) (mode 1),
/// clamp(d travel × 0.1, 0, 1) (otherwise); the colour is linearBlend(color.rgb, fogColor.rgb,
/// clamp(amount, 0, 1)) with the original alpha.
///
/// Conventions: travel 0 leaves the colour unchanged; the colour approaches the fog colour as the
/// travel grows.
///
/// Use when: distance fog and haze in ray-marched scenes
/// Tags: marchFog, fog, haze, atmosphere, exponential fog
/// Node: none (shader library)
inline Vector4Of<float> distanceFog(Vector4Of<float> color, float travel, Vector4Of<float> fogColor, float density, int mode) {
    float d = largerOf(density, 0.0f);
    float amount = mode == 0 ? 1.0f - exponential(-d * travel)
                 : mode == 1 ? 1.0f - exponential(-d * d * travel * travel)
                 : clamped(d * travel * 0.1f, 0.0f, 1.0f);
    Vector3Of<float> blended = linearBlend(Vector3Of<float>(color.x, color.y, color.z),
                                           Vector3Of<float>(fogColor.x, fogColor.y, fogColor.z), clamped(amount, 0.0f, 1.0f));
    return Vector4Of<float>(blended.x, blended.y, blended.z, color.w);
}

/// Sky: ground below the horizon, horizon to zenith above it, and a soft sun in `sunDirection`.
///
/// How: d = normalize(direction), up = d.y; sky = linearBlend(horizon, zenith, clamp(up, 0, 1)^0.6);
/// sky = linearBlend(sky, ground, smoothstep(0, −0.25, up)); sun = max(d · normalize(sunDirection),
/// 0)^(1 / max(sunSize, 0.0005)); the result is sky + (1, 0.9, 0.7) sun with alpha 1.
///
/// Conventions: y is up in this function (graphics_metal's convention for the camera helpers
/// here); a smaller `sunSize` gives a tighter sun. Colours are not clamped, so a sun can pass 1.
///
/// Use when: a background for ray-marched and environment-lit scenes
/// Tags: marchSky, sky, gradient sky, sun, background
/// Node: none (shader library)
inline Vector4Of<float> skyColor(Vector3Of<float> direction, Vector4Of<float> horizon, Vector4Of<float> zenith, Vector4Of<float> ground, Vector3Of<float> sunDirection, float sunSize) {
    Vector3Of<float> d = normalized(direction);
    float up = d.y;
    Vector3Of<float> sky = linearBlend(Vector3Of<float>(horizon.x, horizon.y, horizon.z),
                                       Vector3Of<float>(zenith.x, zenith.y, zenith.z), power(clamped(up, 0.0f, 1.0f), 0.6f));
    sky = linearBlend(sky, Vector3Of<float>(ground.x, ground.y, ground.z), safeSmoothStep(0.0f, -0.25f, up));
    float sun = power(largerOf(dotProduct(d, normalized(sunDirection)), 0.0f), 1.0f / largerOf(sunSize, 0.0005f));
    Vector3Of<float> lit = sky + Vector3Of<float>(1.0f, 0.9f, 0.7f) * sun;
    return Vector4Of<float>(lit.x, lit.y, lit.z, 1.0f);
}

/// A look-at camera: the ray through the centred screen point `screen`, `fov` degrees top to bottom.
///
/// How: forward = normalize(target − origin + (0, 0, 10⁻⁷)); right = cross((0, 1, 0), forward),
/// replaced by (1, 0, 0) when it is shorter than 0.0001 (looking straight up or down), else
/// normalised; up = cross(forward, right); focal = 0.5 / tan(clamp(fov in radians, 0.05,
/// 3) / 2); the ray is normalize(forward focal + right screen.x + up screen.y).
///
/// Conventions: y is up; `screen` is centred, so (0, 0) is straight ahead and the screen's height
/// spans −0.5…0.5; `fov` in degrees, clamped to about 2.9°…172°. Looking along +z, cross((0,1,0),
/// (0,0,1)) = (1, 0, 0), so +x is to the right.
///
/// Use when: a camera for ray-marched scenes
/// Tags: marchCameraRay, camera ray, look at, perspective ray, pinhole
/// Node: none (shader library)
inline Vector3Of<float> lookAtCameraRay(Vector2Of<float> screen, Vector3Of<float> origin, Vector3Of<float> target, float fov) {
    Vector3Of<float> forward = normalized(target - origin + Vector3Of<float>(0.0f, 0.0f, 0.0000001f));
    Vector3Of<float> right = crossProduct(Vector3Of<float>(0.0f, 1.0f, 0.0f), forward);
    right = lengthOf(right) < 0.0001f ? Vector3Of<float>(1.0f, 0.0f, 0.0f) : normalized(right);
    Vector3Of<float> up = crossProduct(forward, right);
    float focal = 0.5f / tangent(clamped(degreesToRadians(fov), 0.05f, 3.0f) * 0.5f);
    return normalized(forward * focal + right * screen.x + up * screen.y);
}

/// A camera position orbiting `target`: yaw turns around the vertical axis, pitch lifts it.
///
/// How: a = yaw in radians, b = clamp(pitch, −89, 89) in radians; target + radius (sin a cos b, sin b,
/// −cos a cos b).
///
/// Conventions: degrees; y is up; yaw 0, pitch 0 puts the camera at −z of the target (looking +z).
///
/// Use when: orbit controls for ray-marched scenes
/// Tags: marchOrbitOrigin, orbit camera, turntable, arcball position
/// Node: none (shader library)
inline Vector3Of<float> orbitCameraOrigin(Vector3Of<float> target, float yaw, float pitch, float radius) {
    float a = degreesToRadians(yaw);
    float b = degreesToRadians(clamped(pitch, -89.0f, 89.0f));
    return target + Vector3Of<float>(sine(a) * cosine(b), sine(b), -cosine(a) * cosine(b)) * radius;
}

/// Marches through a density field: x is the optical depth along the ray, y the transmittance.
///
/// Source: the Beer–Lambert law (transmittance = e^(−absorption × optical depth)), integrated with a
/// midpoint rule.
///
/// How: ray = normalize(direction + (0, 0, 10⁻⁷)); steps = clamp(round-half-up(stepCount), 1, 64);
/// dt = max(maxDistance, 0.001) / steps; depth = Σ max(density(origin + ray (i + 0.5) dt), 0) dt;
/// the result is (depth, e^(−depth max(absorption, 0))).
///
/// Conventions: negative densities count as 0; no density gives (0, 1).
///
/// Use when: fog, smoke and clouds in ray-marched scenes
/// Tags: marchDensity, volume march, optical depth, transmittance, Beer-Lambert
/// Node: none (shader library)
template <typename Density>
inline Vector2Of<float> densityMarch(Density density, Vector3Of<float> origin, Vector3Of<float> direction, float maxDistance, float stepCount, float absorption) {
    Vector3Of<float> ray = normalized(direction + Vector3Of<float>(0.0f, 0.0f, 0.0000001f));
    int steps = clamped(static_cast<int>(roundedDown(stepCount + 0.5f)), 1, 64);
    float dt = largerOf(maxDistance, 0.001f) / static_cast<float>(steps);
    float depth = 0.0f;
    for (int i = 0; i < steps; i++) {
        depth += largerOf(density(origin + ray * ((static_cast<float>(i) + 0.5f) * dt)), 0.0f) * dt;
    }
    return Vector2Of<float>(depth, exponential(-depth * largerOf(absorption, 0.0f)));
}

} // namespace kernels
} // namespace editor
