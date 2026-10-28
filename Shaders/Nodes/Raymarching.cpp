// Raymarching: shapes as distances in three dimensions, a camera, the march itself and the
// lighting that goes with it (graphics_metal's Nodes/Raymarching.swift). Helper names are the
// kernels' (Kernels/Raymarching.h, Kernels/ShaderDistances.h); graphics_metal's name is in each
// kernel's Tags line.
//
// A scene is a number input of a Raymarch node that is a function of the point being marched
// (March Point): wire a 3D shape into it and it is compiled to a lambda of the point (the
// compiler's scoped inputs, as Loop's body is) and handed to the kernel as its `Scene`. Shapes
// read March Point unless a position is wired, so one shape wired to a Raymarch node needs no
// other wire. Previews find a host for them through that fallback (NodePreviews.cpp).
#include "NodeFamilies.h"

namespace editor::shaders {

using namespace spec;
using enum SocketType;

namespace {

/// The scene input of a node that samples the scene: a number, of the point.
InputSpec sceneIn(const std::string& id = "scene", const std::string& label = "Scene", double initial = 1) {
    return scoped(num(id, label, initial), {{"float3", "marchPoint"}});
}

InputSpec position(std::optional<Fallback> fallback = Fallback::marchPoint()) {
    return vec3("position", "Position", std::move(fallback));
}

Fallback skyFallback(const std::string& code, const std::string& label) { return Fallback{code, label, std::nullopt}; }

NodeDefinition shape3(const NodeBuilder& c, const std::string& name, const std::string& label, const std::string& summary,
                      const std::string& symbol, std::vector<InputSpec> inputs, const std::string& templateText) {
    inputs.insert(inputs.begin(), position());
    return c.simple(name, label, summary, std::move(inputs), number, templateText,
                    {.symbol = symbol, .output = {"distance", "Distance"}});
}

} // namespace

NodeGroup raymarchingNodes() {
    NodeBuilder c(NodeCategory::raymarching);
    return {c.category, {
        // Rays and cameras.
        c.simple("ray-origin", "Ray Origin", "Where rays start: the camera's position.",
                 {num("x", "X", 0), num("y", "Y", 0), num("z", "Z", -3)}, vector3, "float3({x}, {y}, {z})",
                 {.symbol = "camera.metering.center.weighted", .output = {"origin", "Origin"}}),
        c.simple("ray-direction", "Ray Direction", "The direction of the ray through each pixel, for a camera looking along +Z.",
                 {vec2("coordinates", "Coordinates", Fallback::centred()), num("fov", "Field of View °", 37, between(10, 120))}, vector3,
                 "lookAtCameraRay({coordinates}, float3(0.0f), float3(0.0f, 0.0f, 1.0f), {fov})",
                 {.symbol = "arrow.up.right", .output = {"direction", "Direction"}}),
        c.node({.name = "orbit-camera", .label = "Orbit Camera", .summary = "A camera circling a target: its position and the ray through each pixel.",
                .symbol = "camera.rotate",
                .inputs = {vec2("coordinates", "Coordinates", Fallback::centred()), vec3("target", "Target"),
                           num("yaw", "Yaw °", 0, between(-180, 180)), num("pitch", "Pitch °", 15, between(-89, 89)),
                           num("radius", "Distance", 3, between(0.1, 20)), num("fov", "Field of View °", 37, between(10, 120))},
                .outputs = {out("origin", "Origin", vector3, "orbitCameraOrigin({target}, {yaw}, {pitch}, {radius})"),
                            out("direction", "Direction", vector3,
                                "lookAtCameraRay({coordinates}, orbitCameraOrigin({target}, {yaw}, {pitch}, {radius}), {target}, {fov})")}}),
        c.node({.name = "look-at-camera", .label = "Look-At Camera",
                .summary = "A camera at a position looking at a target: its position and the ray through each pixel.",
                .symbol = "camera.viewfinder",
                .inputs = {vec2("coordinates", "Coordinates", Fallback::centred()), num("x", "X", 0), num("y", "Y", 0.5), num("z", "Z", -3),
                           vec3("target", "Target"), num("fov", "Field of View °", 37, between(10, 120))},
                .outputs = {out("origin", "Origin", vector3, "float3({x}, {y}, {z})"),
                            out("direction", "Direction", vector3, "lookAtCameraRay({coordinates}, float3({x}, {y}, {z}), {target}, {fov})")}}),

        // 3D shapes.
        shape3(c, "sphere-3d", "Sphere", "The distance to a sphere.", "circle.fill",
               {num("radius", "Radius", 1)}, "sphereDistance({position}, {radius})"),
        shape3(c, "box-3d", "Box", "The distance to a box, with optional rounded edges.", "cube",
               {num("width", "Width", 1.4), num("height", "Height", 1.4), num("depth", "Depth", 1.4), num("rounding", "Rounding", 0.1)},
               "boxDistance({position}, 0.5f * abs(float3({width}, {height}, {depth})), {rounding})"),
        shape3(c, "torus-3d", "Torus", "The distance to a torus lying flat.", "circle.circle",
               {num("major", "Radius", 0.8), num("minor", "Tube Radius", 0.3)}, "torusDistance({position}, {major}, {minor})"),
        shape3(c, "cylinder-3d", "Cylinder", "The distance to a cylinder standing on the y axis.", "cylinder",
               {num("radius", "Radius", 0.6), num("height", "Height", 1.4)}, "cylinderDistance({position}, {radius}, {height})"),
        shape3(c, "cone-3d", "Cone", "The distance to a cone pointing up.", "cone",
               {num("radius", "Base Radius", 0.8), num("height", "Height", 1.6)}, "coneDistance({position}, {radius}, {height})"),
        c.simple("plane-3d", "Plane", "The distance to an infinite plane facing a direction.",
                 {position(), vec3("normal", "Normal", Fallback::up()), num("offset", "Offset", 1)}, number,
                 "planeDistance({position}, {normal}, {offset})",
                 {.symbol = "square.3.layers.3d.down.right", .output = {"distance", "Distance"}}),
        shape3(c, "capsule-3d", "Capsule", "The distance to an upright capsule.", "capsule.portrait",
               {num("radius", "Radius", 0.4), num("height", "Height", 1)}, "capsuleDistance({position}, {radius}, {height})"),

        // Moving the point a shape is measured at.
        c.node({.name = "march-point", .label = "March Point", .summary = "Inside a Raymarch node's scene: the point being measured.",
                .symbol = "scope", .outputs = {out("point", "Point", vector3, "marchPoint")}}),
        c.simple("sdf-translate-3d", "Translate Position", "Moves what a shape measures: feed the result to its Position.",
                 {position(), num("x", "X", 0), num("y", "Y", 0), num("z", "Z", 0)}, vector3,
                 "translatedPoint({position}, float3({x}, {y}, {z}))",
                 {.symbol = "arrow.up.and.down.and.arrow.left.and.right", .output = {"position", "Position"}}),
        c.simple("sdf-rotate-3d", "Rotate Position", "Rotates what a shape measures by angles about x, y and z.",
                 {position(), num("x", "X °", 0), num("y", "Y °", 0), num("z", "Z °", 0)}, vector3,
                 "rotatedPoint({position}, float3({x}, {y}, {z}))", {.symbol = "rotate.3d", .output = {"position", "Position"}}),
        c.node({.name = "sdf-repeat-3d", .label = "Repeat Position",
                .summary = "Repeats space in a grid, infinitely or a limited number of cells; feed it to a shape's Position.",
                .symbol = "cube.transparent",
                .inputs = {position(), num("spacing", "Spacing", 2), num("limit", "Limit", 0, between(0, 16))},
                .outputs = {out("position", "Position", vector3, "repeatedPoint({position}, {spacing}, {limit})"),
                            out("cell", "Cell", vector3, "repeatedCell3({position}, {spacing}, {limit})")}}),
        c.simple("sdf-twist-3d", "Twist Position", "Twists space around the y axis, more with height.",
                 {position(), num("amount", "Degrees per Unit", 30, between(-180, 180))}, vector3,
                 "twistedPoint({position}, {amount})", {.symbol = "tornado", .output = {"position", "Position"}}),

        // The march.
        c.node({.name = "raymarch", .label = "Raymarch", .summary = "Marches rays through a scene (a shape wired into Scene) and lights what they hit.",
                .symbol = "cube.transparent.fill",
                .inputs = {sceneIn(), vec3("origin", "Ray Origin", Fallback::rayOrigin()), vec3("direction", "Ray Direction", Fallback::rayDirection()),
                           num("maxDistance", "Max Distance", 20, between(1, 100)), num("steps", "Steps", 64, between(8, 96)),
                           num("epsilon", "Precision", 0.001, between(0.0001, 0.05)), col("albedo", "Color", Fallback::white()),
                           vec3("light", "Light Direction", Fallback::sun()), num("ambient", "Ambient", 0.2, between(0, 1)),
                           col("background", "Background", Fallback::black())},
                .outputs = {out("color", "Color", color, "shadedSurface({scene}, {$}, {albedo}, {light}, {ambient}, {background}, {epsilon})"),
                            out("distance", "Distance", number, "{$}.travel"),
                            out("depth", "Depth", number, "clamp({$}.travel / max({maxDistance}, 0.001f), 0.0f, 1.0f)"),
                            out("hit", "Hit", number, "float({$}.hit)"),
                            out("position", "Position", vector3, "{$}.position"),
                            out("normal", "Normal", vector3, "({$}.hit ? surfaceNormal({scene}, {$}.position, {epsilon}) : float3(0.0f))"),
                            out("steps", "Steps Used", number, "{$}.stepsUsed")},
                .setup = "sphereTrace({scene}, {origin}, {direction}, {maxDistance}, {steps}, {epsilon})"}),
        c.simple("normal-estimate", "Normal Estimate", "The surface direction of a scene at a position, from central differences.",
                 {sceneIn(), vec3("position", "Position"), num("epsilon", "Precision", 0.001, between(0.0001, 0.05))}, vector3,
                 "surfaceNormal({scene}, {position}, {epsilon})", {.symbol = "arrow.up.forward", .output = {"normal", "Normal"}}),

        // Lighting.
        c.simple("lambert", "Lambert", "Diffuse light: how directly a surface faces the light.",
                 {vec3("normal", "Normal", Fallback::up()), vec3("light", "Light Direction", Fallback::sun())}, number,
                 "lambertDiffuse({normal}, {light})", {.symbol = "sun.max", .output = {"diffuse", "Diffuse"}}),
        c.simple("phong", "Phong", "The Phong specular highlight.",
                 {vec3("normal", "Normal", Fallback::up()), vec3("direction", "Ray Direction", Fallback::rayDirection()),
                  vec3("light", "Light Direction", Fallback::sun()), num("shininess", "Shininess", 32, between(1, 256))}, number,
                 "phongSpecular({normal}, {direction}, {light}, {shininess})", {.symbol = "sparkles", .output = {"specular", "Specular"}}),
        c.simple("blinn-phong", "Blinn-Phong", "The Blinn-Phong specular highlight, with a softer, rounder spread.",
                 {vec3("normal", "Normal", Fallback::up()), vec3("direction", "Ray Direction", Fallback::rayDirection()),
                  vec3("light", "Light Direction", Fallback::sun()), num("shininess", "Shininess", 64, between(1, 256))}, number,
                 "blinnPhongSpecular({normal}, {direction}, {light}, {shininess})", {.symbol = "sparkle", .output = {"specular", "Specular"}}),
        c.simple("fresnel", "Fresnel", "Brighter where a surface is seen at a grazing angle.",
                 {vec3("normal", "Normal", Fallback::up()), vec3("direction", "Ray Direction", Fallback::rayDirection()),
                  num("power", "Power", 3, between(0, 10))}, number,
                 "fresnelRim({normal}, {direction}, {power})", {.symbol = "circle.dotted", .output = {"fresnel", "Fresnel"}}),
        c.simple("ambient-occlusion", "Ambient Occlusion", "Darkens creases and corners: 1 where open, less near other surfaces.",
                 {sceneIn(), position(std::nullopt), vec3("normal", "Normal", Fallback::up()), num("radius", "Radius", 0.5),
                  num("strength", "Strength", 1, between(0, 3))}, number,
                 "ambientOcclusion({scene}, {position}, {normal}, {radius}, {strength})",
                 {.symbol = "circle.lefthalf.filled", .output = {"occlusion", "Occlusion"}}),
        c.simple("soft-shadow", "Soft Shadow", "How much light reaches a position: 1 in the open, 0 in shadow, soft at the edge.",
                 {sceneIn(), position(std::nullopt), vec3("normal", "Normal", Fallback::up()), vec3("light", "Light Direction", Fallback::sun()),
                  num("softness", "Hardness", 8, between(1, 64)), num("maxDistance", "Max Distance", 10, between(1, 50))}, number,
                 "softShadow({scene}, {position}, {normal}, {light}, {softness}, {maxDistance})",
                 {.symbol = "cloud.sun", .output = {"shadow", "Light"}}),
        c.simple("fog", "Fog", "Blends a colour toward the fog colour with distance.",
                 {col("color", "Color", Fallback::black()), num("distance", "Distance", 0), col("fogColor", "Fog Color", Fallback::gray()),
                  num("density", "Density", 0.1, between(0, 2))}, color,
                 "distanceFog({color}, {distance}, {fogColor}, {density}, {#mode})",
                 {.symbol = "cloud.fog", .options = {option("mode", "Falloff", {"Exponential", "Exponential²", "Linear"})}}),
        c.simple("sky-gradient", "Sky Gradient", "An environment colour for a direction: ground, horizon, zenith and a sun.",
                 {vec3("direction", "Direction", Fallback::rayDirection()),
                  col("horizon", "Horizon", skyFallback("float4(0.85f, 0.9f, 1.0f, 1.0f)", "horizon")),
                  col("zenith", "Zenith", skyFallback("float4(0.25f, 0.45f, 0.85f, 1.0f)", "zenith")),
                  col("ground", "Ground", skyFallback("float4(0.25f, 0.22f, 0.2f, 1.0f)", "ground")),
                  vec3("sun", "Sun Direction", Fallback::sun()), num("sunSize", "Sun Size", 0.01, between(0.001, 0.2))}, color,
                 "skyColor({direction}, {horizon}, {zenith}, {ground}, {sun}, {sunSize})", {.symbol = "sunset"}),
        c.node({.name = "density-accumulate", .label = "Density Accumulate",
                .summary = "Marches through a density (wired into Density) and gathers how much of the ray it absorbs.",
                .symbol = "cloud",
                .inputs = {sceneIn("density", "Density", 0.3), vec3("origin", "Ray Origin", Fallback::rayOrigin()),
                           vec3("direction", "Ray Direction", Fallback::rayDirection()), num("maxDistance", "Max Distance", 6, between(0.1, 50)),
                           num("steps", "Steps", 32, between(1, 64)), num("absorption", "Absorption", 1, between(0, 10)),
                           col("tint", "Color", Fallback::white())},
                .outputs = {out("color", "Color", color, "float4({tint}.rgb, 1.0f - {$}.y)"),
                            out("density", "Optical Depth", number, "{$}.x"),
                            out("transmittance", "Transmittance", number, "{$}.y")},
                .setup = "densityMarch({density}, {origin}, {direction}, {maxDistance}, {steps}, {absorption})"}),
    }};
}

} // namespace editor::shaders
