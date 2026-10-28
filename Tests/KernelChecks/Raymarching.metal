// Compiles Kernels/Raymarching.h as Metal Shading Language (`make kernels`). Every function is
// called here, with a struct as the scene (Metal has no lambdas).
#include <metal_stdlib>

#include "../../Kernels/Raymarching.h"

using namespace editor::kernels;

struct UnitSphere {
    float operator()(Vector3Of<float> p) const { return lengthOf(p) - 1.0f; }
};

struct UnitCloud {
    float operator()(Vector3Of<float> p) const { return 1.0f - lengthOf(p); }
};

kernel void raymarchingCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f - 0.5f;
    UnitSphere sphere;
    Vector3Of<float> origin(x, 0.0f, -3.0f);
    Vector3Of<float> forward(0.0f, 0.0f, 1.0f);
    Vector4Of<float> color(0.8f, 0.4f, 0.2f, 1.0f);

    SphereTraceResult hit = sphereTrace(sphere, origin, forward, 20.0f, 64.0f, 0.001f);
    Vector3Of<float> normal = surfaceNormal(sphere, hit.position, 0.001f);
    results[index * 16 + 0] = hit.travel + hit.stepsUsed + (hit.hit ? 1.0f : 0.0f) + normal.x + normal.y + normal.z;
    Vector4Of<float> shaded = shadedSurface(sphere, hit, color, Vector3Of<float>(1.0f, 1.0f, -1.0f), 0.2f, color, 0.001f);
    results[index * 16 + 1] = shaded.x + shaded.y + shaded.z + shaded.w;
    results[index * 16 + 2] = lambertDiffuse(normal, forward) + phongSpecular(normal, forward, -forward, 16.0f)
        + blinnPhongSpecular(normal, forward, -forward, 16.0f) + fresnelRim(normal, forward, 3.0f);
    results[index * 16 + 3] = ambientOcclusion(sphere, hit.position, normal, 0.3f, 1.0f)
        + softShadow(sphere, hit.position, normal, Vector3Of<float>(0.0f, 1.0f, -1.0f), 8.0f, 10.0f);
    Vector4Of<float> fogged = distanceFog(color, hit.travel, color, 0.2f, int(index % 3));
    Vector4Of<float> sky = skyColor(forward, color, color, color, normal, 0.1f);
    results[index * 16 + 4] = fogged.x + sky.x;
    Vector3Of<float> ray = lookAtCameraRay(Vector2Of<float>(x, 0.1f), origin, Vector3Of<float>(0.0f), 50.0f);
    Vector3Of<float> orbit = orbitCameraOrigin(Vector3Of<float>(0.0f), 30.0f, 20.0f, 4.0f);
    results[index * 16 + 5] = ray.x + orbit.x + reflectedAbout(ray, normal).y + tangent(x);
    UnitCloud cloud;
    Vector2Of<float> volume = densityMarch(cloud, origin, forward, 6.0f, 32.0f, 1.0f);
    results[index * 16 + 6] = volume.x + volume.y;
}
