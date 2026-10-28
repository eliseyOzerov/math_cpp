// Compiles Kernels/ShaderSimulation.h as Metal Shading Language (`make kernels`). Every function is called here.
#include <metal_stdlib>

#include "../../Kernels/ShaderSimulation.h"

using namespace editor::kernels;

kernel void shaderSimulationCheck(device float* results [[buffer(0)]], uint index [[thread_position_in_grid]]) {
    float x = float(index) / 64.0f;
    Vector2Of<float> uv(x, 1.0f - x);
    Vector2Of<float> texel(0.01f, 0.02f);
    Vector2Of<float> aspect(1.5f, 1.0f);
    Vector2Of<float> pair(x, 0.3f);
    int count = int(index % 9);

    Vector4Of<float> start = grayScottStartState(uv, texel);
    Vector2Of<float> laplacian = grayScottLaplacian(pair, uv, aspect);
    Vector2Of<float> reacted = grayScottReaction(pair, laplacian, 0.04f, 0.06f, 1.0f, 0.5f, 1.0f);
    Vector2Of<float> brushed = grayScottBrushed(reacted, uv, Vector2Of<float>(0.5f), 1.0f, texel);
    Vector4Of<float> view = grayScottViewColor(Vector4Of<float>(brushed.x, brushed.y, 0.0f, 1.0f), Vector3Of<float>(0.0f), Vector3Of<float>(1.0f));
    results[index * 8 + 0] = grayScottEdgeWeight() + grayScottCornerWeight() + start.y + view.x + view.w;

    Vector2Of<float> step = lifeCellStep(8.0f, Vector2Of<float>(512.0f, 256.0f));
    Vector2Of<float> centreUv = lifeCellCentreUv(uv, step);
    Vector4Of<float> lifeStart = lifeStartColor(uv, step, 0.3f);
    Vector2Of<float> neighbour = lifeNeighbourPoint(centreUv, -1, 1, step);
    results[index * 8 + 1] = centreUv.x + lifeStart.x + neighbour.y + (lifeAdvances(float(index), 3.0f) ? 1.0f : 0.0f)
        + (lifeCellIsAlive(x) ? 1.0f : 0.0f) + (lifeRule(index % 2 == 0, count) ? 1.0f : 0.0f);
    Vector4Of<float> next = lifeNextState(Vector4Of<float>(x, 0.2f, 0.7f, 1.0f), count, 0.9f);
    Vector4Of<float> lifeView = lifeViewColor(next, Vector3Of<float>(1.0f), Vector3Of<float>(0.1f, 0.1f, 0.2f));
    results[index * 8 + 2] = next.x + next.y + next.z + lifeView.x + lifeView.z;

    Vector2Of<float> velocity = fluidVelocityOf(uv);
    float divergence = fluidDivergenceOf(velocity, uv, pair, aspect);
    Vector2Of<float> origin = fluidAdvectionOrigin(uv, velocity, texel);
    Vector2Of<float> viscous = fluidViscousVelocity(velocity, pair, uv, aspect, texel, 0.3f);
    Vector2Of<float> damped = fluidDampedVelocity(viscous, divergence, 0.1f, 0.2f, 0.3f, 0.5f);
    float falloff = fluidFalloff(uv, Vector2Of<float>(0.5f), fluidAspect(640.0f, 480.0f), 0.1f);
    Vector4Of<float> fluidNext = fluidNextState(damped, 0.4f, Vector2Of<float>(0.3f, -0.2f), 2.0f, falloff, 0.99f);
    Vector4Of<float> fluidView = fluidViewColor(fluidNext, Vector3Of<float>(0.2f, 0.5f, 1.0f));
    results[index * 8 + 3] = origin.x + fluidStartState().x + fluidNext.x + fluidNext.z + fluidView.y;

    Vector2Of<float> random(0.3f, 0.6f);
    Vector2Of<float> random2(0.8f, 0.1f);
    Vector3Of<float> placed = particlePosition(random, random2, aspect, 1.0f, x * 10.0f, 1.0f, int(index % 4));
    results[index * 8 + 4] = placed.x + placed.y + placed.z + float(particleMaximumCount());
    Vector4Of<float> field = particleField(uv, aspect, 12.0f, 0.1f, 1.0f, x * 5.0f, 3.0f, int(index % 4), 1.0f);
    results[index * 8 + 5] = field.x + field.y + field.z + field.w;
}
