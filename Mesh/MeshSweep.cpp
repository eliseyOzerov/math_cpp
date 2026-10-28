// Sweep: a flat ring dragged along a 3D curve (Engine/Geometry/Mesh+Sweep.swift). The doc
// comments are on the declarations in Mesh.h; the generic part, which needs the curve, is the
// template `Mesh::sweep` at the end of that file, and the rest is here.
#include <string>

#include "Mesh.h"

namespace editor {

std::vector<Vector2> Mesh::circleProfile(float radius, int segments) {
    if (!(std::isfinite(radius) && radius > 0)) throw GeometryProblem("A circle profile needs a finite positive radius.");
    if (segments < MeshOperationLimits::minimumSweepProfilePoints || segments > MeshOperationLimits::maximumSweepProfilePoints) {
        throw GeometryProblem(
            "A circle profile needs " + std::to_string(MeshOperationLimits::minimumSweepProfilePoints) + "..."
            + std::to_string(MeshOperationLimits::maximumSweepProfilePoints) + " segments, but got " + std::to_string(segments) + ".");
    }
    std::vector<Vector2> corners;
    for (int step = 0; step < segments; ++step) {
        Angle angle = turn(step, segments);
        corners.push_back(Vector2(radius * angle.cosine(), radius * angle.sine()));
    }
    return corners;
}

void Mesh::checkSweepSize(size_t profilePointCount, int samples) {
    if (profilePointCount < static_cast<size_t>(MeshOperationLimits::minimumSweepProfilePoints)
        || profilePointCount > static_cast<size_t>(MeshOperationLimits::maximumSweepProfilePoints)) {
        throw GeometryProblem(
            "A sweep profile needs " + std::to_string(MeshOperationLimits::minimumSweepProfilePoints) + "..."
            + std::to_string(MeshOperationLimits::maximumSweepProfilePoints) + " points, but has " + std::to_string(profilePointCount) + ".");
    }
    if (samples < MeshOperationLimits::minimumSweepSamples || samples > MeshOperationLimits::maximumSweepSamples) {
        throw GeometryProblem(
            "A sweep needs " + std::to_string(MeshOperationLimits::minimumSweepSamples) + "..."
            + std::to_string(MeshOperationLimits::maximumSweepSamples) + " samples, but got " + std::to_string(samples) + ".");
    }
}

Mesh Mesh::sweepAlongFrames(
    const std::vector<Vector2>& profile, const std::vector<CurveFrame>& frames, const std::vector<float>& scales, bool capped) {
    int samples = static_cast<int>(frames.size());
    int ringSize = static_cast<int>(profile.size());
    bool startIsTip = scales[0] == 0;
    bool endIsTip = scales[samples - 1] == 0;

    // Points: a tip is one point, any other ring is `ringSize` points. `firstPoint[i]` is the
    // number of the first point of ring i, so ring i point j is `firstPoint[i] + j`; a tip's
    // only point is `firstPoint[i]`.
    Mesh mesh;
    std::vector<int> firstPoint;
    for (int sample = 0; sample < samples; ++sample) {
        firstPoint.push_back(static_cast<int>(mesh.positions.size()));
        const CurveFrame& frame = frames[sample];
        if (scales[sample] == 0) {
            mesh.positions.push_back(frame.position);
        } else {
            for (Vector2 corner : profile) {
                Vector3 offset = frame.normal * corner.x + frame.binormal * corner.y;
                mesh.positions.push_back(frame.position + offset * scales[sample]);
            }
        }
    }
    auto isTip = [&](int sample) { return scales[sample] == 0; };
    auto pointNumber = [&](int ring, int corner) { return isTip(ring) ? firstPoint[ring] : firstPoint[ring] + corner % ringSize; };

    for (int ring = 0; ring < samples - 1; ++ring) {
        for (int corner = 0; corner < ringSize; ++corner) {
            std::vector<int> quad = {
                pointNumber(ring, corner), pointNumber(ring, corner + 1),
                pointNumber(ring + 1, corner + 1), pointNumber(ring + 1, corner),
            };
            // A collapsed ring repeats one point twice in a row: the quad becomes the
            // triangle that meets at the tip.
            if (std::optional<Face> face = Face::droppingRepeatedCorners(quad)) mesh.faces.push_back(*face);
        }
    }
    if (capped) {
        if (!startIsTip) {
            Face cap;
            for (int offset = ringSize - 1; offset >= 0; --offset) cap.corners.push_back(firstPoint[0] + offset);
            mesh.faces.push_back(cap);
        }
        if (!endIsTip) {
            Face cap;
            for (int offset = 0; offset < ringSize; ++offset) cap.corners.push_back(firstPoint[samples - 1] + offset);
            mesh.faces.push_back(cap);
        }
    }
    return mesh;
}

} // namespace editor
