// The 2D shape kernels (Kernels/ShaderShapes.h) on the CPU, against values worked out by hand from
// each function's formula, and the properties each must keep: the sign (negative inside), the
// Lipschitz bound |d(p) − d(q)| ≤ |p − q| for the exact distances, and scaling. The same header is
// compiled as Metal by `make kernels`.
#include <cmath>

#include "../Kernels/ShaderShapes.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {
using Kernel2 = kernels::Vector2Of<float>;

bool nearly(float first, float second, float tolerance = 1e-5f) { return isApproximatelyEqual(first, second, tolerance); }
bool nearly(Kernel2 first, Kernel2 second, float tolerance = 1e-5f) {
    return nearly(first.x, second.x, tolerance) && nearly(first.y, second.y, tolerance);
}

// Checks |d(p) − d(q)| ≤ |p − q| (plus `slack` for rounding) over a grid of points p, each with a
// neighbour q a fixed small step away.
template <typename Distance>
bool isLipschitzOne(Distance distance, float slack = 1e-4f) {
    const Kernel2 step(0.043f, -0.031f);
    const float stepLength = std::sqrt(0.043f * 0.043f + 0.031f * 0.031f);
    for (int i = -60; i <= 60; i += 1) {
        for (int j = -60; j <= 60; j += 1) {
            Kernel2 point(0.0371f * static_cast<float>(i) + 0.0113f, 0.0349f * static_cast<float>(j) - 0.0071f);
            if (std::fabs(distance(point) - distance(point + step)) > stepLength + slack) return false;
        }
    }
    return true;
}

// The distance from `point` to a quadratic Bézier curve found the slow way: the nearest of 4001
// sampled points on it.
float sampledBezierDistance(Kernel2 point, Kernel2 a, Kernel2 b, Kernel2 c) {
    float best = 1e30f;
    for (int index = 0; index <= 4000; index += 1) {
        float t = static_cast<float>(index) / 4000.0f;
        float u = 1.0f - t;
        Kernel2 onCurve(u * u * a.x + 2.0f * u * t * b.x + t * t * c.x, u * u * a.y + 2.0f * u * t * b.y + t * t * c.y);
        float distance = std::sqrt((onCurve.x - point.x) * (onCurve.x - point.x) + (onCurve.y - point.y) * (onCurve.y - point.y));
        best = distance < best ? distance : best;
    }
    return best;
}
} // namespace

TEST(regularPolygonDistanceIsTheRadialDepthInsideTheEdge) {
    // Square (4 sides), circumradius 1, apothem cos 45° = 0.70710678, an edge facing +x.
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.0f, 0.0f), 4.0f, 1.0f, 0.0f), 0.70710678f));
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.5f, 0.0f), 4.0f, 1.0f, 0.0f), 0.20710678f));
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.70710678f, 0.0f), 4.0f, 1.0f, 0.0f), 0.0f));
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(2.0f, 0.0f), 4.0f, 1.0f, 0.0f), -1.29289322f));
    // Along a corner direction the boundary is the circumradius: (π/4 up) at distance 1 → 0
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.70710678f, 0.70710678f), 4.0f, 1.0f, 0.0f), 0.0f, 1e-4f));
    // Hexagon: apothem cos 30° = 0.8660254; sides below 3 become 3 (apothem 0.5); negative radius is its size.
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.0f, 0.0f), 6.0f, 1.0f, 0.0f), 0.8660254f));
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.0f, 0.0f), 2.0f, 1.0f, 0.0f), 0.5f));
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(0.0f, 0.0f), -6.0f, -1.0f, 0.0f), 0.8660254f));
    // Rotating the polygon by an eighth of a turn puts a corner on +x: (1, 0) is then on the edge.
    EXPECT(nearly(kernels::regularPolygonDistance(Kernel2(1.0f, 0.0f), 4.0f, 1.0f, 0.78539816f), 0.0f, 1e-4f));
    // Property: positive inside the circumcircle's inscribed disc, negative outside the circumcircle.
    for (int step = 0; step < 100; step += 1) {
        float angle = 0.0631f * static_cast<float>(step);
        EXPECT(kernels::regularPolygonDistance(Kernel2(0.5f * std::cos(angle), 0.5f * std::sin(angle)), 5.0f, 1.0f, 0.2f) > 0.0f);
        EXPECT(kernels::regularPolygonDistance(Kernel2(1.2f * std::cos(angle), 1.2f * std::sin(angle)), 5.0f, 1.0f, 0.2f) < 0.0f);
    }
}

TEST(polygonEdgeMaskIsASoftStepAcrossTheEdge) {
    // antialiasing 0.1: depth 0 → 0.5, 0.1 → 1, −0.1 → 0, 0.05 → t = 0.75 → 0.84375
    EXPECT(nearly(kernels::polygonEdgeMask(0.0f, 0.1f, 0.0f), 0.5f));
    EXPECT(nearly(kernels::polygonEdgeMask(0.1f, 0.1f, 0.0f), 1.0f));
    EXPECT(nearly(kernels::polygonEdgeMask(-0.1f, 0.1f, 0.0f), 0.0f));
    EXPECT(nearly(kernels::polygonEdgeMask(0.05f, 0.1f, 0.0f), 0.84375f));
    // The larger of feather and pixel width wins: feather 0.2 widens the edge: 0.1 → t = 0.75
    EXPECT(nearly(kernels::polygonEdgeMask(0.1f, 0.1f, 0.2f), 0.84375f));
    // No antialiasing at all is a hard step.
    EXPECT(kernels::polygonEdgeMask(0.01f, 0.0f, 0.0f) == 1.0f);
    EXPECT(kernels::polygonEdgeMask(-0.01f, 0.0f, 0.0f) == 0.0f);
}

TEST(rotatedShapePointTurnsThePointTheOppositeWay) {
    // 90°: angle −π/2, s = −1, c = 0 → (c x − s y, s x + c y) = (y, −x): (1, 0) → (0, −1)
    EXPECT(nearly(kernels::rotatedShapePoint(Kernel2(1.0f, 0.0f), 90.0f), Kernel2(0.0f, -1.0f)));
    EXPECT(nearly(kernels::rotatedShapePoint(Kernel2(0.0f, 1.0f), 90.0f), Kernel2(1.0f, 0.0f)));
    EXPECT(nearly(kernels::rotatedShapePoint(Kernel2(0.3f, -0.4f), 0.0f), Kernel2(0.3f, -0.4f)));
    EXPECT(nearly(kernels::rotatedShapePoint(Kernel2(1.0f, 0.0f), 180.0f), Kernel2(-1.0f, 0.0f)));
    // A bar along x, rotated by 90°, lies along y: the point (0, 1) is then inside it.
    float inside = kernels::rectangleDistance(kernels::rotatedShapePoint(Kernel2(0.0f, 1.0f), 90.0f), Kernel2(1.5f, 0.2f));
    EXPECT(inside < 0.0f);
}

TEST(circleRectangleAndRoundedRectangleHaveHandWorkedDistances) {
    EXPECT(nearly(kernels::circleDistance(Kernel2(3.0f, 4.0f), 1.0f), 4.0f));
    EXPECT(nearly(kernels::circleDistance(Kernel2(0.0f, 0.0f), 2.0f), -2.0f));
    // Half size (1, 2): centre → −1 (the smaller half size); (3, 0): q = (2, −2) → 2; corner (2, 4): q = (1, 2) → √5
    EXPECT(nearly(kernels::rectangleDistance(Kernel2(0.0f, 0.0f), Kernel2(1.0f, 2.0f)), -1.0f));
    EXPECT(nearly(kernels::rectangleDistance(Kernel2(3.0f, 0.0f), Kernel2(1.0f, 2.0f)), 2.0f));
    EXPECT(nearly(kernels::rectangleDistance(Kernel2(2.0f, 4.0f), Kernel2(1.0f, 2.0f)), 2.236068f));
    // Rounded: half (1, 1), radius 0.5, (2, 0): the inner box is half 0.5: q = (1.5, −0.5) → 1.5 − 0.5 = 1
    EXPECT(nearly(kernels::roundedRectangleDistance(Kernel2(2.0f, 0.0f), Kernel2(1.0f, 1.0f), 0.5f), 1.0f));
    // The corner is cut: (1, 1) is √2 − ... inner corner (0.5, 0.5) → q = (0.5, 0.5) → 0.7071 − 0.5
    EXPECT(nearly(kernels::roundedRectangleDistance(Kernel2(1.0f, 1.0f), Kernel2(1.0f, 1.0f), 0.5f), 0.20710678f));
    // A radius above the smaller half size is limited to it: a circle of radius 1.
    EXPECT(nearly(kernels::roundedRectangleDistance(Kernel2(2.0f, 0.0f), Kernel2(1.0f, 1.0f), 5.0f), 1.0f));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::circleDistance(point, 1.1f); }));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::rectangleDistance(point, Kernel2(1.2f, 0.6f)); }));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::roundedRectangleDistance(point, Kernel2(1.2f, 0.6f), 0.3f); }));
}

TEST(segmentDistanceProjectsAndClampsToTheEnds) {
    Kernel2 start(0.0f, 0.0f);
    Kernel2 end(2.0f, 0.0f);
    // (1, 1): h = 0.5 → (1, 1) − (1, 0) → 1
    EXPECT(nearly(kernels::segmentDistance(Kernel2(1.0f, 1.0f), start, end), 1.0f));
    // (3, 1): h clamps to 1 → (1, 1) → √2; (−3, 0): h clamps to 0 → 3
    EXPECT(nearly(kernels::segmentDistance(Kernel2(3.0f, 1.0f), start, end), 1.4142135f));
    EXPECT(nearly(kernels::segmentDistance(Kernel2(-3.0f, 0.0f), start, end), 3.0f));
    // On the segment: 0; never negative
    EXPECT(nearly(kernels::segmentDistance(Kernel2(0.7f, 0.0f), start, end), 0.0f));
    // A zero-length segment is the distance to its point.
    EXPECT(nearly(kernels::segmentDistance(Kernel2(3.0f, 4.0f), start, start), 5.0f));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::segmentDistance(point, Kernel2(-1.0f, -0.5f), Kernel2(1.2f, 0.8f)); }));
}

TEST(ellipseDistanceIsExactOnTheAxesAndSignedEverywhere) {
    Kernel2 radii(2.0f, 1.0f);
    // On the edge: 0; (4, 0): k0 = 2, k1 = 1 → 2; (0, 3): k0 = 3, k1 = 3 → 3 × 2 / 3 = 2
    EXPECT(nearly(kernels::ellipseDistance(Kernel2(2.0f, 0.0f), radii), 0.0f));
    EXPECT(nearly(kernels::ellipseDistance(Kernel2(4.0f, 0.0f), radii), 2.0f));
    EXPECT(nearly(kernels::ellipseDistance(Kernel2(0.0f, 3.0f), radii), 2.0f));
    // The centre: minus the smaller radius.
    EXPECT(nearly(kernels::ellipseDistance(Kernel2(0.0f, 0.0f), radii), -1.0f));
    // Property: the sign is right: inside points negative, outside positive.
    for (int step = 0; step < 60; step += 1) {
        float angle = 0.1047f * static_cast<float>(step);
        EXPECT(kernels::ellipseDistance(Kernel2(1.8f * std::cos(angle), 0.9f * std::sin(angle)), radii) < 0.0f);
        EXPECT(kernels::ellipseDistance(Kernel2(2.2f * std::cos(angle), 1.1f * std::sin(angle)), radii) > 0.0f);
    }
}

TEST(triangleDistanceIsInsideByTheInradius) {
    // Radius 1: side 2, inradius 1/√3 = 0.57735 (negative inside), base at y = −0.57735
    EXPECT(nearly(kernels::triangleDistance(Kernel2(0.0f, 0.0f), 1.0f), -0.57735027f));
    // One below the origin's base: (0, −2) is 2 − 0.57735 = 1.42265 below the base
    EXPECT(nearly(kernels::triangleDistance(Kernel2(0.0f, -2.0f), 1.0f), 1.42264973f));
    // The apex is at y = 2/√3 = 1.1547; above it by 1
    EXPECT(nearly(kernels::triangleDistance(Kernel2(0.0f, 2.1547005f), 1.0f), 1.0f));
    // The corner (1, −0.57735) is on the outline
    EXPECT(nearly(kernels::triangleDistance(Kernel2(1.0f, -0.57735027f), 1.0f), 0.0f));
    // Symmetric about the y axis
    EXPECT(nearly(kernels::triangleDistance(Kernel2(0.4f, 0.3f), 1.0f), kernels::triangleDistance(Kernel2(-0.4f, 0.3f), 1.0f)));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::triangleDistance(point, 1.0f); }));
}

TEST(ringDistanceMeasuresFromTheCentreLineOfTheBand) {
    // radius 2, thickness 0.5
    EXPECT(nearly(kernels::ringDistance(Kernel2(2.0f, 0.0f), 2.0f, 0.5f), -0.25f));
    EXPECT(nearly(kernels::ringDistance(Kernel2(3.0f, 0.0f), 2.0f, 0.5f), 0.75f));
    EXPECT(nearly(kernels::ringDistance(Kernel2(0.0f, 0.0f), 2.0f, 0.5f), 1.75f));
    EXPECT(nearly(kernels::ringDistance(Kernel2(0.0f, 2.25f), 2.0f, 0.5f), 0.0f));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::ringDistance(point, 1.0f, 0.3f); }));
}

TEST(arcDistanceDrawsAnArcOrAPieSliceOpeningUp) {
    // Arc, aperture 45°, radius 1, thickness 0.2
    EXPECT(nearly(kernels::arcDistance(Kernel2(0.0f, 1.0f), 45.0f, 1.0f, 0.2f, false), -0.1f));
    EXPECT(nearly(kernels::arcDistance(Kernel2(0.0f, 2.0f), 45.0f, 1.0f, 0.2f, false), 0.9f));
    // Outside the aperture the nearest part is the arc's end (sin 45°, cos 45°): (1, 0) is
    // √((1 − 0.7071)² + 0.7071²) = 0.765367 away, less 0.1
    EXPECT(nearly(kernels::arcDistance(Kernel2(1.0f, 0.0f), 45.0f, 1.0f, 0.2f, false), 0.665367f));
    // Symmetric about the y axis
    EXPECT(nearly(kernels::arcDistance(Kernel2(-1.0f, 0.0f), 45.0f, 1.0f, 0.2f, false), 0.665367f));
    // Pie slice: (0, 0.5) is 0.5 from the outer edge but 0.5 sin 45° = 0.35355 from a straight side
    EXPECT(nearly(kernels::arcDistance(Kernel2(0.0f, 0.5f), 45.0f, 1.0f, 0.2f, true), -0.35355339f));
    EXPECT(nearly(kernels::arcDistance(Kernel2(0.0f, 2.0f), 45.0f, 1.0f, 0.2f, true), 1.0f));
    // Outside the wedge, below: (0, −1) is 1 from the apex
    EXPECT(nearly(kernels::arcDistance(Kernel2(0.0f, -1.0f), 45.0f, 1.0f, 0.2f, true), 1.0f));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::arcDistance(point, 60.0f, 1.0f, 0.3f, false); }));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::arcDistance(point, 60.0f, 1.0f, 0.3f, true); }));
}

TEST(starDistanceWithFullPointinessIsTheRegularPolygon) {
    // Five points, radius 1: a tip is straight up at (0, 1)
    for (float pointiness : {0.0f, 0.5f, 1.0f}) {
        EXPECT(nearly(kernels::starDistance(Kernel2(0.0f, 1.0f), 1.0f, 5.0f, pointiness), 0.0f, 1e-5f));
    }
    // Pointiness 1: a regular pentagon: the centre is inside by the apothem cos 36° = 0.809017, and
    // the middle of the bottom edge (0, −0.809017) is on the outline.
    EXPECT(nearly(kernels::starDistance(Kernel2(0.0f, 0.0f), 1.0f, 5.0f, 1.0f), -0.809017f));
    EXPECT(nearly(kernels::starDistance(Kernel2(0.0f, -0.809017f), 1.0f, 5.0f, 1.0f), 0.0f, 1e-5f));
    // Pointiness 0.5: the centre is still inside; the pentagon's edge midpoint is now outside the star.
    EXPECT(kernels::starDistance(Kernel2(0.0f, 0.0f), 1.0f, 5.0f, 0.5f) < 0.0f);
    EXPECT(kernels::starDistance(Kernel2(0.0f, -0.8f), 1.0f, 5.0f, 0.5f) > 0.0f);
    // Five-fold symmetry: turning by 72° gives the same distance.
    float angle = 0.3f;
    float turned = angle + 6.2831853f / 5.0f;
    EXPECT(nearly(kernels::starDistance(Kernel2(0.7f * std::sin(angle), 0.7f * std::cos(angle)), 1.0f, 5.0f, 0.5f),
        kernels::starDistance(Kernel2(0.7f * std::sin(turned), 0.7f * std::cos(turned)), 1.0f, 5.0f, 0.5f), 1e-4f));
    // Fewer than three points become three.
    EXPECT(nearly(kernels::starDistance(Kernel2(0.1f, 0.2f), 1.0f, 1.0f, 0.5f), kernels::starDistance(Kernel2(0.1f, 0.2f), 1.0f, 3.0f, 0.5f)));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::starDistance(point, 1.0f, 5.0f, 0.5f); }, 1e-3f));
}

TEST(heartDistanceScalesWithItsSize) {
    // size 1, origin: p = (0, 0.5); not past the diagonal; √min(0.25, 0.125) × sign(−0.5) = −0.35355339
    EXPECT(nearly(kernels::heartDistance(Kernel2(0.0f, 0.0f), 1.0f), -0.35355339f));
    // Far away is positive, the bottom tip is on the outline.
    EXPECT(kernels::heartDistance(Kernel2(3.0f, 3.0f), 1.0f) > 0.0f);
    EXPECT(nearly(kernels::heartDistance(Kernel2(0.0f, -0.5f), 1.0f), 0.0f));
    // Symmetric about the y axis.
    EXPECT(nearly(kernels::heartDistance(Kernel2(0.3f, 0.2f), 1.0f), kernels::heartDistance(Kernel2(-0.3f, 0.2f), 1.0f)));
    // Property: distance(k point, k size) = k distance(point, size).
    for (int step = 0; step < 40; step += 1) {
        Kernel2 point(0.11f * static_cast<float>(step) - 2.0f, 0.07f * static_cast<float>(step) - 1.0f);
        EXPECT(nearly(kernels::heartDistance(point * 2.5f, 2.5f), 2.5f * kernels::heartDistance(point, 1.0f), 1e-4f));
    }
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::heartDistance(point, 1.0f); }, 1e-3f));
}

TEST(crossDistanceIsInsideAnArmAndMeasuresToTheInnerCorner) {
    // size 1, thickness 0.2: at the centre the nearest outline is the inner corner (0.1, 0.1)
    EXPECT(nearly(kernels::crossDistance(Kernel2(0.0f, 0.0f), 1.0f, 0.2f), -0.14142136f));
    // Along an arm: half the thickness from the edge
    EXPECT(nearly(kernels::crossDistance(Kernel2(0.5f, 0.0f), 1.0f, 0.2f), -0.1f));
    EXPECT(nearly(kernels::crossDistance(Kernel2(0.0f, 0.5f), 1.0f, 0.2f), -0.1f));
    // Past the end of an arm: (2, 0) → 1
    EXPECT(nearly(kernels::crossDistance(Kernel2(2.0f, 0.0f), 1.0f, 0.2f), 1.0f));
    // In the notch between the arms: (0.5, 0.5): nearest the inner corner (0.1, 0.1)? no — the
    // edge of the arm y = 0.1 is 0.4 away
    EXPECT(nearly(kernels::crossDistance(Kernel2(0.5f, 0.5f), 1.0f, 0.2f), 0.4f));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::crossDistance(point, 1.0f, 0.3f); }));
}

TEST(moonDistanceIsACrescentOpeningToTheRight) {
    // outer 1, inner 1, offset 1: leftmost point of the outer circle is on the outline, the middle
    // of the crescent's thickness there is 0.5 inside
    EXPECT(nearly(kernels::moonDistance(Kernel2(-1.0f, 0.0f), 1.0f, 1.0f, 1.0f), 0.0f));
    EXPECT(nearly(kernels::moonDistance(Kernel2(-0.5f, 0.0f), 1.0f, 1.0f, 1.0f), -0.5f));
    // The bite is outside the moon: (1, 0) is the middle of the cut-out disc, 1 from its edge
    EXPECT(nearly(kernels::moonDistance(Kernel2(1.0f, 0.0f), 1.0f, 1.0f, 1.0f), 1.0f));
    // Past the horn: (2, 0) → nearest the horn tip (0.5, ±0.866): distance √(1.5² + 0.866²) = √3
    EXPECT(nearly(kernels::moonDistance(Kernel2(2.0f, 0.0f), 1.0f, 1.0f, 1.0f), 1.7320508f));
    // Symmetric about the x axis.
    EXPECT(nearly(kernels::moonDistance(Kernel2(-0.3f, 0.6f), 1.0f, 1.0f, 0.8f), kernels::moonDistance(Kernel2(-0.3f, -0.6f), 1.0f, 1.0f, 0.8f)));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::moonDistance(point, 1.0f, 1.0f, 0.8f); }, 1e-3f));
}

TEST(vesicaDistanceIsTheLensBetweenTwoDiscs) {
    // radius 1, spread 0.6: half height b = 0.8
    EXPECT(nearly(kernels::vesicaDistance(Kernel2(0.0f, 0.0f), 1.0f, 0.6f), -0.4f));
    EXPECT(nearly(kernels::vesicaDistance(Kernel2(0.0f, 2.0f), 1.0f, 0.6f), 1.2f));
    EXPECT(nearly(kernels::vesicaDistance(Kernel2(0.0f, 0.8f), 1.0f, 0.6f), 0.0f));
    EXPECT(nearly(kernels::vesicaDistance(Kernel2(0.4f, 0.0f), 1.0f, 0.6f), 0.0f));
    // Symmetric about both axes.
    EXPECT(nearly(kernels::vesicaDistance(Kernel2(0.2f, 0.3f), 1.0f, 0.6f), kernels::vesicaDistance(Kernel2(-0.2f, -0.3f), 1.0f, 0.6f)));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::vesicaDistance(point, 1.0f, 0.6f); }));
}

TEST(eggDistanceJoinsABigCircleBelowToASmallOneAbove) {
    // bottom 1, top 0.5
    EXPECT(nearly(kernels::eggDistance(Kernel2(0.0f, 0.0f), 1.0f, 0.5f), -1.0f));
    EXPECT(nearly(kernels::eggDistance(Kernel2(0.0f, -2.0f), 1.0f, 0.5f), 1.0f));
    // Above the small circle centred at (0, √3 × 0.5 = 0.8660254), radius 0.5: (0, 5) → 5 − 0.866 − 0.5
    EXPECT(nearly(kernels::eggDistance(Kernel2(0.0f, 5.0f), 1.0f, 0.5f), 3.6339746f));
    // Symmetric about the y axis.
    EXPECT(nearly(kernels::eggDistance(Kernel2(0.4f, 0.3f), 1.0f, 0.5f), kernels::eggDistance(Kernel2(-0.4f, 0.3f), 1.0f, 0.5f)));
    EXPECT(isLipschitzOne([](Kernel2 point) { return kernels::eggDistance(point, 1.0f, 0.5f); }));
}

TEST(bezierDistanceMatchesTheDistanceToSampledPointsOnTheCurve) {
    // The parabola y = (1 − x²) / 2 through (−1, 0), (0.5 at x = 0) with control point (0, 1).
    Kernel2 a(-1.0f, 0.0f);
    Kernel2 b(0.0f, 1.0f);
    Kernel2 c(1.0f, 0.0f);
    // On the curve → 0; straight above by 1; below → the nearest end (±1, 0) at √2
    EXPECT(nearly(kernels::bezierDistance(Kernel2(0.0f, 0.5f), a, b, c), 0.0f, 1e-3f));
    EXPECT(nearly(kernels::bezierDistance(Kernel2(0.0f, 1.5f), a, b, c), 1.0f, 1e-3f));
    EXPECT(nearly(kernels::bezierDistance(Kernel2(0.0f, -1.0f), a, b, c), 1.4142135f, 1e-3f));
    // A straight "curve" (control point at the middle) is a segment.
    EXPECT(nearly(kernels::bezierDistance(Kernel2(1.0f, 1.0f), Kernel2(0.0f, 0.0f), Kernel2(1.0f, 0.0f), Kernel2(2.0f, 0.0f)), 1.0f));
    // Property: agrees with the brute-force sampled distance over a grid (both branches of the cubic).
    for (int i = -12; i <= 12; i += 1) {
        for (int j = -10; j <= 14; j += 1) {
            Kernel2 point(0.171f * static_cast<float>(i) + 0.013f, 0.153f * static_cast<float>(j) + 0.021f);
            EXPECT(nearly(kernels::bezierDistance(point, a, b, c), sampledBezierDistance(point, a, b, c), 2e-3f));
        }
    }
    // An asymmetric curve too.
    Kernel2 d(-0.5f, -0.2f);
    Kernel2 e(1.4f, 1.1f);
    Kernel2 f(0.8f, -0.9f);
    for (int i = -10; i <= 10; i += 1) {
        for (int j = -10; j <= 10; j += 1) {
            Kernel2 point(0.19f * static_cast<float>(i) + 0.017f, 0.17f * static_cast<float>(j) + 0.011f);
            EXPECT(nearly(kernels::bezierDistance(point, d, e, f), sampledBezierDistance(point, d, e, f), 2e-3f));
        }
    }
    EXPECT(isLipschitzOne([=](Kernel2 point) { return kernels::bezierDistance(point, d, e, f); }, 5e-3f));
}

TEST(polygonDistanceIsSignedForAnyWinding) {
    Kernel2 origin(0.0f, 0.0f);
    // Triangle (0,0), (2,0), (0,2); the nearest edge to (0.5, 0.5) is 0.5 away (inside → negative)
    EXPECT(nearly(kernels::polygonDistance(Kernel2(0.5f, 0.5f), origin, Kernel2(2.0f, 0.0f), Kernel2(0.0f, 2.0f), origin, origin, origin, 3.0f), -0.5f));
    // Outside, nearest vertex (2, 0): (3, 0) → 1
    EXPECT(nearly(kernels::polygonDistance(Kernel2(3.0f, 0.0f), origin, Kernel2(2.0f, 0.0f), Kernel2(0.0f, 2.0f), origin, origin, origin, 3.0f), 1.0f));
    // Outside beside the slanted edge x + y = 2: (2, 2) is 2/√2 = √2 away
    EXPECT(nearly(kernels::polygonDistance(Kernel2(2.0f, 2.0f), origin, Kernel2(2.0f, 0.0f), Kernel2(0.0f, 2.0f), origin, origin, origin, 3.0f), 1.4142135f));
    // The same triangle wound the other way gives the same distances.
    EXPECT(nearly(kernels::polygonDistance(Kernel2(0.5f, 0.5f), origin, Kernel2(0.0f, 2.0f), Kernel2(2.0f, 0.0f), origin, origin, origin, 3.0f), -0.5f));
    // Square (0,0), (2,0), (2,2), (0,2) with count 4; the centre is 1 from every side.
    EXPECT(nearly(kernels::polygonDistance(Kernel2(1.0f, 1.0f), origin, Kernel2(2.0f, 0.0f), Kernel2(2.0f, 2.0f), Kernel2(0.0f, 2.0f), origin, origin, 4.0f), -1.0f));
    // A count below 3 is 3 and a count is rounded.
    EXPECT(nearly(kernels::polygonDistance(Kernel2(0.5f, 0.5f), origin, Kernel2(2.0f, 0.0f), Kernel2(0.0f, 2.0f), origin, origin, origin, 1.0f), -0.5f));
    EXPECT(nearly(kernels::polygonDistance(Kernel2(1.0f, 1.0f), origin, Kernel2(2.0f, 0.0f), Kernel2(2.0f, 2.0f), Kernel2(0.0f, 2.0f), origin, origin, 3.6f), -1.0f));
    // A concave "L": (0,0), (2,0), (2,1), (1,1), (1,2), (0,2); the notch (1.5, 1.5) is outside, 0.5 from both walls (y = 1 and x = 1)
    Kernel2 l0(0.0f, 0.0f), l1(2.0f, 0.0f), l2(2.0f, 1.0f), l3(1.0f, 1.0f), l4(1.0f, 2.0f), l5(0.0f, 2.0f);
    EXPECT(nearly(kernels::polygonDistance(Kernel2(1.5f, 1.5f), l0, l1, l2, l3, l4, l5, 6.0f), 0.5f));
    EXPECT(kernels::polygonDistance(Kernel2(0.5f, 1.5f), l0, l1, l2, l3, l4, l5, 6.0f) < 0.0f);
    EXPECT(nearly(kernels::polygonDistance(Kernel2(0.5f, 0.5f), l0, l1, l2, l3, l4, l5, 6.0f), -0.5f));
    EXPECT(isLipschitzOne([=](Kernel2 point) { return kernels::polygonDistance(point, l0, l1, l2, l3, l4, l5, 6.0f); }));
}
