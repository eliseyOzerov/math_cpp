// Ports of EditorTests/Math/Curves/CubicBezierFittingTests and CubicBezierOffsetTests, and of
// EditorTests/Math/Paths/Path2DOffsetTests and Path2DCornersTests (the corner, fillet and chamfer
// tests; the self-crossing tests are in PathTests.cpp), plus bit-for-bit checks against values
// printed by the Swift engine (the Engine/Math files compiled with swiftc and a small main that
// prints `bitPattern`). The tests that draw random curves with Swift's own generator are replaced
// by ones that use the core's `SeededRandom` (same checks, other numbers); the Double-precision
// `ExactCubic` of the Swift tests is a small helper here (sampled, then refined).
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "../Curves/CubicBezier.h"
#include "../Numerics/SeededRandom.h"
#include "../Curves/Path2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Curve2 = CubicBezier<Vector2>;

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
// Swift's `Float.pi`: rounded toward zero, one step below the nearest float.
constexpr float swiftPi = 0x1.921fb4p1f;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

bool sameBits(float value, std::uint32_t bits) { return std::bit_cast<std::uint32_t>(value) == bits; }

/// Swift's `Float.ulp` of a finite number.
float ulpOf(float value) {
    float magnitude = std::fabs(value);
    return std::nextafter(magnitude, infinity) - magnitude;
}

// MARK: - A Double cubic, for measuring results

struct ExactVector {
    double x = 0;
    double y = 0;
};

ExactVector exact(Vector2 point) { return ExactVector{point.x, point.y}; }

/// A cubic in Double: its point, derivatives, and the nearest point to a sample (sampled at 400
/// parameters, then narrowed by ternary search).
struct ExactCubic {
    ExactVector p[4];

    explicit ExactCubic(const Curve2& curve)
        : p{exact(curve.start), exact(curve.startHandle), exact(curve.endHandle), exact(curve.end)} {}

    ExactVector point(double t) const {
        double u = 1 - t;
        double w[4] = {u * u * u, 3 * u * u * t, 3 * u * t * t, t * t * t};
        ExactVector result;
        for (int index = 0; index < 4; index += 1) {
            result.x += p[index].x * w[index];
            result.y += p[index].y * w[index];
        }
        return result;
    }

    ExactVector derivative(double t) const {
        double u = 1 - t;
        double w[3] = {3 * u * u, 6 * u * t, 3 * t * t};
        ExactVector result;
        for (int index = 0; index < 3; index += 1) {
            result.x += (p[index + 1].x - p[index].x) * w[index];
            result.y += (p[index + 1].y - p[index].y) * w[index];
        }
        return result;
    }

    ExactVector secondDerivative(double t) const {
        double u = 1 - t;
        ExactVector result;
        result.x = (p[2].x - 2 * p[1].x + p[0].x) * 6 * u + (p[3].x - 2 * p[2].x + p[1].x) * 6 * t;
        result.y = (p[2].y - 2 * p[1].y + p[0].y) * 6 * u + (p[3].y - 2 * p[2].y + p[1].y) * 6 * t;
        return result;
    }

    double distanceAt(double t, ExactVector target) const {
        ExactVector at = point(t);
        return std::hypot(at.x - target.x, at.y - target.y);
    }

    /// The parameter of the nearest point and its distance.
    std::pair<double, double> nearest(ExactVector target) const {
        int steps = 400;
        double bestParameter = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (int step = 0; step <= steps; step += 1) {
            double t = static_cast<double>(step) / steps;
            double distance = distanceAt(t, target);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestParameter = t;
            }
        }
        double low = std::max(0.0, bestParameter - 1.0 / steps);
        double high = std::min(1.0, bestParameter + 1.0 / steps);
        for (int round = 0; round < 80; round += 1) {
            double first = low + (high - low) / 3;
            double second = high - (high - low) / 3;
            if (distanceAt(first, target) < distanceAt(second, target)) high = second; else low = first;
        }
        double parameter = (low + high) / 2;
        double distance = distanceAt(parameter, target);
        if (distance > bestDistance) return {bestParameter, bestDistance};
        return {parameter, distance};
    }
};

/// The largest true distance from any of `points` to the pieces.
double largestDistance(const std::vector<Vector2>& points, const std::vector<Curve2>& pieces) {
    std::vector<ExactCubic> exactPieces;
    for (const Curve2& piece : pieces) exactPieces.push_back(ExactCubic(piece));
    double largest = 0;
    for (Vector2 point : points) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const ExactCubic& piece : exactPieces) nearest = std::min(nearest, piece.nearest(exact(point)).second);
        largest = std::max(largest, nearest);
    }
    return largest;
}

/// Points on the circle arc of `radius` around `centre`, `count` of them from angle 0 to `sweep`.
std::vector<Vector2> arcPoints(int count, float radius = 1, Vector2 centre = Vector2(), float sweep = swiftPi) {
    std::vector<Vector2> points;
    for (int index = 0; index < count; index += 1) {
        Angle angle = Angle::fromRadians(sweep * static_cast<float>(index) / static_cast<float>(count - 1));
        points.push_back(centre + Vector2(angle.cosine(), angle.sine()) * radius);
    }
    return points;
}

std::vector<std::array<std::uint32_t, 8>> noBits;

/// Whether the pieces' control points have exactly these bit patterns.
bool matchesBits(const std::vector<Curve2>& pieces, const std::vector<std::array<std::uint32_t, 8>>& expected) {
    if (pieces.size() != expected.size()) return false;
    for (size_t index = 0; index < pieces.size(); index += 1) {
        const Curve2& piece = pieces[index];
        const std::array<std::uint32_t, 8>& bits = expected[index];
        if (!(sameBits(piece.start.x, bits[0]) && sameBits(piece.start.y, bits[1])
              && sameBits(piece.startHandle.x, bits[2]) && sameBits(piece.startHandle.y, bits[3])
              && sameBits(piece.endHandle.x, bits[4]) && sameBits(piece.endHandle.y, bits[5])
              && sameBits(piece.end.x, bits[6]) && sameBits(piece.end.y, bits[7]))) {
            return false;
        }
    }
    return true;
}

struct ExpectedSegment {
    bool isLine;
    std::array<std::uint32_t, 6> words;
};

struct ExpectedPath {
    std::array<std::uint32_t, 2> start;
    bool closed;
    std::vector<ExpectedSegment> segments;
};

/// Whether the path has exactly this start, closed flag and segments (bit patterns).
bool matchesBits(const std::optional<Path2D>& path, const ExpectedPath& expected) {
    if (!path) return false;
    if (!(sameBits(path->start.x, expected.start[0]) && sameBits(path->start.y, expected.start[1]))) return false;
    if (path->isClosed() != expected.closed || path->segments().size() != expected.segments.size()) return false;
    for (size_t index = 0; index < expected.segments.size(); index += 1) {
        const Path2D::Segment& segment = path->segments()[index];
        const ExpectedSegment& want = expected.segments[index];
        if (segment.isLine() != want.isLine) return false;
        if (want.isLine) {
            if (!(sameBits(segment.endPoint.x, want.words[0]) && sameBits(segment.endPoint.y, want.words[1]))) return false;
        } else if (!(sameBits(segment.startHandle.x, want.words[0]) && sameBits(segment.startHandle.y, want.words[1])
                     && sameBits(segment.endHandle.x, want.words[2]) && sameBits(segment.endHandle.y, want.words[3])
                     && sameBits(segment.endPoint.x, want.words[4]) && sameBits(segment.endPoint.y, want.words[5]))) {
            return false;
        }
    }
    return true;
}

// The values the Swift engine prints for the same inputs.
const std::vector<std::array<std::uint32_t, 8>> expected_fitKnown = {
    {0x0, 0x0, 0x3f7d5b4e, 0x403f4567, 0x4040a929, 0x403f4561, 0x40800000, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitSemi = {
    {0x3f800000, 0x0, 0x3f7ffe2b, 0x3e5cafaa, 0x3f6dcd6e, 0x3edb5d22, 0x3f4ca848, 0x3f19ca3e},
    {0x3f4ca848, 0x3f19ca3e, 0x3f33d993, 0x3f3acd7e, 0x3f130d8f, 0x3f5594a3, 0x3edb7d97, 0x3f674887},
    {0x3edb7d97, 0x3f674887, 0xbe6f2397, 0x3f9bdcf8, 0xbf7ff9c4, 0x3f3ba17a, 0xbf800000, 0x34222169},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitArc55 = {
    {0x3f800000, 0x0, 0x3f7ff922, 0x3f0564bd, 0x3f19438d, 0x3f74a1ae, 0x3da22d47, 0x3f7f3233},
    {0x3da22d47, 0x3f7f3233, 0xbe4f6d0b, 0x3f827681, 0xbef7b336, 0x3f6b70ef, 0xbf304f7b, 0x3f399bd9},
    {0xbf304f7b, 0x3f399bd9, 0xbf58d19e, 0x3f132138, 0xbf73f7f4, 0x3ebf2c9a, 0xbf7cca15, 0x3e21aae6},
    {0xbf7cca15, 0x3e21aae6, 0xbf8189b4, 0x3b8c908b, 0xbf800550, 0xbe1e336d, 0xbf73fcbb, 0xbe9affef},
    {0xbf73fcbb, 0xbe9affef, 0xbf6a5976, 0xbed7af02, 0xbf5b0442, 0xbf085020, 0xbf46fcd1, 0xbf210f0f},
    {0xbf46fcd1, 0xbf210f0f, 0xbecea460, 0xbf8ba092, 0x3e9441a2, 0xbf902bcc, 0x3f356b62, 0xbf349e4a},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitLoop = {
    {0x40400000, 0xbf800000, 0x40400000, 0xbf0fe9d2, 0x40369b94, 0xbe04f435, 0x40258c9a, 0x3e5ebe48},
    {0x40258c9a, 0x3e5ebe48, 0x3fdc3d11, 0x3fac5181, 0xbca98521, 0x3f9db5a9, 0xbf3b67aa, 0x35000000},
    {0xbf3b67aa, 0x35000000, 0xbf68297c, 0xbe9b0af7, 0xbf800000, 0xbf267c61, 0xbf800000, 0xbf800001},
    {0xbf800000, 0xbf800001, 0xbf800000, 0xbfb80b17, 0xbf5a6e50, 0xbfef6179, 0xbf16326a, 0xc00debe4},
    {0xbf16326a, 0xc00debe4, 0x3e8f0bb1, 0xc05628c1, 0x4001530b, 0xc04edad4, 0x402ed9eb, 0xc0000002},
    {0x402ed9eb, 0xc0000002, 0x403a0a5f, 0xbfd93d44, 0x40400000, 0xbfacc1ce, 0x40400000, 0xbf800000},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitU0_01 = {
    {0x0, 0x0, 0xb850f257, 0x3fe196f3, 0x3c0c59b0, 0x40a74140, 0x3e99de89, 0x40de2615},
    {0x3e99de89, 0x40de2615, 0x3f096a2b, 0x410548bc, 0x3f3aeab8, 0x40d8c6a6, 0x3f446aa3, 0x40ce7a15},
    {0x3f446aa3, 0x40ce7a15, 0x3f7dde85, 0x40903083, 0x3f8001d0, 0x3ffb45c6, 0x3f800000, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitU0_001 = {
    {0x0, 0x0, 0xb8073ff6, 0x3f9205d4, 0x3c42bfde, 0x40120917, 0x3d4155bf, 0x405b03af},
    {0x3d4155bf, 0x405b03af, 0x3d932e1f, 0x4086fe08, 0x3dd8b3a2, 0x40a07db1, 0x3e2ec945, 0x40b9e856},
    {0x3e2ec945, 0x40b9e856, 0x3e4e5bf7, 0x40c5fc95, 0x3e72a214, 0x40d231d2, 0x3e99de89, 0x40de2615},
    {0x3e99de89, 0x40de2615, 0x3ea2594e, 0x40e1433e, 0x3ec40a9d, 0x40ee0da0, 0x3ef45e01, 0x40efc795},
    {0x3ef45e01, 0x40efc795, 0x3f2308e6, 0x40f2b2c8, 0x3f3e0a78, 0x40d56399, 0x3f446aa3, 0x40ce7a15},
    {0x3f446aa3, 0x40ce7a15, 0x3f57acc1, 0x40b99917, 0x3f61d451, 0x40a3cda5, 0x3f69b05f, 0x408ecf57},
    {0x3f69b05f, 0x408ecf57, 0x3f7b67ce, 0x403ef9ee, 0x3f80015e, 0x3fbd7ac7, 0x3f800000, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitDirs = {
    {0x0, 0x0, 0x0, 0x3f800000, 0x40400000, 0xbf800000, 0x40400000, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitTriangleClosed = {
    {0x0, 0x0, 0x3e715bef, 0xbe715bef, 0x3f43a904, 0xbe715bef, 0x3f800000, 0x0},
    {0x3f800000, 0x0, 0x40029692, 0x3f852d24, 0xbfce034d, 0x3fce034d, 0x0, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitThereBack = {
    {0x0, 0x0, 0x3eaaaaab, 0x0, 0x3f2aaaab, 0x0, 0x3f800000, 0x0},
    {0x3f800000, 0x0, 0x3faaaaab, 0x0, 0x3eaaaaab, 0x0, 0x0, 0x0},
};
const std::vector<std::array<std::uint32_t, 8>> expected_fitFar = {
    {0x447a4000, 0x447a0000, 0x447a3ffc, 0x447a1598, 0x447a3517, 0x447a29c4, 0x447a2301, 0x447a3594},
    {0x447a2301, 0x447a3594, 0x447a10ee, 0x447a4162, 0x4479fa10, 0x447a4348, 0x4479e64b, 0x447a3a9c},
    {0x4479e64b, 0x447a3a9c, 0x4479dc96, 0x447a365a, 0x4479d402, 0x447a2fad, 0x4479cd7f, 0x447a274f},
    {0x4479cd7f, 0x447a274f, 0x4479c4c6, 0x447a1c19, 0x4479c003, 0x447a0e33, 0x4479c000, 0x447a0000},
};
const std::vector<std::array<std::uint32_t, 8>> expected_offsetKnown0_2 = {
    {0x3e424a54, 0xbd8186e3, 0x3edde832, 0x3f2af3af, 0x3f3c87fb, 0x3f9a678b, 0x3f86df04, 0x3fc6b392},
    {0x3f86df04, 0x3fc6b392, 0x3f9b2c87, 0x3fdcd996, 0x3fafd9f6, 0x3fecd6d6, 0x3fc4246d, 0x3ff73354},
    {0x3fc4246d, 0x3ff73354, 0x3fd86ee4, 0x4000c7e9, 0x3fec4444, 0x40033333, 0x40000000, 0x40033333},
    {0x40000000, 0x40033333, 0x4009ddde, 0x40033333, 0x4013c88e, 0x4000c7e9, 0x401dedc9, 0x3ff73354},
    {0x401dedc9, 0x3ff73354, 0x40281304, 0x3fecd6d6, 0x403269bc, 0x3fdcd996, 0x403c907e, 0x3fc6b392},
    {0x403c907e, 0x3fc6b392, 0x4050de01, 0x3f9a678b, 0x406442fa, 0x3f2af3af, 0x4073db5b, 0xbd8186e3},
};
const std::vector<std::array<std::uint32_t, 8>> expected_offsetKnown_0_2 = {
    {0xbe424a54, 0x3d8186e3, 0x3d885f3c, 0x3f550c52, 0x3ec6f00b, 0x3fb59875, 0x3f4241f9, 0x3fe94c6e},
    {0x3f4241f9, 0x3fe94c6e, 0x3f71a6f3, 0x40019335, 0x3f92260a, 0x400b9495, 0x3facdb93, 0x40126656},
    {0x3facdb93, 0x40126656, 0x3fc7911c, 0x40193817, 0x3fe3bbbc, 0x401ccccd, 0x40000000, 0x401ccccd},
    {0x40000000, 0x401ccccd, 0x400e2222, 0x401ccccd, 0x401c3772, 0x40193817, 0x40299237, 0x40126656},
    {0x40299237, 0x40126656, 0x4036ecfc, 0x400b9495, 0x40439644, 0x40019335, 0x404f6f82, 0x3fe94c6e},
    {0x404f6f82, 0x3fe94c6e, 0x406721ff, 0x3fb59875, 0x407bbd07, 0x3f550c52, 0x40861253, 0x3d8186e3},
};
const std::vector<std::array<std::uint32_t, 8>> expected_offsetVar = {
    {0x3dc24a54, 0xbd0186e3, 0x3e6fb78a, 0x3eaaae9a, 0x3ec68396, 0x3f260a07, 0x3f0cc3cb, 0x3f69b5f2},
    {0x3f0cc3cb, 0x3f69b5f2, 0x3f3645cb, 0x3f96b0ee, 0x3f61fdd6, 0x3fb1fe98, 0x3f86df04, 0x3fc6b392},
    {0x3f86df04, 0x3fc6b392, 0x3f9cbf1d, 0x3fdb688c, 0x3fb29816, 0x3fe972f0, 0x3fc70d89, 0x3ff1802a},
    {0x3fc70d89, 0x3ff1802a, 0x3fd14843, 0x3ff586c7, 0x3fdb2991, 0x3ff81440, 0x3fe4a44d, 0x3ff95c8d},
    {0x3fe4a44d, 0x3ff95c8d, 0x3fee1f09, 0x3ffaa4da, 0x3ff73333, 0x3ffaaaab, 0x40000000, 0x3ff9999a},
    {0x40000000, 0x3ff9999a, 0x40046666, 0x3ff88889, 0x4008aaa2, 0x3ff661bb, 0x400ce630, 0x3ff3278f},
    {0x400ce630, 0x3ff3278f, 0x401121be, 0x3fefed63, 0x401555a4, 0x3feb9df4, 0x40199020, 0x3fe619d4},
    {0x40199020, 0x3fe619d4, 0x40220519, 0x3fdb1194, 0x402a9cc4, 0x3fcb2434, 0x403320fc, 0x3fb56724},
    {0x403320fc, 0x3fb56724, 0x403ba534, 0x3f9faa14, 0x40440c14, 0x3f842340, 0x404bed28, 0x3f4521d6},
    {0x404bed28, 0x3f4521d6, 0x4053ce3c, 0x3f01fd2c, 0x405b28f8, 0x3e4b33ba, 0x4061a463, 0xbe21e89c},
};
const std::vector<std::array<std::uint32_t, 8>> expected_offsetS = {
    {0x3e7f9b1a, 0xbe2a6766, 0x3ebc86a4, 0x3c3c3de0, 0x3ef44804, 0x3e1bf4e3, 0x3f131e64, 0x3e8504b8},
    {0x3f131e64, 0x3e8504b8, 0x3f2c18c6, 0x3ebc0efe, 0x3f4229ab, 0x3ee1db4d, 0x3f544e00, 0x3ef9cdd2},
    {0x3f544e00, 0x3ef9cdd2, 0x3f667255, 0x3f08e02b, 0x3f744bfb, 0x3f0dec83, 0x3f7ebfe9, 0x3f0fdc9c},
    {0x3f7ebfe9, 0x3f0fdc9c, 0x3f8499ec, 0x3f11ccb5, 0x3f885d64, 0x3f113316, 0x3f8cf741, 0x3f0e705e},
    {0x3f8cf741, 0x3f0e705e, 0x3f91911e, 0x3f0bada6, 0x3f970d1f, 0x3f061e28, 0x3f9d626e, 0x3ef8f9c2},
    {0x3f9d626e, 0x3ef8f9c2, 0x3fa3b7bd, 0x3ee5b735, 0x3faac2fe, 0x3eca872f, 0x3fb20e1d, 0x3ea91064},
    {0x3fb20e1d, 0x3ea91064, 0x3fc0a45b, 0x3e4c459c, 0x3fd00c9d, 0x3cacc4d0, 0x3fe00c9d, 0xbe2a6766},
    {0x3fe00c9d, 0xbe2a6766, 0x3ff00c9d, 0xbeb533b3, 0x4000522d, 0xbf0cee99, 0x400a070e, 0xbf3977ce},
    {0x400a070e, 0xbf3977ce, 0x400ee17e, 0xbf4fbc68, 0x40141bde, 0xbf64a466, 0x401a1137, 0xbf76c31f},
    {0x401a1137, 0xbf76c31f, 0x40200690, 0xbf8470ec, 0x4026c88f, 0xbf8c292d, 0x402e7ba1, 0xbf90c7d1},
    {0x402e7ba1, 0xbf90c7d1, 0x40362eb3, 0xbf956675, 0x403eccf6, 0xbf9699a5, 0x40474ffa, 0xbf9371b2},
    {0x40474ffa, 0xbf9371b2, 0x404fd2fe, 0xbf9049bf, 0x40581c95, 0xbf890fea, 0x40601380, 0xbf7d1917},
    {0x40601380, 0xbf7d1917, 0x40680a6b, 0xbf681259, 0x406fc631, 0xbf4c7881, 0x4077a799, 0xbf29bda4},
    {0x4077a799, 0xbf29bda4, 0x407f8901, 0xbf0702c7, 0x4083c86a, 0xbeba1e11, 0x4087fcd9, 0xbe2a6766},
};
const ExpectedPath expected_sqOutMiter = {{0x0, 0xbe800000}, true, {
    {true, {0x3f800000, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f800000, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_sqOutRound = {{0x0, 0xbe800000}, true, {
    {true, {0x3f800000, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3f91ac51, 0xbe800000, 0x3fa00000, 0xbe0d6289, 0x3fa00000, 0x0}},
    {true, {0x3fa00000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3fa00000, 0x3f91ac51, 0x3f91ac51, 0x3fa00000, 0x3f800000, 0x3fa00000}},
    {true, {0x0, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0xbe0d628a, 0x3fa00000, 0xbe800000, 0x3f91ac51, 0xbe800000, 0x3f800000}},
    {true, {0xbe800000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {false, {0xbe800001, 0xbe0d6286, 0xbe0d6289, 0xbe800000, 0x0, 0xbe800000}},
}};
const ExpectedPath expected_sqOutBevel = {{0x0, 0xbe800000}, true, {
    {true, {0x3f800000, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3fa00000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f800000, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3fa00000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0xbe800000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0xbe800000, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_sqIn = {{0x0, 0x3dcccccd}, true, {
    {true, {0x3f800000, 0x3dcccccd, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f666666, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f666666, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f800000, 0x3f666666, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3f666666, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3dcccccd, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3dcccccd, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3dcccccd, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_openMix = {{0x3dc24a54, 0xbd0186e3}, false, {
    {false, {0x3eb7f301, 0x3f33fa06, 0x3f2f2438, 0x3f9f06f8, 0x3f82ad93, 0x3fca8b8e}},
    {false, {0x3f983b4e, 0x3fe04dd9, 0x3fae1348, 0x3fefa84e, 0x3fc32c2c, 0x3ff91980}},
    {false, {0x3fd84510, 0x40014559, 0x3fec8101, 0x400314b6, 0x40000000, 0x40027cfe}},
    {false, {0x4009bf80, 0x4001e546, 0x40133b98, 0x3ffddd32, 0x401ccbee, 0x3ff2c40f}},
    {false, {0x40265c44, 0x3fe7aaec, 0x402ffca8, 0x3fd7441e, 0x40396b25, 0x3fc0ef19}},
    {false, {0x404c481e, 0x3f94450e, 0x405e3623, 0x3f1f9619, 0x406c6f51, 0xbdd0b1f6}},
    {false, {0x406e2a1d, 0xbe3b5f24, 0x4071e278, 0xbe800033, 0x4076c6e2, 0xbe9391db}},
    {true, {0x40ba4692, 0xbfadcb70, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_sqFillet = {{0x3e4ccccd, 0x0}, true, {
    {true, {0x3f4ccccd, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3f6913b5, 0xb1a5e588, 0x3f800000, 0x3db76257, 0x3f800000, 0x3e4ccccd}},
    {true, {0x3f800000, 0x3f4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3f800000, 0x3f6913b5, 0x3f6913b5, 0x3f800000, 0x3f4ccccd, 0x3f800000}},
    {true, {0x3e4ccccd, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3db76257, 0x3f800000, 0x31b434f0, 0x3f6913b5, 0x0, 0x3f4ccccd}},
    {true, {0x0, 0x3e4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {false, {0xb28f447e, 0x3db7625b, 0x3db76259, 0xb28b5086, 0x3e4ccccd, 0x0}},
}};
const ExpectedPath expected_sqChamfer = {{0x3e4ccccd, 0x0}, true, {
    {true, {0x3f4ccccd, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f800000, 0x3e4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f800000, 0x3f4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3f4ccccd, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3e4ccccd, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3f4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3e4ccccd, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3e4ccccd, 0x0, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_sqFilletClamp = {{0x3f000000, 0x0}, true, {
    {true, {0x3f000000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3f46b144, 0xb24f5eea, 0x3f800000, 0x3e653aee, 0x3f800000, 0x3f000000}},
    {true, {0x3f800000, 0x3f000000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3f800000, 0x3f46b144, 0x3f46b144, 0x3f800000, 0x3f000000, 0x3f800000}},
    {true, {0x3f000000, 0x3f800000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3e653aec, 0x3f800000, 0xb2cf5eea, 0x3f46b144, 0x0, 0x3f000000}},
    {true, {0x0, 0x3f000000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0xb333159d, 0x3e653af2, 0x3e653aee, 0xb1624a74, 0x3f000000, 0x0}},
}};
const ExpectedPath expected_openMixFillet = {{0x0, 0x0}, false, {
    {false, {0x3f7a4f84, 0x403bbba3, 0x403a579c, 0x403fe7b9, 0x407ba3b8, 0x3e484420}},
    {false, {0x407e8aec, 0x3d914fbe, 0x408221c9, 0xbd087251, 0x4085ed80, 0xbdbdb00f}},
    {true, {0x40c00000, 0xbf800000, 0x0, 0x0, 0x0, 0x0}},
}};
const ExpectedPath expected_curveFillet = {{0x3e99999a, 0x0}, true, {
    {true, {0x40000000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x403e18e2, 0x0, 0x403ff184, 0x3f7100fe, 0x400589e7, 0x3ff49628}},
    {false, {0x40020c72, 0x3ffc005d, 0x3ffa4714, 0x40000000, 0x3ff0181d, 0x40000000}},
    {true, {0x3e99999a, 0x40000000, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x3e0989c3, 0x40000000, 0x328393da, 0x3feecec8, 0x0, 0x3fd9999a}},
    {true, {0x0, 0x3e99999a, 0x0, 0x0, 0x0, 0x0}},
    {false, {0xb2d6e6bd, 0x3e0989c6, 0x3e0989c2, 0xb107c646, 0x3e99999a, 0x0}},
}};
const ExpectedPath expected_curveChamfer = {{0x3e99999a, 0x0}, true, {
    {true, {0x40000000, 0x0, 0x0, 0x0, 0x0, 0x0}},
    {false, {0x403b4d58, 0x0, 0x403fa7b8, 0x3f5bcbdd, 0x400d0f21, 0x3fe3dd04}},
    {true, {0x3fd9999a, 0x40000000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3e99999a, 0x40000000, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3fd9999a, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x0, 0x3e99999a, 0x0, 0x0, 0x0, 0x0}},
    {true, {0x3e99999a, 0x0, 0x0, 0x0, 0x0, 0x0}},
}};

const Curve2 knownCubic(Vector2(0, 0), Vector2(1, 3), Vector2(3, 3), Vector2(4, 0));

/// `count` points on `curve`, evaluated in Double and rounded once to Float (as Swift's test does).
std::vector<Vector2> samples(const Curve2& c, int count) {
    std::vector<Vector2> points;
    for (int i = 0; i < count; i += 1) {
        double t = static_cast<double>(i) / static_cast<double>(count - 1);
        double u = 1 - t;
        auto component = [&](float a, float b, float cc, float d) {
            return static_cast<float>(
                static_cast<double>(a) * u * u * u + static_cast<double>(b) * 3 * u * u * t
                + static_cast<double>(cc) * 3 * u * t * t + static_cast<double>(d) * t * t * t);
        };
        points.push_back(Vector2(
            component(c.start.x, c.startHandle.x, c.endHandle.x, c.end.x),
            component(c.start.y, c.startHandle.y, c.endHandle.y, c.end.y)));
    }
    return points;
}

bool allFinite(const std::vector<Curve2>& pieces) {
    for (const Curve2& piece : pieces) {
        if (!piece.startHandle.isFinite() || !piece.endHandle.isFinite()) return false;
    }
    return true;
}

} // namespace

// MARK: - Fitting (CubicBezierFittingTests)

TEST(fitPointsFromOneCubicGiveOnePiece) {
    std::vector<Vector2> points;
    for (int i = 0; i < 30; i += 1) points.push_back(knownCubic.point(static_cast<float>(i) / 29));
    std::vector<Curve2> pieces = Curve2::fitted(points, 0.01f);
    EXPECT(pieces.size() == 1);
    EXPECT(pieces[0].start == points[0]);
    EXPECT(pieces[0].end == points[29]);
    EXPECT(largestDistance(points, pieces) <= 0.01);
    EXPECT(isClose(pieces[0].startHandle, knownCubic.startHandle, 0.04f));
    EXPECT(isClose(pieces[0].endHandle, knownCubic.endHandle, 0.04f));
    EXPECT(matchesBits(pieces, expected_fitKnown));
}

TEST(fitPointsOnALineGiveOneStraightPiece) {
    std::vector<Vector2> points;
    for (float x : {0.0f, 0.3f, 0.4f, 1.5f, 1.6f, 3.0f, 4.0f}) points.push_back(Vector2(x, x / 2));
    std::vector<Curve2> pieces = Curve2::fitted(points, 1e-3f);
    EXPECT(pieces.size() == 1);
    Vector2 direction = Vector2(2, 1).normalized();
    EXPECT(std::fabs((pieces[0].startHandle - points[0]).cross(direction)) < 1e-5f);
    EXPECT(std::fabs((pieces[0].endHandle - points[6]).cross(direction)) < 1e-5f);
    EXPECT(pieces[0].startHandle.x > 0 && pieces[0].endHandle.x < 4);
}

TEST(fitTwoPointsGiveAStraightPieceWithHandlesAtThirds) {
    std::vector<Curve2> pieces = Curve2::fitted({Vector2(1, 1), Vector2(4, 7)}, 0.1f);
    EXPECT(pieces.size() == 1);
    EXPECT(isClose(pieces[0].startHandle, Vector2(2, 3)));
    EXPECT(isClose(pieces[0].endHandle, Vector2(3, 5)));
}

TEST(fitGivenDirectionsBendTheTwoPointCurve) {
    std::vector<Curve2> pieces = Curve2::fitted({Vector2(0, 0), Vector2(3, 0)}, 0.1f, Vector2(0, 5), Vector2(0, 1));
    EXPECT(pieces.size() == 1);
    EXPECT(isClose(pieces[0].startHandle, Vector2(0, 1)));
    EXPECT(isClose(pieces[0].endHandle, Vector2(3, -1)));
    EXPECT(matchesBits(pieces, expected_fitDirs));
}

TEST(fitNoPiecesForTooLittleOrBadInput) {
    EXPECT(Curve2::fitted({}, 0.1f).empty());
    EXPECT(Curve2::fitted({Vector2(1, 1)}, 0.1f).empty());
    EXPECT(Curve2::fitted({Vector2(1, 1), Vector2(1, 1), Vector2(1, 1)}, 0.1f).empty());
    std::vector<Vector2> line = {Vector2(0, 0), Vector2(1, 0), Vector2(2, 0)};
    EXPECT(Curve2::fitted(line, 0).empty());
    EXPECT(Curve2::fitted(line, -1).empty());
    EXPECT(Curve2::fitted(line, notANumber).empty());
    EXPECT(Curve2::fitted(line, infinity).empty());
    EXPECT(Curve2::fitted({Vector2(0, 0), Vector2(notANumber, 1), Vector2(2, 0)}, 0.1f).empty());
    EXPECT(Curve2::fitted({Vector2(0, 0), Vector2(1, infinity), Vector2(2, 0)}, 0.1f).empty());
}

TEST(fitDuplicateConsecutivePointsAreDropped) {
    std::vector<Vector2> clean = arcPoints(12);
    std::vector<Vector2> doubled;
    for (Vector2 point : clean) {
        doubled.push_back(point);
        doubled.push_back(point);
    }
    std::vector<Curve2> withDuplicates = Curve2::fitted(doubled, 0.01f);
    std::vector<Curve2> withoutDuplicates = Curve2::fitted(clean, 0.01f);
    EXPECT(withDuplicates == withoutDuplicates);
    EXPECT(!withDuplicates.empty());
    std::vector<Vector2> nearlyRepeated = clean;
    nearlyRepeated.push_back(clean[11] + Vector2(1e-9f, 0));
    std::vector<Curve2> pieces = Curve2::fitted(nearlyRepeated, 0.01f);
    EXPECT(pieces.back().end == nearlyRepeated[12]);
}

TEST(fitASemicircleStaysWithinTheToleranceInFewPieces) {
    std::vector<Vector2> points = arcPoints(40);
    for (auto [tolerance, mostPieces] : std::vector<std::pair<float, size_t>>{{0.01f, 2}, {0.001f, 3}, {0.0001f, 5}}) {
        std::vector<Curve2> pieces = Curve2::fitted(points, tolerance);
        EXPECT(!pieces.empty());
        EXPECT(pieces.size() <= mostPieces);
        EXPECT(largestDistance(points, pieces) <= static_cast<double>(tolerance));
    }
    EXPECT(matchesBits(Curve2::fitted(points, 0.001f), expected_fitSemi));
}

TEST(fitPiecesJoinExactlyAndSmoothly) {
    std::vector<Vector2> points = arcPoints(60, 1, Vector2(), 5.5f);
    std::vector<Curve2> pieces = Curve2::fitted(points, 0.0005f);
    EXPECT(pieces.size() >= 2);
    EXPECT(pieces.front().start == points.front());
    EXPECT(pieces.back().end == points.back());
    for (size_t index = 1; index < pieces.size(); index += 1) {
        const Curve2& before = pieces[index - 1];
        const Curve2& after = pieces[index];
        EXPECT(before.end == after.start);
        Vector2 incoming = (before.end - before.endHandle).normalized();
        Vector2 outgoing = (after.startHandle - after.start).normalized();
        EXPECT(incoming.dot(outgoing) > 0.999999f);
    }
    for (const Curve2& piece : pieces) {
        EXPECT(std::find(points.begin(), points.end(), piece.end) != points.end());
    }
    EXPECT(matchesBits(pieces, expected_fitArc55));
}

TEST(fitAClosedLoopGivesAClosedSmoothCurve) {
    std::vector<Vector2> points = arcPoints(49, 2, Vector2(1, -1), 2 * swiftPi);
    points[48] = points[0];
    std::vector<Curve2> pieces = Curve2::fitted(points, 0.002f);
    EXPECT(pieces.size() >= 2 && pieces.size() <= 8);
    EXPECT(pieces.back().end == pieces.front().start);
    EXPECT(largestDistance(points, pieces) <= 0.002);
    Vector2 incoming = (pieces.back().end - pieces.back().endHandle).normalized();
    Vector2 outgoing = (pieces.front().startHandle - pieces.front().start).normalized();
    EXPECT(incoming.dot(outgoing) > 0.9999f);
    std::optional<Path2D> path = Path2D::fitted(points, 0.002f);
    EXPECT(path.has_value());
    EXPECT(path->isClosed());
    EXPECT(path->signedArea() > 0);
    EXPECT(std::fabs(path->signedArea() - 4 * swiftPi) < 0.1f);
    EXPECT(path->signedArea() == std::bit_cast<float>(std::uint32_t{0x4149124f}));
    EXPECT(path->segments().size() == 6);
    EXPECT(matchesBits(pieces, expected_fitLoop));
}

TEST(fitAWigglyStrokeKeepsEveryPointWithinTolerance) {
    SeededRandom generator(7);
    std::vector<Vector2> points;
    for (int index = 0; index < 120; index += 1) {
        float t = static_cast<float>(index) / 119;
        float wobbleX = generator.nextUnitFloat() - 0.5f;
        float wobbleY = generator.nextUnitFloat() - 0.5f;
        points.push_back(Vector2(t * 6, 1.2f * Angle::fromRadians(t * 2 * swiftPi).sine()) + Vector2(wobbleX, wobbleY) * 0.02f);
    }
    std::vector<Curve2> pieces = Curve2::fitted(points, 0.03f);
    EXPECT(pieces.size() >= 2 && pieces.size() <= 9);
    EXPECT(largestDistance(points, pieces) <= 0.03);
    std::vector<Curve2> tighter = Curve2::fitted(points, 0.005f);
    EXPECT(tighter.size() >= pieces.size());
    EXPECT(largestDistance(points, tighter) <= 0.005);
}

TEST(fitAToleranceBelowTheRoundingIsRaisedToTheFloor) {
    std::vector<Vector2> points = arcPoints(20, 1, Vector2(1000, 1000));
    std::vector<Curve2> pieces = Curve2::fitted(points, 1e-9f);
    EXPECT(!pieces.empty() && pieces.size() <= 5);
    EXPECT(pieces.front().start == points.front());
    EXPECT(pieces.back().end == points.back());
    float floor = Curve2::smallestFittingTolerance(points);
    EXPECT(largestDistance(points, pieces) <= static_cast<double>(floor));
    EXPECT(sameBits(floor, 0x39800000));
    EXPECT(matchesBits(pieces, expected_fitFar));
}

TEST(fitTheResultFollowsTheScaleAndPositionOfTheStroke) {
    std::vector<Vector2> base = arcPoints(40);
    std::vector<Curve2> reference = Curve2::fitted(base, 0.001f);
    for (auto [scale, offset] : std::vector<std::pair<float, Vector2>>{
             {1e-3f, Vector2()}, {1e3f, Vector2()}, {1, Vector2(1000, 1000)}, {1e3f, Vector2(1e6f, 0)}}) {
        std::vector<Vector2> points;
        for (Vector2 point : base) points.push_back(point * scale + offset);
        std::vector<Curve2> pieces = Curve2::fitted(points, 0.001f * scale);
        EXPECT(pieces.size() == reference.size());
        double slack = static_cast<double>(ulpOf(offset.x) + ulpOf(offset.y)) * 2;
        EXPECT(largestDistance(points, pieces) <= static_cast<double>(0.001f * scale) + slack);
    }
}

TEST(fitTooManyPointsGiveNothing) {
    std::vector<Vector2> points(Curve2::mostFittedPoints + 1, Vector2());
    EXPECT(Curve2::fitted(points, 0.1f).empty());
}

TEST(fitATinyStrokeStillFits) {
    std::vector<Vector2> points = arcPoints(40);
    for (Vector2& point : points) point = point * 1e-30f;
    std::vector<Curve2> pieces = Curve2::fitted(points, 1e-32f);
    EXPECT(!pieces.empty());
    EXPECT(largestDistance(points, pieces) <= 1e-32);
}

TEST(fitCoordinatesNearTheFloatLimitFit) {
    std::vector<Vector2> circle = arcPoints(40, 3e38f);
    std::vector<Curve2> pieces = Curve2::fitted(circle, 3e36f);
    EXPECT(!pieces.empty());
    EXPECT(allFinite(pieces));
    EXPECT(largestDistance(circle, pieces) <= 3e36);
    std::vector<Vector2> corners = {Vector2(-3e38f, -3e38f), Vector2(0, 1e38f), Vector2(3e38f, 3e38f), Vector2(3e38f, -3e38f)};
    std::vector<Curve2> cornerPieces = Curve2::fitted(corners, 1e36f);
    EXPECT(!cornerPieces.empty());
    EXPECT(allFinite(cornerPieces));
    EXPECT(cornerPieces.front().start == corners.front() && cornerPieces.back().end == corners.back());
    EXPECT(largestDistance(corners, cornerPieces) <= 1e36);
}

TEST(fitClosePointsAreKeptSoTheyStayWithinTolerance) {
    std::vector<Vector2> points;
    for (int index = 0; index <= 1000; index += 1) points.push_back(Vector2(static_cast<float>(index), 0));
    points.insert(points.begin() + 501, Vector2(500.0002f, 4e-4f));
    std::vector<Curve2> pieces = Curve2::fitted(points, 1e-4f);
    float floor = Curve2::smallestFittingTolerance(points);
    EXPECT(floor == 4 * ulpOf(1000));
    EXPECT(largestDistance(points, pieces) <= static_cast<double>(floor));
}

TEST(fitTheToleranceFloorIsFourFloatSteps) {
    EXPECT(Curve2::smallestFittingTolerance({Vector2(1, 0), Vector2(0, -1)}) == 4 * 0x1p-23f);
    EXPECT(Curve2::smallestFittingTolerance({Vector2(1e5f, 0), Vector2(0, 1)}) == 4 * 0x1p-7f);
    EXPECT(Curve2::smallestFittingTolerance({Vector2(0, -1000001)}) == 4 * 0x1p-4f);
    EXPECT(Curve2::smallestFittingTolerance({Vector2()}) == 4 * std::numeric_limits<float>::denorm_min());
}

TEST(fitRoundingTheHandlesToFloatKeepsEveryPointWithinTolerance) {
    for (auto [offset, tolerance] : std::vector<std::pair<float, float>>{{1e5f, 0.01f}, {1e6f, 0.05f}, {0, 1e-7f}}) {
        std::vector<Vector2> points = arcPoints(40, 1, Vector2(offset, offset));
        float floor = Curve2::smallestFittingTolerance(points);
        for (float used : {largerOf(tolerance, floor), floor * 1.5f, floor * 2}) {
            std::vector<Curve2> pieces = Curve2::fitted(points, used);
            EXPECT(!pieces.empty());
            EXPECT(largestDistance(points, pieces) <= static_cast<double>(used));
        }
    }
}

TEST(fitATightUTurnNeedsFewPieces) {
    Curve2 uTurn(Vector2(0, 0), Vector2(0, 10), Vector2(1, 10), Vector2(1, 0));
    std::vector<Vector2> points = samples(uTurn, 100);
    for (auto [tolerance, expectedPieces] : std::vector<std::pair<float, size_t>>{{0.1f, 4}, {0.01f, 3}, {0.001f, 7}}) {
        std::vector<Curve2> pieces = Curve2::fitted(points, tolerance);
        EXPECT(pieces.size() == expectedPieces);
        EXPECT(largestDistance(points, pieces) <= static_cast<double>(tolerance));
    }
    EXPECT(matchesBits(Curve2::fitted(points, 0.01f), expected_fitU0_01));
    EXPECT(matchesBits(Curve2::fitted(points, 0.001f), expected_fitU0_001));
}

TEST(fitClosedSaysWhetherTheStrokeIsALoop) {
    Vector2 a(0, 0), b(1, 0), c(0, 1);
    std::vector<Curve2> thereAndBack = Curve2::fitted({a, b, a}, 0.01f);
    EXPECT(thereAndBack.size() == 2);
    EXPECT(matchesBits(thereAndBack, expected_fitThereBack));
    EXPECT(!Path2D::fitted({a, b, a}, 0.01f)->isClosed());
    EXPECT(Curve2::fitted({a, b, a}, 0.01f, std::nullopt, std::nullopt, true).empty());
    std::vector<Curve2> triangle = Curve2::fitted({a, b, c}, 0.01f, std::nullopt, std::nullopt, true);
    EXPECT(triangle.size() >= 2);
    EXPECT(triangle.front().start == a && triangle.back().end == a);
    EXPECT(Path2D::fitted({a, b, c}, 0.01f, std::nullopt, std::nullopt, true)->isClosed());
    EXPECT(matchesBits(triangle, expected_fitTriangleClosed));
    std::vector<Vector2> loop = arcPoints(49, 1, Vector2(), 2 * swiftPi);
    loop[48] = loop[0];
    std::optional<Path2D> open = Path2D::fitted(loop, 0.002f, std::nullopt, std::nullopt, false);
    EXPECT(open.has_value() && !open->isClosed());
    EXPECT(Path2D::fitted(loop, 0.002f)->isClosed());
    std::vector<Curve2> openPieces = Curve2::fitted(loop, 0.002f, std::nullopt, std::nullopt, false);
    EXPECT(largestDistance(loop, openPieces) <= 0.002);
}

// MARK: - Offset (CubicBezierOffsetTests)

namespace {

/// The arch x = 3t, y = 3t(1 − t): it turns 90° (from 45° up to 45° down).
const Curve2 arch(Vector2(0, 0), Vector2(1, 1), Vector2(2, 1), Vector2(3, 0));

/// The largest error of `pieces` as the offset of `curve` by a distance linear in the parameter,
/// measured in Double: each piece is sampled, each sample projected onto the original, and its
/// distance there compared with the distance wanted at that parameter.
double largestError(const std::vector<Curve2>& pieces, const Curve2& curve, float startDistance, float endDistance) {
    ExactCubic original(curve);
    double largest = 0;
    for (const Curve2& piece : pieces) {
        ExactCubic exactPiece(piece);
        for (int step = 0; step <= 40; step += 1) {
            ExactVector sample = exactPiece.point(static_cast<double>(step) / 40);
            auto [parameter, distance] = original.nearest(sample);
            double wanted = static_cast<double>(startDistance) + (static_cast<double>(endDistance) - startDistance) * parameter;
            ExactVector side = original.derivative(parameter);
            ExactVector at = original.point(parameter);
            double crossValue = side.x * (sample.y - at.y) - side.y * (sample.x - at.x);
            double signedDistance = crossValue <= 0 ? distance : -distance;
            largest = std::max(largest, std::fabs(signedDistance - wanted));
        }
    }
    return largest;
}

/// The largest distance from a sample of `pieces` to the true offset curve O(t) = B(t) + d(t)·N(t),
/// in Double, the true curve drawn as a polyline through 4000 parameter steps.
double largestDistanceFromTrueOffset(
    const std::vector<Curve2>& pieces, const Curve2& curve, float startDistance, float endDistance) {
    ExactCubic original(curve);
    int steps = 4000;
    std::vector<ExactVector> polyline;
    for (int step = 0; step <= steps; step += 1) {
        double t = static_cast<double>(step) / steps;
        ExactVector tangent = original.derivative(t);
        if (tangent.x * tangent.x + tangent.y * tangent.y == 0) {
            ExactVector second = original.secondDerivative(t);
            double sign = t == 0 ? 1 : -1;
            tangent = ExactVector{second.x * sign, second.y * sign};
        }
        double size = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        ExactVector unit{tangent.x / size, tangent.y / size};
        double distance = static_cast<double>(startDistance) + (static_cast<double>(endDistance) - startDistance) * t;
        ExactVector at = original.point(t);
        polyline.push_back(ExactVector{at.x + unit.y * distance, at.y - unit.x * distance});
    }
    double largest = 0;
    for (const Curve2& piece : pieces) {
        ExactCubic exactPiece(piece);
        for (int step = 0; step <= 40; step += 1) {
            ExactVector sample = exactPiece.point(static_cast<double>(step) / 40);
            double nearest = std::numeric_limits<double>::infinity();
            for (size_t index = 0; index + 1 < polyline.size(); index += 1) {
                ExactVector from = polyline[index];
                ExactVector chord{polyline[index + 1].x - from.x, polyline[index + 1].y - from.y};
                double along = ((sample.x - from.x) * chord.x + (sample.y - from.y) * chord.y)
                    / std::max(chord.x * chord.x + chord.y * chord.y, std::numeric_limits<double>::min());
                along = std::min(std::max(along, 0.0), 1.0);
                double gapX = sample.x - (from.x + chord.x * along);
                double gapY = sample.y - (from.y + chord.y * along);
                nearest = std::min(nearest, std::sqrt(gapX * gapX + gapY * gapY));
            }
            largest = std::max(largest, nearest);
        }
    }
    return largest;
}

/// The smallest radius of curvature of `curve`, and how close two parts of it at least a quarter
/// of the parameter range apart come, from 200 samples in Double.
std::pair<double, double> tightestBendAndClosestApproach(const Curve2& curve) {
    ExactCubic original(curve);
    double radius = std::numeric_limits<double>::infinity();
    std::vector<ExactVector> points;
    for (int step = 0; step <= 200; step += 1) {
        double t = static_cast<double>(step) / 200;
        ExactVector first = original.derivative(t);
        ExactVector second = original.secondDerivative(t);
        double speed = std::sqrt(first.x * first.x + first.y * first.y);
        double bend = std::fabs(first.x * second.y - first.y * second.x);
        radius = std::min(radius, bend == 0 ? std::numeric_limits<double>::infinity() : speed * speed * speed / bend);
        points.push_back(original.point(t));
    }
    double approach = std::numeric_limits<double>::infinity();
    for (size_t first = 0; first < points.size(); first += 1) {
        for (size_t second = first + 50; second < points.size(); second += 1) {
            approach = std::min(approach, std::hypot(points[first].x - points[second].x, points[first].y - points[second].y));
        }
    }
    return {radius, approach};
}

/// A random curve with every control point in −1…1.
Curve2 randomCurve(SeededRandom& generator) {
    auto point = [&generator] {
        float x = generator.nextFloat(-1, 1);
        float y = generator.nextFloat(-1, 1);
        return Vector2(x, y);
    };
    Vector2 start = point();
    Vector2 startHandle = point();
    Vector2 endHandle = point();
    Vector2 end = point();
    return Curve2(start, startHandle, endHandle, end);
}

} // namespace

TEST(offsetStraightCurveOffsetsToTheParallelLine) {
    Curve2 straight = Path2D::evenlyPacedCubic(Vector2(0, 0), Vector2(3, 0));
    auto pieces = straight.offset(1, 1e-4f);
    EXPECT(pieces.has_value() && pieces->size() == 1);
    EXPECT(pieces->front().start == Vector2(0, -1));
    EXPECT(pieces->front().end == Vector2(3, -1));
    EXPECT(isClose(pieces->front().startHandle, Vector2(1, -1)));
    EXPECT(isClose(pieces->front().endHandle, Vector2(2, -1)));
}

TEST(offsetArchOffsetStaysAtTheDistance) {
    for (float distance : {0.1f, -0.1f, 0.3f}) {
        auto pieces = arch.offset(distance, 1e-4f);
        EXPECT(pieces.has_value() && !pieces->empty());
        EXPECT(isClose(pieces->front().start, arch.start + arch.startDirection().rightNormal() * distance));
        EXPECT(isClose(pieces->back().end, arch.end + arch.endDirection().rightNormal() * distance));
        for (size_t index = 1; index < pieces->size(); index += 1) EXPECT((*pieces)[index - 1].end == (*pieces)[index].start);
        EXPECT(largestError(*pieces, arch, distance, distance) <= 1e-4);
    }
}

TEST(offsetVaryingWidthHasTheRequestedEndValues) {
    auto pieces = arch.offset(0.2f, 0.05f, 1e-4f);
    EXPECT(pieces.has_value());
    Vector2 startNormal = Vector2(1, 1).unitDirection()->rightNormal();
    Vector2 endNormal = Vector2(1, -1).unitDirection()->rightNormal();
    EXPECT(isClose(pieces->front().start, startNormal * 0.2f));
    EXPECT(isClose(pieces->back().end, Vector2(3, 0) + endNormal * 0.05f));
    EXPECT(largestError(*pieces, arch, 0.2f, 0.05f) <= 1e-4);
}

TEST(offsetPointsSitAtTheRequestedDistance) {
    SeededRandom generator(61);
    double worst = 0;
    int measured = 0;
    for (int trial = 0; trial < 150; trial += 1) {
        Curve2 curve = randomCurve(generator);
        float distance = 0.02f;
        auto [radius, approach] = tightestBendAndClosestApproach(curve);
        if (!(radius > 4 * static_cast<double>(distance) && approach > 4 * static_cast<double>(distance)
              && !curve.selfCrossingParameters().has_value())) {
            continue;
        }
        measured += 1;
        auto pieces = curve.offset(distance, 1e-4f);
        EXPECT(pieces.has_value());
        if (pieces) worst = std::max(worst, largestError(*pieces, curve, distance, distance));
    }
    EXPECT(measured > 20);
    EXPECT(worst <= 1e-4);
}

TEST(offsetVaryingWidthOnRandomCurvesNeedsFewPiecesAndMeetsTheTolerance) {
    SeededRandom generator(83);
    double worst = 0;
    size_t mostPieces = 0;
    int measured = 0;
    for (int trial = 0; trial < 250; trial += 1) {
        Curve2 curve = randomCurve(generator);
        for (auto [startDistance, endDistance] : std::vector<std::pair<float, float>>{{0.01f, 0.1f}, {-0.01f, -0.1f}}) {
            auto [radius, approach] = tightestBendAndClosestApproach(curve);
            if (!(radius > 2 * 0.1 && approach > 3 * 0.1 && !curve.selfCrossingParameters().has_value())) continue;
            measured += 1;
            auto pieces = curve.offset(startDistance, endDistance, 1e-4f);
            EXPECT(pieces.has_value());
            if (!pieces) continue;
            mostPieces = std::max(mostPieces, pieces->size());
            worst = std::max(worst, largestError(*pieces, curve, startDistance, endDistance));
        }
    }
    EXPECT(measured > 30);
    EXPECT(mostPieces <= 48);
    EXPECT(worst <= 1e-4);
}

TEST(offsetEveryRandomCurveCanBeOffset) {
    SeededRandom generator(99);
    size_t mostPieces = 0;
    for (int trial = 0; trial < 300; trial += 1) {
        Curve2 curve = randomCurve(generator);
        auto pieces = curve.offset(0.05f, 0.3f, 1e-4f);
        EXPECT(pieces.has_value());
        if (pieces) mostPieces = std::max(mostPieces, pieces->size());
    }
    EXPECT(mostPieces <= 160);
}

TEST(offsetNearlyStraightVaryingWidthKeepsItsHandlesInPlace) {
    Curve2 flat(Vector2(), Vector2(1, 2e-3f), Vector2(2, 2e-3f), Vector2(3, 0));
    auto pieces = flat.offset(0, 1, 1e-4f);
    EXPECT(pieces.has_value());
    EXPECT(isClose(pieces->front().startHandle, Vector2(1.0006667f, -0.3313327f), 1e-5f));
    EXPECT(pieces->size() == 1);
    EXPECT(largestError(*pieces, flat, 0, 1) < 2e-8);
}

TEST(offsetHandleOnItsEndPointUsesTheLimitingBend) {
    Curve2 pulled(Vector2(), Vector2(), Vector2(2, 1), Vector2(3, 0));
    auto pieces = pulled.offset(0.05f, 0.15f, 1e-4f);
    EXPECT(pieces.has_value());
    EXPECT(largestDistanceFromTrueOffset(*pieces, pulled, 0.05f, 0.15f) <= 1e-4);
}

TEST(offsetACuspHasNoOffset) {
    Curve2 cusp(Vector2(), Vector2(1, 1), Vector2(0, 1), Vector2(1, 0));
    EXPECT(!cusp.offset(0.1f, 1e-4f).has_value());
}

TEST(offsetABrokenFitMeasuresAsInfinitelyFar) {
    Curve2 nan(Vector2(), Vector2(notANumber, 0), Vector2(2, 1), Vector2(3, 0));
    EXPECT(arch.largestOffsetError(nan, ParameterRange{0, 1}, 0.1f, 0.1f) == infinity);
}

TEST(offsetAtEveryScale) {
    for (auto [scale, shift] : std::vector<std::pair<float, Vector2>>{{1e-3f, Vector2()}, {1e3f, Vector2()}, {1, Vector2(1000, 1000)}}) {
        Curve2 placed(
            arch.start * scale + shift, arch.startHandle * scale + shift, arch.endHandle * scale + shift,
            arch.end * scale + shift);
        float distance = 0.1f * scale;
        auto pieces = placed.offset(distance, 1e-4f * scale);
        EXPECT(pieces.has_value());
        if (!pieces) continue;
        double error = largestError(*pieces, placed, distance, distance);
        double allowed = std::max(1e-4 * static_cast<double>(scale), 4 * static_cast<double>(ulpOf(1000)) * 3);
        EXPECT(error < allowed);
    }
}

TEST(offsetSimplePiecesTurnLessThanTheLimit) {
    auto archWithHalfTurn = [](Angle halfTurn) {
        float rise = static_cast<float>(std::tan(static_cast<double>(halfTurn.radians)));
        return Curve2(Vector2(), Vector2(1, rise), Vector2(2, rise), Vector2(3, 0));
    };
    EXPECT(archWithHalfTurn(Angle::degrees(29.5f)).isSimpleForOffset());
    EXPECT(!archWithHalfTurn(Angle::degrees(30.5f)).isSimpleForOffset());
    EXPECT(!arch.isSimpleForOffset());
    std::vector<Curve2> pieces = arch.simplePiecesForOffset();
    EXPECT(pieces.size() >= 2);
    for (const Curve2& piece : pieces) EXPECT(piece.isSimpleForOffset());
    EXPECT(pieces.front().start == arch.start);
    EXPECT(pieces.back().end == arch.end);
    Curve2 wave(Vector2(), Vector2(1, 1), Vector2(2, -1), Vector2(3, 0));
    EXPECT(!wave.isSimpleForOffset());
    EXPECT(wave.simplePiecesForOffset().size() >= 2);
    Curve2 tinyWave(Vector2(), Vector2(1e-20f, 1e-20f), Vector2(2e-20f, -1e-20f), Vector2(3e-20f, 0));
    EXPECT(!tinyWave.isSimpleForOffset());
    // The pieces an S is cut into, as the Swift engine cuts it.
    Curve2 sCurve(Vector2(0, 0), Vector2(2, 3), Vector2(2, -3), Vector2(4, 0));
    std::vector<ParameterRange> ranges = sCurve.simplePieceParameters();
    std::vector<std::array<std::uint32_t, 2>> expectedRanges = {
        {0x0, 0x3e000000}, {0x3e000000, 0x3e800000}, {0x3e800000, 0x3f000000},
        {0x3f000000, 0x3f400000}, {0x3f400000, 0x3f600000}, {0x3f600000, 0x3f800000}};
    EXPECT(ranges.size() == expectedRanges.size());
    for (size_t index = 0; index < ranges.size() && index < expectedRanges.size(); index += 1) {
        EXPECT(sameBits(ranges[index].lowerBound, expectedRanges[index][0]));
        EXPECT(sameBits(ranges[index].upperBound, expectedRanges[index][1]));
    }
}

TEST(offsetNearlyStraightCurvesOffsetCleanly) {
    Curve2 flat(Vector2(), Vector2(1, 5e-4f), Vector2(2, 5e-4f), Vector2(3, 0));
    auto pieces = flat.offset(0.1f, 1e-5f);
    EXPECT(pieces.has_value());
    EXPECT(allFinite(*pieces));
    EXPECT(largestError(*pieces, flat, 0.1f, 0.1f) < 1.5e-8);
    Curve2 bent(Vector2(), Vector2(1, 2e-3f), Vector2(2, 2e-3f), Vector2(3, 0));
    auto bentPieces = bent.offset(0.1f, 1e-5f);
    EXPECT(bentPieces.has_value());
    EXPECT(largestError(*bentPieces, bent, 0.1f, 0.1f) < 1e-8);
}

TEST(offsetDegenerateCurvesHaveNoOffset) {
    Curve2 point(Vector2(1, 1), Vector2(1, 1), Vector2(1, 1), Vector2(1, 1));
    EXPECT(!point.offset(0.1f, 1e-4f).has_value());
    EXPECT(!arch.offset(notANumber, 1e-4f).has_value());
    EXPECT(!arch.offset(0.1f, infinity, 1e-4f).has_value());
    EXPECT_PROBLEM(arch.offset(0.1f, 0), "An offset needs a positive, finite tolerance.");
}

TEST(offsetBitForBitAgainstTheSwiftEngine) {
    EXPECT(matchesBits(*knownCubic.offset(0.2f, 0.001f), expected_offsetKnown0_2));
    EXPECT(matchesBits(*knownCubic.offset(-0.2f, 0.001f), expected_offsetKnown_0_2));
    EXPECT(matchesBits(*knownCubic.offset(0.1f, 0.5f, 0.0005f), expected_offsetVar));
    Curve2 sCurve(Vector2(0, 0), Vector2(2, 3), Vector2(2, -3), Vector2(4, 0));
    EXPECT(matchesBits(*sCurve.offset(0.3f, 0.001f), expected_offsetS));
}

// MARK: - Path offset (Path2DOffsetTests)

namespace {

/// A square of side `side` with its lower-left corner at `corner`, counterclockwise.
Path2D square(float side = 1, Vector2 corner = Vector2()) {
    Path2D path(corner);
    path.addLine(corner + Vector2(side, 0));
    path.addLine(corner + Vector2(side, side));
    path.addLine(corner + Vector2(0, side));
    path.close();
    return path;
}

/// A circle of radius `radius` around `centre`, counterclockwise, as four quarter arcs.
Path2D circle(float radius, Vector2 centre = Vector2()) {
    Path2D path(centre + Vector2(radius, 0));
    path.addArc(centre, radius, Angle::degrees(0), Angle::degrees(360));
    path.close();
    return path;
}

/// The area inside four cubic quarter arcs of radius `radius`: πr² plus the 8.8e-4·r² the cubics
/// bulge past the true circle. Fillets are made of these arcs, so their areas are compared with this.
float cubicCircleArea(float radius) { return circle(radius).signedArea(); }

using Join = Path2D::OffsetJoin;

} // namespace

TEST(pathOffsetSquareMiterOffsetIsTheLargerSquare) {
    auto grown = square().offset(0.1f, Join::miter(4), 1e-4f);
    EXPECT(grown.has_value());
    EXPECT(isApproximatelyEqual(grown->signedArea(), 1.44f));
    EXPECT(grown->isClosed());
    EXPECT(isClose(grown->bounds().minimum, Vector2(-0.1f, -0.1f)));
    EXPECT(isClose(grown->bounds().maximum, Vector2(1.1f, 1.1f)));
}

TEST(pathOffsetSquareRoundOffsetAddsQuarterCirclesAtTheCorners) {
    auto grown = square().offset(0.1f, Join::round(), 1e-4f);
    EXPECT(grown.has_value());
    float expected = 1 + 0.4f + circle(0.1f).signedArea();
    EXPECT(std::fabs(grown->signedArea() - expected) < 1e-6f);
}

TEST(pathOffsetSquareBevelOffsetCutsTheCornersOff) {
    auto grown = square().offset(0.1f, Join::bevel(), 1e-4f);
    EXPECT(grown.has_value());
    EXPECT(isApproximatelyEqual(grown->signedArea(), 1.44f - 4 * 0.005f));
}

TEST(pathOffsetMiterLimitSwitchesToBevelAtTheThreshold) {
    auto justAllowed = square().offset(0.1f, Join::miter(1.415f), 1e-4f);
    auto justRefused = square().offset(0.1f, Join::miter(1.413f), 1e-4f);
    EXPECT(isApproximatelyEqual(justAllowed->signedArea(), 1.44f));
    EXPECT(isApproximatelyEqual(justRefused->signedArea(), 1.42f));
    auto nanLimit = square().offset(0.1f, Join::miter(notANumber), 1e-4f);
    EXPECT(isApproximatelyEqual(nanLimit->signedArea(), 1.42f));
}

TEST(pathOffsetInsetJoinsInnerCornersStraightAcross) {
    auto shrunk = square().offset(-0.1f, Join::round(), 1e-4f);
    EXPECT(shrunk.has_value());
    EXPECT(isApproximatelyEqual(shrunk->signedArea(), 0.64f - 4 * 0.005f));
    EXPECT(shrunk->windingNumber(Vector2(0.5f, 0.5f)) == 1);
}

TEST(pathOffsetCircleOffsetIsTheLargerCircle) {
    for (float distance : {0.25f, -0.25f}) {
        auto offsetCircle = circle(1).offset(distance, Join::round(), 1e-5f);
        EXPECT(offsetCircle.has_value());
        float worst = 0;
        for (const Path2D::Edge& edge : offsetCircle->edgesWithClosingLine()) {
            std::optional<Curve2> curve = edge.cubic();
            if (!curve) {
                EXPECT((edge.segment.end() - edge.start).length() < 1e-5f);
                continue;
            }
            for (int step = 0; step <= 20; step += 1) {
                float radius = curve->point(static_cast<float>(step) / 20).length();
                worst = largerOf(worst, std::fabs(radius - (1 + distance)));
            }
        }
        EXPECT(worst < 3e-4f);
    }
}

TEST(pathOffsetOpenPolylineMitersItsCornerAndKeepsItsEnds) {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(1, 0));
    path.addLine(Vector2(1, 1));
    auto side = path.offset(0.1f, Join::miter(4), 1e-4f);
    EXPECT(side.has_value());
    EXPECT(!side->isClosed());
    EXPECT(side->start == Vector2(0, -0.1f));
    EXPECT(isClose(side->currentPoint(), Vector2(1.1f, 1)));
    bool found = false;
    for (Vector2 point : side->flattened(1e-4f)) found = found || isClose(point, Vector2(1.1f, -0.1f));
    EXPECT(found);
}

TEST(pathOffsetVaryingWidthGrowsWithTheLengthWalked) {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(1, 0));
    path.addLine(Vector2(1, 1));
    auto side = path.offset(0.2f, 0, Join::bevel(), 1e-4f);
    EXPECT(side.has_value());
    EXPECT(side->start == Vector2(0, -0.2f));
    EXPECT(isClose(side->currentPoint(), Vector2(1, 1)));
    std::vector<Path2D::Edge> edges = side->edges();
    EXPECT(isClose(edges.at(0).segment.end(), Vector2(1, -0.1f)));
    EXPECT(isClose(edges.at(1).segment.end(), Vector2(1.1f, 0)));
}

TEST(pathOffsetVaryingWidthOnACurveHasTheRequestedEndValues) {
    Path2D path{Vector2()};
    path.addCubic(Vector2(1, 1), Vector2(2, 1), Vector2(3, 0));
    auto side = path.offset(0.3f, 0.1f, Join::round(), 1e-4f);
    EXPECT(side.has_value());
    EXPECT(isClose(side->start, Vector2(1, 1).unitDirection()->rightNormal() * 0.3f));
    EXPECT(isClose(side->currentPoint(), Vector2(3, 0) + Vector2(1, -1).unitDirection()->rightNormal() * 0.1f));
}

TEST(pathOffsetAtEveryScale) {
    for (auto [scale, shift] : std::vector<std::pair<float, Vector2>>{{1e-3f, Vector2()}, {1e3f, Vector2()}, {1, Vector2(1000, 1000)}}) {
        Path2D placed = square(scale, shift);
        float distance = 0.1f * scale;
        auto miter = placed.offset(distance, Join::miter(4), 1e-4f * scale);
        auto round = placed.offset(distance, Join::round(), 1e-4f * scale);
        EXPECT(miter.has_value() && round.has_value());
        if (!miter || !round) continue;
        float area = scale * scale;
        EXPECT(isApproximatelyEqual(miter->signedArea() / area, 1.44f, 1e-3f));
        EXPECT(isApproximatelyEqual(round->signedArea() / area, 1.4f + swiftPi * 0.01f, 1e-3f));
        EXPECT(isClose(miter->bounds().minimum, shift - Vector2(distance, distance), 1e-4f * scale));
    }
}

TEST(pathOffsetEmptyOrNonFiniteOffsetsAreNil) {
    Path2D point(Vector2(1, 1));
    point.addLine(Vector2(1, 1));
    EXPECT(!point.offset(0.1f, Join::round(), 1e-4f).has_value());
    EXPECT(!square().offset(notANumber, Join::round(), 1e-4f).has_value());
    EXPECT(!square().offset(0, infinity, Join::round(), 1e-4f).has_value());
    EXPECT_PROBLEM(square().offset(0.1f, Join::round(), 0), "An offset needs a positive, finite tolerance.");
}

TEST(pathOffsetNonFiniteEdgesHaveNoLength) {
    EXPECT(!(Path2D::Edge{Vector2(), Path2D::Segment::line(Vector2(1, notANumber))}.hasLength()));
    EXPECT(!(Path2D::Edge{Vector2(notANumber, 0), Path2D::Segment::line(Vector2(1, 0))}.hasLength()));
    EXPECT((Path2D::Edge{Vector2(), Path2D::Segment::line(Vector2(1, 0))}.hasLength()));
}

TEST(pathOffsetACuspCannotBeOffset) {
    Path2D cusp{Vector2()};
    cusp.addCubic(Vector2(1, 1), Vector2(0, 1), Vector2(1, 0));
    EXPECT(!cusp.offset(0.1f, Join::round(), 1e-4f).has_value());
}

TEST(pathOffsetZeroOffsetGivesTheSameOutline) {
    auto same = square().offset(0, Join::miter(4), 1e-4f);
    EXPECT(same.has_value());
    EXPECT(isApproximatelyEqual(same->signedArea(), 1));
}

TEST(pathOffsetBitForBitAgainstTheSwiftEngine) {
    EXPECT(matchesBits(square().offset(0.25f, Join::miter(4), 0.001f), expected_sqOutMiter));
    EXPECT(matchesBits(square().offset(0.25f, Join::round(), 0.001f), expected_sqOutRound));
    EXPECT(matchesBits(square().offset(0.25f, Join::bevel(), 0.001f), expected_sqOutBevel));
    EXPECT(matchesBits(square().offset(-0.1f, Join::miter(4), 0.001f), expected_sqIn));
    Path2D open{Vector2()};
    open.addCubic(Vector2(1, 3), Vector2(3, 3), Vector2(4, 0));
    open.addLine(Vector2(6, -1));
    EXPECT(matchesBits(open.offset(0.1f, 0.4f, Join::round(), 0.001f), expected_openMix));
}

// MARK: - Corners (Path2DCornersTests)

TEST(cornersSquareHasFourRightCorners) {
    std::vector<PathCorner> corners = square().corners();
    EXPECT(corners.size() == 4);
    for (const PathCorner& corner : corners) {
        EXPECT(isApproximatelyEqual(corner.turn().radians, Angle::degrees(90).radians));
        EXPECT(isApproximatelyEqual(corner.interiorAngle().radians, Angle::degrees(90).radians));
    }
    if (corners.size() == 4) {
        EXPECT(corners[0].point == Vector2(0, 0) && corners[1].point == Vector2(1, 0));
        EXPECT(corners[2].point == Vector2(1, 1) && corners[3].point == Vector2(0, 1));
        for (int index = 0; index < 4; index += 1) EXPECT(corners[index].outgoingPieceIndex == index);
    }
}

TEST(cornersClockwiseCornersTurnRight) {
    Path2D path{Vector2()};
    path.addLine(Vector2(0, 1));
    path.addLine(Vector2(1, 1));
    std::vector<PathCorner> corners = path.corners();
    EXPECT(corners.size() == 1);
    EXPECT(isApproximatelyEqual(corners.at(0).turn().radians, Angle::degrees(-90).radians));
}

TEST(cornersMinimumTurnSkipsSmallKinksInsideTheBand) {
    auto kinked = [](Angle turn) {
        Path2D path{Vector2()};
        path.addLine(Vector2(1, 0));
        path.addLine(Vector2(1, 0) + Vector2(static_cast<float>(std::cos(static_cast<double>(turn.radians))),
                                             static_cast<float>(std::sin(static_cast<double>(turn.radians)))));
        return path;
    };
    EXPECT(kinked(Angle::degrees(0.9f)).corners().empty());
    EXPECT(kinked(Angle::degrees(1.1f)).corners().size() == 1);
}

TEST(cornersSmoothCircleHasNoCorners) {
    Path2D smooth = circle(1);
    EXPECT(smooth.corners().empty());
    EXPECT(smooth.filleted(0.1f).signedArea() == smooth.signedArea());
}

TEST(cornersFilletedSquareLosesTheCornerAreas) {
    for (float radius : {0.1f, 0.25f, 0.4f}) {
        Path2D rounded = square().filleted(radius);
        float expected = 1 - 4 * radius * radius + cubicCircleArea(radius);
        EXPECT(std::fabs(rounded.signedArea() - expected) < 4e-7f);
        EXPECT(rounded.isClosed());
        EXPECT(rounded.corners().empty());
    }
}

TEST(cornersFilletTouchesBothSidesAtTheTangentLength) {
    Path2D rounded = square().filleted(0.2f);
    EXPECT(rounded.start == Vector2(0.2f, 0));
    bool foundFirst = false;
    bool foundSecond = false;
    for (const Path2D::Edge& edge : rounded.edges()) {
        if (edge.cubic()) continue;
        foundFirst = foundFirst || isClose(edge.segment.end(), Vector2(0.8f, 0));
        foundSecond = foundSecond || isClose(edge.segment.end(), Vector2(1, 0.8f));
    }
    EXPECT(foundFirst && foundSecond);
}

TEST(cornersOversizedFilletIsClampedToHalfOfEachSide) {
    Path2D rounded = square().filleted(1);
    EXPECT(std::fabs(rounded.signedArea() - cubicCircleArea(0.5f)) < 1e-6f);
}

TEST(cornersOpenPathKeepsItsEnds) {
    Path2D path{Vector2()};
    path.addLine(Vector2(1, 0));
    path.addLine(Vector2(1, 1));
    Path2D rounded = path.filleted(0.2f);
    EXPECT(rounded.start == Vector2(0, 0));
    EXPECT(rounded.currentPoint() == Vector2(1, 1));
    EXPECT(!rounded.isClosed());
    Path2D chamfered = path.chamfered(0.2f);
    EXPECT(chamfered.start == Vector2(0, 0));
    EXPECT(chamfered.currentPoint() == Vector2(1, 1));
    std::vector<Path2D::Edge> edges = chamfered.edges();
    EXPECT(edges.size() == 3);
    if (edges.size() == 3) {
        EXPECT(edges[0].segment.end() == Vector2(0.8f, 0));
        EXPECT(edges[1].segment.end() == Vector2(1, 0.2f));
        EXPECT(edges[2].segment.end() == Vector2(1, 1));
    }
}

TEST(cornersChamferedSquareLosesFourTriangles) {
    for (float cut : {0.1f, 0.3f}) {
        EXPECT(isApproximatelyEqual(square().chamfered(cut).signedArea(), 1 - 2 * cut * cut));
    }
}

TEST(cornersPerCornerRadiusAndInvalidRadiiLeaveCornersSharp) {
    std::vector<float> radii = {notANumber, -1, 0.2f, 0};
    Path2D rounded = square().filleted([&radii](const PathCorner& corner) { return radii[corner.outgoingPieceIndex]; });
    EXPECT(isApproximatelyEqual(rounded.signedArea(), 1 - 0.04f + cubicCircleArea(0.2f) / 4));
    EXPECT(rounded.corners().size() == 3);
}

TEST(cornersFilletNextToACurveMeetsItTangentially) {
    Path2D path{Vector2()};
    path.addLine(Vector2(2, 0));
    path.addCubic(Vector2(2, 1), Vector2(1, 2), Vector2(0, 2));
    path.close();
    Path2D rounded = path.filleted(0.2f);
    EXPECT(rounded.corners().empty());
    EXPECT(rounded.signedArea() < path.signedArea());
}

TEST(cornersFilletAndChamferAtEveryScale) {
    for (auto [scale, shift] : std::vector<std::pair<float, Vector2>>{{1e-3f, Vector2()}, {1e3f, Vector2()}, {1, Vector2(1000, 1000)}}) {
        Path2D placed = square(scale, shift);
        float area = scale * scale;
        Path2D rounded = placed.filleted(0.25f * scale);
        Path2D cut = placed.chamfered(0.25f * scale);
        EXPECT(isApproximatelyEqual(rounded.signedArea() / area, 1 - (4 - swiftPi) * 0.0625f, 1e-3f));
        EXPECT(isApproximatelyEqual(cut.signedArea() / area, 1 - 2 * 0.0625f, 1e-3f));
        EXPECT(rounded.corners().empty());
        EXPECT(cut.corners().size() == 8);
    }
}

TEST(cornersAPathWithOnlyAReversalIsUnchanged) {
    Path2D hairpin{Vector2()};
    hairpin.addLine(Vector2(1, 0));
    hairpin.addLine(Vector2(0, 0));
    hairpin.addLine(Vector2(0, 0));
    EXPECT(hairpin.filleted(0.1f) == hairpin);
    EXPECT(hairpin.chamfered(0.1f) == hairpin);
}

TEST(cornersBitForBitAgainstTheSwiftEngine) {
    EXPECT(matchesBits(square().filleted(0.2f), expected_sqFillet));
    EXPECT(matchesBits(square().chamfered(0.2f), expected_sqChamfer));
    EXPECT(matchesBits(square().filleted(5), expected_sqFilletClamp));
    Path2D open{Vector2()};
    open.addCubic(Vector2(1, 3), Vector2(3, 3), Vector2(4, 0));
    open.addLine(Vector2(6, -1));
    EXPECT(matchesBits(open.filleted(0.5f), expected_openMixFillet));
    Path2D rounded{Vector2()};
    rounded.addLine(Vector2(2, 0));
    rounded.addCubic(Vector2(3, 0), Vector2(3, 1), Vector2(2, 2));
    rounded.addLine(Vector2(0, 2));
    rounded.close();
    EXPECT(matchesBits(rounded.filleted(0.3f), expected_curveFillet));
    EXPECT(matchesBits(rounded.chamfered(0.3f), expected_curveChamfer));
    std::vector<PathCorner> corners = rounded.corners();
    EXPECT(corners.size() == 3);
    if (corners.size() == 3) {
        EXPECT(corners[0].outgoingPieceIndex == 0 && corners[1].outgoingPieceIndex == 2 && corners[2].outgoingPieceIndex == 3);
        EXPECT(sameBits(corners[0].turn().radians, 0x3fc90fdb) && sameBits(corners[0].interiorAngle().radians, 0x3fc90fdb));
        EXPECT(sameBits(corners[1].point.x, 0x40000000) && sameBits(corners[1].point.y, 0x40000000));
        EXPECT(sameBits(corners[1].turn().radians, 0x3f490fdb) && sameBits(corners[1].interiorAngle().radians, 0x4016cbe4));
    }
}
