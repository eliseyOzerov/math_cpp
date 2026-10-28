// Tests of Curves/ExactCoverage, the reference rasterizer. The hand-worked cases use squares and
// triangles whose pixel areas follow from the picture; the overlapping squares pin the fill
// rules where contours overlap inside one pixel; the circle converges to πr². The properties:
// every coverage lies in 0…1, the total is the shape's area within the flattening bound,
// polygons agree with point sampling of the winding number, and a whole-pixel shift moves the
// image. The picture is a "Q" (ring, hole and an overlapping tail) under both fill rules, written
// to build/visuals/math-curves/exact-coverage-glyph.pgm.
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <vector>

#include "../Curves/ExactCoverage.h"
#include "../Numerics/SeededRandom.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

Path2D polygonPath(const std::vector<Vector2>& corners) {
    Path2D path(corners.front());
    for (size_t index = 1; index < corners.size(); index += 1) path.addLine(corners[index]);
    path.close();
    return path;
}

Path2D rectanglePath(float left, float bottom, float right, float top, bool isCounterclockwise = true) {
    if (isCounterclockwise) return polygonPath({{left, bottom}, {right, bottom}, {right, top}, {left, top}});
    return polygonPath({{left, bottom}, {left, top}, {right, top}, {right, bottom}});
}

Path2D circlePath(Vector2 centre, float radius, bool isCounterclockwise = true) {
    Path2D path(centre + Vector2(radius, 0));
    path.addArc(centre, radius, Angle::zero(), Angle::degrees(isCounterclockwise ? 360 : -360));
    path.close();
    return path;
}

/// An upper bound on the length of a path's curved pieces: the length of their control polygons
/// (a Bézier curve is never longer than its control polygon).
float curvedLengthBound(const Shape2D& shape) {
    float total = 0;
    for (const Path2D& outline : shape.outlines) {
        Vector2 from = outline.start;
        for (const Path2D::Segment& segment : outline.segments()) {
            if (!segment.isLine()) {
                total += from.distance(segment.startHandle) + segment.startHandle.distance(segment.endHandle)
                    + segment.endHandle.distance(segment.end());
            }
            from = segment.end();
        }
    }
    return total;
}

bool hasEveryCoverageInUnitRange(const CoverageImage& image) {
    for (float pixelCoverage : image.coverages) {
        if (!(pixelCoverage >= -1e-6f && pixelCoverage <= 1 + 1e-6f)) return false;
    }
    return true;
}

/// A polygon of `cornerCount` random corners inside a `size` × `size` square: self-crossing,
/// overlapping itself, with windings above 1.
Shape2D randomPolygon(SeededRandom& random, int cornerCount, float size, FillRule fillRule) {
    std::vector<Vector2> corners;
    for (int index = 0; index < cornerCount; index += 1) {
        corners.push_back(Vector2(random.nextFloat(0.5f, size - 0.5f), random.nextFloat(0.5f, size - 0.5f)));
    }
    return Shape2D{{polygonPath(corners)}, fillRule};
}

/// The share of pixel (`column`, `row`) that `shape` fills by sampling the winding number at the
/// centres of a `samples` × `samples` grid of sub-squares.
float sampledCoverage(const Shape2D& shape, int column, int row, int samples) {
    int inside = 0;
    for (int sampleRow = 0; sampleRow < samples; sampleRow += 1) {
        for (int sampleColumn = 0; sampleColumn < samples; sampleColumn += 1) {
            Vector2 point(column + (sampleColumn + 0.5f) / samples, row + (sampleRow + 0.5f) / samples);
            if (shape.contains(point)) inside += 1;
        }
    }
    return static_cast<float>(inside) / static_cast<float>(samples * samples);
}

/// A letter "Q" in a 64 × 64 grid: a ring (outer circle counterclockwise, the counter clockwise)
/// and a tail drawn counterclockwise across the ring's lower right, so the outlines overlap.
Shape2D glyphQ(FillRule fillRule) {
    Path2D tail = polygonPath({{34, 20}, {54, 2}, {60, 8}, {40, 26}});
    return Shape2D{{circlePath(Vector2(30, 36), 24), circlePath(Vector2(30, 36), 14, false), tail}, fillRule};
}

/// Writes `images` side by side as one binary greyscale PGM, ink dark on white, each pixel drawn
/// as a `scale` × `scale` block, top row first (the images are y-up).
void writeSideBySide(const std::vector<CoverageImage>& images, int scale, const std::filesystem::path& file) {
    int gap = 2;
    int width = 0;
    int height = 0;
    for (const CoverageImage& image : images) {
        width += image.columns * scale + gap;
        height = std::max(height, image.rows * scale);
    }
    std::vector<unsigned char> grey(static_cast<size_t>(width) * height, 160);
    int left = 0;
    for (const CoverageImage& image : images) {
        for (int y = 0; y < image.rows * scale; y += 1) {
            int row = image.rows - 1 - y / scale;
            for (int x = 0; x < image.columns * scale; x += 1) {
                float ink = std::clamp(image.coverage(x / scale, row), 0.0f, 1.0f);
                grey[static_cast<size_t>(y) * width + left + x] = static_cast<unsigned char>(std::lround(255 * (1 - ink)));
            }
        }
        left += image.columns * scale + gap;
    }
    std::filesystem::create_directories(file.parent_path());
    std::ofstream out(file, std::ios::binary);
    out << "P5\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<const char*>(grey.data()), static_cast<std::streamsize>(grey.size()));
}

} // namespace

// MARK: Hand-worked

TEST(exactCoverageUnitSquareHalfCoversTwoPixels) {
    // [0.5, 1.5] × [0, 1]: the right half of pixel 0 and the left half of pixel 1.
    CoverageImage image = exactCoverage(Shape2D{{rectanglePath(0.5f, 0, 1.5f, 1)}}, 3, 1, 0.01f);
    EXPECT(isApproximatelyEqual(image.coverage(0, 0), 0.5f));
    EXPECT(isApproximatelyEqual(image.coverage(1, 0), 0.5f));
    EXPECT(isApproximatelyEqual(image.coverage(2, 0), 0));
    // [0.5, 1.5]²: a quarter of each of four pixels.
    CoverageImage corner = exactCoverage(Shape2D{{rectanglePath(0.5f, 0.5f, 1.5f, 1.5f)}}, 2, 2, 0.01f);
    for (float pixelCoverage : corner.coverages) EXPECT(isApproximatelyEqual(pixelCoverage, 0.25f));
}

TEST(exactCoverageTriangle) {
    // (0,0), (2,0), (0,1): under the line y = 1 − x/2. Pixel 0: ∫₀¹ (1 − x/2) dx = 3/4;
    // pixel 1: ∫₁² (1 − x/2) dx = 1/4.
    CoverageImage shallow = exactCoverage(Shape2D{{polygonPath({{0, 0}, {2, 0}, {0, 1}})}}, 2, 1, 0.01f);
    EXPECT(isApproximatelyEqual(shallow.coverage(0, 0), 0.75f));
    EXPECT(isApproximatelyEqual(shallow.coverage(1, 0), 0.25f));
    // (0,0), (3,0), (0,3): the diagonal x + y = 3 halves (0,2), (1,1) and (2,0); the three
    // pixels below it are full, the three above empty; 4.5 in all.
    CoverageImage diagonal = exactCoverage(Shape2D{{polygonPath({{0, 0}, {3, 0}, {0, 3}})}}, 3, 3, 0.01f);
    float expected[3][3] = {{1, 1, 0.5f}, {1, 0.5f, 0}, {0.5f, 0, 0}}; // [row][column]
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) {
            EXPECT(isApproximatelyEqual(diagonal.coverage(column, row), expected[row][column]));
        }
    }
    EXPECT(isApproximatelyEqual(diagonal.totalCoverage(), 4.5f));
}

TEST(exactCoverageOverlappingSquaresUnderBothFillRules) {
    // [0,2]² and [1,3]², both counterclockwise: they overlap in pixel (1,1), winding 2 there.
    std::vector<Path2D> squares{rectanglePath(0, 0, 2, 2), rectanglePath(1, 1, 3, 3)};
    CoverageImage nonzero = exactCoverage(Shape2D{squares, FillRule::nonzero}, 3, 3, 0.01f);
    CoverageImage evenOdd = exactCoverage(Shape2D{squares, FillRule::evenOdd}, 3, 3, 0.01f);
    float expectedNonzero[3][3] = {{1, 1, 0}, {1, 1, 1}, {0, 1, 1}};
    float expectedEvenOdd[3][3] = {{1, 1, 0}, {1, 0, 1}, {0, 1, 1}};
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) {
            EXPECT(isApproximatelyEqual(nonzero.coverage(column, row), expectedNonzero[row][column]));
            EXPECT(isApproximatelyEqual(evenOdd.coverage(column, row), expectedEvenOdd[row][column]));
        }
    }

    // Overlapping inside one pixel, where summing signed areas would give 1.5: [0,1]² and
    // [0.5,1.5] × [0,1]. Pixel 0 holds the whole first square and half the second.
    std::vector<Path2D> inPixel{rectanglePath(0, 0, 1, 1), rectanglePath(0.5f, 0, 1.5f, 1)};
    CoverageImage inPixelNonzero = exactCoverage(Shape2D{inPixel, FillRule::nonzero}, 2, 1, 0.01f);
    CoverageImage inPixelEvenOdd = exactCoverage(Shape2D{inPixel, FillRule::evenOdd}, 2, 1, 0.01f);
    EXPECT(isApproximatelyEqual(inPixelNonzero.coverage(0, 0), 1));
    EXPECT(isApproximatelyEqual(inPixelNonzero.coverage(1, 0), 0.5f));
    EXPECT(isApproximatelyEqual(inPixelEvenOdd.coverage(0, 0), 0.5f));
    EXPECT(isApproximatelyEqual(inPixelEvenOdd.coverage(1, 0), 0.5f));

    // The second square clockwise: winding 0 in the overlap, so nonzero leaves it empty too.
    std::vector<Path2D> opposite{rectanglePath(0, 0, 1, 1), rectanglePath(0.5f, 0, 1.5f, 1, false)};
    CoverageImage oppositeNonzero = exactCoverage(Shape2D{opposite, FillRule::nonzero}, 2, 1, 0.01f);
    EXPECT(isApproximatelyEqual(oppositeNonzero.coverage(0, 0), 0.5f));
    EXPECT(isApproximatelyEqual(oppositeNonzero.coverage(1, 0), 0.5f));
}

TEST(exactCoverageCircleConvergesToPiRSquared) {
    // Radius 10 round (12, 12) in a 24 × 24 grid. The total differs from the cubic circle's exact
    // area by at most tolerance × the curve's length (≤ its control polygons), and the cubic
    // circle (quarters with handle 0.5523) lies outside the true one by at most 2.73e-4·r, so its
    // area is within (1 + 2.73e-4)² − 1 ≈ 5.5e-4 of πr².
    float radius = 10;
    Shape2D circle{{circlePath(Vector2(12, 12), radius)}};
    float cubicArea = std::fabs(circle.signedArea());
    float circleArea = std::numbers::pi_v<float> * radius * radius;
    EXPECT(std::fabs(cubicArea - circleArea) <= 5.5e-4f * circleArea);
    // The chords cut inside the curve by up to the tolerance, so the error falls in step with it
    // (about ⅔ × tolerance × length: 0.36 at 0.01, 0.037 at 0.001).
    float previousError = INFINITY;
    float finestTotal = 0;
    for (float tolerance : {0.5f, 0.1f, 0.01f, 0.001f}) {
        float total = exactCoverage(circle, 24, 24, tolerance).totalCoverage();
        float flatteningError = std::fabs(total - cubicArea);
        EXPECT(flatteningError <= tolerance * curvedLengthBound(circle) + 1e-3f);
        EXPECT(flatteningError <= previousError);
        previousError = flatteningError;
        EXPECT(std::fabs(total - circleArea) <= tolerance * curvedLengthBound(circle) + 5.5e-4f * circleArea);
        finestTotal = total;
    }
    EXPECT(std::fabs(finestTotal - circleArea) <= 3e-4f * circleArea);
}

// MARK: Properties

TEST(exactCoverageStaysInUnitRange) {
    SeededRandom random(2026'10'28);
    for (int trial = 0; trial < 40; trial += 1) {
        FillRule fillRule = trial % 2 == 0 ? FillRule::nonzero : FillRule::evenOdd;
        Shape2D polygon = randomPolygon(random, 12, 16, fillRule);
        EXPECT(hasEveryCoverageInUnitRange(exactCoverage(polygon, 16, 16, 0.01f)));
        // Curved and overlapping: three circles of random size, some clockwise.
        std::vector<Path2D> circles;
        for (int index = 0; index < 3; index += 1) {
            Vector2 centre(random.nextFloat(4, 12), random.nextFloat(4, 12));
            circles.push_back(circlePath(centre, random.nextFloat(0.3f, 4), random.nextUnitFloat() < 0.5f));
        }
        EXPECT(hasEveryCoverageInUnitRange(exactCoverage(Shape2D{circles, fillRule}, 16, 16, random.nextFloat(0.001f, 0.3f))));
    }
}

TEST(exactCoverageTotalIsTheAreaWithinTheFlatteningBound) {
    // Tidy shapes (one counterclockwise outline, one clockwise hole inside it), where the filled
    // area is the signed area, which is exact for the cubics (Green's theorem).
    SeededRandom random(7);
    for (int trial = 0; trial < 20; trial += 1) {
        Vector2 centre(random.nextFloat(9, 11), random.nextFloat(9, 11));
        float outer = random.nextFloat(3, 8);
        float inner = random.nextFloat(0.2f, outer - 0.5f);
        Vector2 holeCentre = centre + Vector2(random.nextFloat(-0.2f, 0.2f), random.nextFloat(-0.2f, 0.2f));
        Shape2D ring{{circlePath(centre, outer), circlePath(holeCentre, inner, false)}};
        float tolerance = random.nextFloat(0.001f, 0.2f);
        float total = exactCoverage(ring, 20, 20, tolerance).totalCoverage();
        EXPECT(std::fabs(total - ring.signedArea()) <= tolerance * curvedLengthBound(ring) + 1e-3f);
    }
    // Polygons are exact: the triangle (0.3, 0.2), (7.9, 1.1), (2.6, 6.7) has area
    // ½|(7.6)(6.5) − (0.9)(2.3)| = ½(49.4 − 2.07) = 23.665.
    CoverageImage triangle = exactCoverage(Shape2D{{polygonPath({{0.3f, 0.2f}, {7.9f, 1.1f}, {2.6f, 6.7f}})}}, 8, 8, 0.01f);
    EXPECT(isApproximatelyEqual(triangle.totalCoverage(), 23.665f, 1e-4f));
}

TEST(exactCoverageAgreesWithSampledWindingNumbers) {
    // Self-crossing polygons under both rules. 32 × 32 samples per pixel miss at most the area
    // of the sub-squares an edge passes through, so they agree to within a few hundredths.
    SeededRandom random(11);
    for (int trial = 0; trial < 6; trial += 1) {
        FillRule fillRule = trial % 2 == 0 ? FillRule::nonzero : FillRule::evenOdd;
        Shape2D polygon = randomPolygon(random, 9, 8, fillRule);
        CoverageImage image = exactCoverage(polygon, 8, 8, 0.01f);
        float largestDifference = 0;
        for (int row = 0; row < 8; row += 1) {
            for (int column = 0; column < 8; column += 1) {
                float difference = std::fabs(image.coverage(column, row) - sampledCoverage(polygon, column, row, 32));
                largestDifference = std::max(largestDifference, difference);
            }
        }
        EXPECT(largestDifference <= 0.15f);
    }
}

TEST(exactCoverageMovesWithAWholePixelShift) {
    Shape2D shape{{polygonPath({{0.3f, 0.2f}, {3.9f, 1.1f}, {1.6f, 3.7f}}), circlePath(Vector2(2, 2), 1.3f)},
                  FillRule::evenOdd};
    Shape2D shifted{{polygonPath({{2.3f, 1.2f}, {5.9f, 2.1f}, {3.6f, 4.7f}}), circlePath(Vector2(4, 3), 1.3f)},
                    FillRule::evenOdd};
    CoverageImage image = exactCoverage(shape, 8, 8, 0.01f);
    CoverageImage moved = exactCoverage(shifted, 8, 8, 0.01f);
    for (int row = 0; row + 1 < 8; row += 1) {
        for (int column = 0; column + 2 < 8; column += 1) {
            EXPECT(isApproximatelyEqual(image.coverage(column, row), moved.coverage(column + 2, row + 1), 1e-4f));
        }
    }
}

TEST(exactCoverageRefusesBadInput) {
    Shape2D square{{rectanglePath(0, 0, 1, 1)}};
    EXPECT_PROBLEM(exactCoverage(square, 2, 2, 0), "flattening tolerance must be positive");
    EXPECT_PROBLEM(exactCoverage(square, -1, 2, 0.1f), "0…16384 pixels");
    EXPECT_PROBLEM(exactCoverage(square, 2, 2, 0.1f).coverage(2, 0), "outside a 2 × 2 coverage image");
    // A shape far outside the grid covers nothing.
    EXPECT(exactCoverage(Shape2D{{rectanglePath(-1e9f, 0, -1e9f + 1e3f, 1)}}, 2, 1, 0.1f).totalCoverage() == 0);
}

// MARK: Picture

TEST(exactCoverageGlyphPicture) {
    // What to check: a "Q" whose ring is smooth and whose counter is empty; on the left
    // (nonzero) the tail is solid where it crosses the ring, on the right (even-odd) that
    // crossing is cut out.
    CoverageImage nonzero = exactCoverage(glyphQ(FillRule::nonzero), 64, 64, 0.01f);
    CoverageImage evenOdd = exactCoverage(glyphQ(FillRule::evenOdd), 64, 64, 0.01f);
    writeSideBySide({nonzero, evenOdd}, 6,
                    std::filesystem::path(MATH_CPP_FOLDER) / "build" / "visuals" / "math-curves" / "exact-coverage-glyph.pgm");
    // The counter's centre is empty under both rules.
    EXPECT(nonzero.coverage(30, 36) == 0 && evenOdd.coverage(30, 36) == 0);
    // A pixel wholly inside both the ring and the tail: winding 2 at its four corners.
    Shape2D shape = glyphQ(FillRule::nonzero);
    bool foundOverlap = false;
    for (int row = 0; row < 64 && !foundOverlap; row += 1) {
        for (int column = 0; column < 64 && !foundOverlap; column += 1) {
            bool isWound2 = true;
            for (Vector2 corner : {Vector2(0.01f, 0.01f), Vector2(0.99f, 0.01f), Vector2(0.01f, 0.99f), Vector2(0.99f, 0.99f)}) {
                if (shape.windingNumber(Vector2(column, row) + corner) != 2) isWound2 = false;
            }
            if (!isWound2) continue;
            foundOverlap = true;
            EXPECT(isApproximatelyEqual(nonzero.coverage(column, row), 1));
            EXPECT(isApproximatelyEqual(evenOdd.coverage(column, row), 0));
        }
    }
    EXPECT(foundOverlap);
    // Even-odd drops the overlap that nonzero fills, so it holds less ink.
    EXPECT(nonzero.totalCoverage() > evenOdd.totalCoverage());
}
