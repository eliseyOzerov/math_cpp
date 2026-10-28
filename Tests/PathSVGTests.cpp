// Tests of Curves/PathSVG: reading and writing SVG path data. The golden cases are from graphics_cpp's
// graphics_cpp tests/golden/svg.json; the strings it writes are not compared (JS number printing differs), so
// written text is tested by reading it back.
#include <string>
#include <vector>

#include "../Curves/PathSVG.h"
#include "../Curves/Shapes2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

using Cubic = CubicBezier<Vector2>;

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// The nodes a path passes (each piece's end, after its start), from its pieces including a closing line.
std::vector<Vector2> endsOf(const Path2D& path) {
    std::vector<Vector2> ends;
    for (const Cubic& piece : Shapes2D::cubicsOf(path)) ends.push_back(piece.end);
    return ends;
}

} // namespace

TEST(parsedReadsLinesAbsoluteAndRelative) {
    // Golden: a triangle, with the closing line implied by Z.
    std::vector<Path2D> triangle = PathSVG::parsed("M0 0L10 0L10 10Z");
    EXPECT_EQUAL(triangle.size(), size_t(1));
    EXPECT(triangle[0].isClosed());
    EXPECT_EQUAL(triangle[0].segments().size(), size_t(2));
    std::vector<Vector2> ends = endsOf(triangle[0]);
    EXPECT_EQUAL(ends.size(), size_t(3));
    EXPECT(isClose(ends[2], Vector2(0, 0)));
    // Golden: relative commands; "m10 10 l5 0 0 5 -5 0z" is a 5 square at (10,10); further pairs after l are lines.
    std::vector<Path2D> square = PathSVG::parsed("m10 10 l5 0 0 5 -5 0z");
    EXPECT(isClose(square[0].start, Vector2(10, 10)));
    EXPECT(isClose(endsOf(square[0])[1], Vector2(15, 15)) && isClose(endsOf(square[0])[2], Vector2(10, 15)));
    EXPECT(isApproximatelyEqual(std::fabs(square[0].signedArea()), 25.0f, 1e-3f));
    // H and V, absolute and relative.
    std::vector<Path2D> hv = PathSVG::parsed("M1 2 H5 V6 h-2 v-1");
    EXPECT(isClose(endsOf(hv[0])[0], Vector2(5, 2)) && isClose(endsOf(hv[0])[1], Vector2(5, 6)));
    EXPECT(isClose(endsOf(hv[0])[2], Vector2(3, 6)) && isClose(endsOf(hv[0])[3], Vector2(3, 5)));
}

TEST(parsedReadsCubicsAndReflectsSmoothHandles) {
    // Golden: C then S: S's first handle is the last handle reflected about the current point (10,0):
    // 2·(10,0) - (10,10) = (10,-10).
    std::vector<Path2D> path = PathSVG::parsed("M0 0 C0 10 10 10 10 0 S20 -10 20 0");
    std::vector<Cubic> cubics = Shapes2D::cubicsOf(path[0]);
    EXPECT_EQUAL(cubics.size(), size_t(3));
    EXPECT(isClose(cubics[0].startHandle, Vector2(0, 10)) && isClose(cubics[0].endHandle, Vector2(10, 10)));
    EXPECT(isClose(cubics[1].startHandle, Vector2(10, -10)) && isClose(cubics[1].endHandle, Vector2(20, -10)));
    EXPECT(isClose(cubics[1].end, Vector2(20, 0)));
    // Without a cubic before it, S uses the current point as its first handle.
    Cubic lone = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 L5 5 S9 9 10 10")[0])[1];
    EXPECT(isClose(lone.startHandle, Vector2(5, 5)));
}

TEST(parsedRaisesQuadraticsExactly) {
    // Golden: Q5 10 10 0 from (0,0): handles two thirds of the way to the control: (3.33,6.67), (6.67,6.67).
    // T reflects the control about (10,0): (5,10) -> (15,-10), handles (13.33,-6.67), (16.67,-6.67).
    std::vector<Cubic> cubics = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 Q5 10 10 0 T20 0")[0]);
    EXPECT(isClose(cubics[0].startHandle, Vector2(3.3333f, 6.6667f)) && isClose(cubics[0].endHandle, Vector2(6.6667f, 6.6667f)));
    EXPECT(isClose(cubics[1].startHandle, Vector2(13.3333f, -6.6667f)) && isClose(cubics[1].endHandle, Vector2(16.6667f, -6.6667f)));
    // Property: the raised cubic traces the quadratic: its middle is (start + 2·control + end)/4 = (5, 5).
    EXPECT(isClose(cubics[0].point(0.5f), Vector2(5, 5)));
    // T after a line has no control to reflect, so it is a straight run.
    Cubic straight = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 L10 0 T30 0")[0])[1];
    EXPECT(isClose(straight.startHandle, Vector2(10, 0)) && isClose(straight.endHandle, Vector2(16.6667f, 0)));
}

TEST(parsedCutsArcsIntoQuarterTurnsOnTheCircle) {
    // Golden: A10 10 0 0 1 20 0 is a half circle about (10,0), in two quarters; the sweep flag 1 turns
    // clockwise in the plane as written, so it passes (10,-10).
    std::vector<Cubic> half = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 A10 10 0 0 1 20 0")[0]);
    EXPECT_EQUAL(half.size(), size_t(3)); // two quarters and the closing line
    EXPECT(isClose(half[0].end, Vector2(10, -10), 2e-3f));
    EXPECT(isClose(half[0].startHandle, Vector2(0, -5.5228f), 2e-3f));
    EXPECT(isClose(half[1].end, Vector2(20, 0), 2e-3f));
    // Property: every point of every piece lies on the circle about (10,0) with radius 10.
    for (size_t index = 0; index < 2; index++)
        for (float parameter : {0.0f, 0.3f, 0.6f, 1.0f})
            EXPECT(isApproximatelyEqual(half[index].point(parameter).distance(Vector2(10, 0)), 10.0f, 0.03f));
    // A rotated, stretched arc (golden A10 5 30 1 0 20 10) still ends where it was told.
    std::vector<Cubic> tilted = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 A10 5 30 1 0 20 10")[0]);
    EXPECT(isClose(tilted[0].end, Vector2(8.3702f, 10.5024f), 5e-3f));
    
    // Radii too small to reach are grown: a radius 1 arc to 20 away is a half circle of radius 10.
    std::vector<Cubic> grown = Shapes2D::cubicsOf(PathSVG::parsed("M0 0 A1 1 0 0 1 20 0")[0]);
    EXPECT(isClose(grown[0].end, Vector2(10, -10), 5e-3f));
    // A zero radius is a straight line; equal end points make no piece (golden: the whole circle needs two arcs).
    EXPECT(PathSVG::parsed("M0 0 A0 5 0 0 1 10 0 L10 10")[0].segments()[0].isLine());
    EXPECT(PathSVG::parsed("M0 0 A10 10 0 1 1 0 0").empty());
    EXPECT_EQUAL(PathSVG::parsed("M0 0 A10 10 0 0 1 10 0 A10 10 0 0 1 0 0")[0].segments().size(), size_t(2));
}

TEST(parsedReadsNumbersTheWaySvgWritesThem) {
    // Golden: "1e1", "2E-1", ".5.5" (two numbers), "+3." and flags without separators.
    std::vector<Vector2> ends = endsOf(PathSVG::parsed("M0 0 L1e1 2E-1 L-.5.5 L+3. 4")[0]);
    EXPECT(isClose(ends[0], Vector2(10, 0.2f)) && isClose(ends[1], Vector2(-0.5f, 0.5f)) && isClose(ends[2], Vector2(3, 4)));
    std::vector<Cubic> arcs = Shapes2D::cubicsOf(PathSVG::parsed("M0 0a10 10 0 0120 0")[0]);
    EXPECT(isClose(arcs[0].end, Vector2(10, -10), 2e-3f));
    // Commas and every ASCII space separate numbers.
    EXPECT_EQUAL(PathSVG::parsed("M0,0\tL10,\n0\r\nL10 10").size(), size_t(1));
}

TEST(parsedSplitsSubpathsAndClosesOpenOnes) {
    // Golden: two subpaths, each closed; a moveto alone leaves nothing; a second M replaces an unused first.
    std::vector<Path2D> two = PathSVG::parsed("M10 10 L20 20 M30 30 L40 40 L30 40 Z");
    EXPECT_EQUAL(two.size(), size_t(2));
    EXPECT(two[0].isClosed() && two[1].isClosed());
    EXPECT(isClose(two[1].start, Vector2(30, 30)));
    EXPECT(PathSVG::parsed("M0 0 Z").empty());
    EXPECT(PathSVG::parsed("").empty());
    std::vector<Path2D> replaced = PathSVG::parsed("M0 0 M5 5 L9 9");
    EXPECT_EQUAL(replaced.size(), size_t(1));
    EXPECT(isClose(replaced[0].start, Vector2(5, 5)));
    // After Z the pen is back at the subpath's start: a following L draws from there.
    std::vector<Path2D> after = PathSVG::parsed("M0 0 L10 0 L10 10 Z L5 5 L0 5");
    EXPECT_EQUAL(after.size(), size_t(2));
    EXPECT(isClose(after[1].start, Vector2(0, 0)));
}

TEST(parsedThrowsOnMalformedData) {
    EXPECT_PROBLEM(PathSVG::parsed("M0 0 L10"), "a number expected");
    EXPECT_PROBLEM(PathSVG::parsed("M0 0 X5 5"), "unknown command X");
    EXPECT_PROBLEM(PathSVG::parsed("10 20"), "a command expected");
    EXPECT_PROBLEM(PathSVG::parsed("M0 0 A5 5 0 2 0 10 10"), "a flag expected");
    EXPECT_PROBLEM(PathSVG::parsed("M0 0 Z 5 5"), "a command expected");
    EXPECT_PROBLEM(PathSVG::parsed("M a"), "a number expected");
}

TEST(textWritesFixedDecimalsWithoutTrailingZeros) {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0.5f));
    path.addCubic(Vector2(10, 5), Vector2(8.126f, 10), Vector2(-0.004f, 10));
    path.close();
    EXPECT_EQUAL(PathSVG::text({path}, 2), std::string("M0 0L10 0.5C10 5 8.13 10 0 10Z"));
    EXPECT_EQUAL(PathSVG::text({path}, 0), std::string("M0 0L10 1C10 5 8 10 0 10Z")); // 0.5 rounds away from zero
    // An open path has no Z; a path without a piece is skipped; no paths give no text.
    Path2D open(Vector2(1, 2));
    open.addLine(Vector2(3, 4));
    EXPECT_EQUAL(PathSVG::text({Path2D(Vector2(9, 9)), open}), std::string("M1 2L3 4"));
    EXPECT_EQUAL(PathSVG::text({}), std::string(""));
    EXPECT_PROBLEM(PathSVG::text({open}, 12), "digits");
    Path2D broken(Vector2(0, 0));
    broken.addLine(Vector2(std::nanf(""), 0));
    EXPECT_PROBLEM(PathSVG::text({broken}), "not finite");
}

TEST(textReadsBackAsTheSamePaths) {
    // Round trip: write the paths, read them, and the pieces are the same within the digits written.
    Path2D oval = Shapes2D::oval(0, 0, 100, 200);
    Path2D triangle = PathSVG::parsed("M0 0L10 0L10 10Z")[0];
    std::vector<Path2D> back = PathSVG::parsed(PathSVG::text({oval, triangle}, 4));
    EXPECT_EQUAL(back.size(), size_t(2));
    std::vector<Cubic> original = Shapes2D::cubicsOf(oval), reread = Shapes2D::cubicsOf(back[0]);
    EXPECT_EQUAL(reread.size(), original.size());
    for (size_t index = 0; index < original.size() && index < reread.size(); index++) {
        EXPECT(isClose(original[index].startHandle, reread[index].startHandle, 1e-3f));
        EXPECT(isClose(original[index].endHandle, reread[index].endHandle, 1e-3f));
        EXPECT(isClose(original[index].end, reread[index].end, 1e-3f));
    }
    EXPECT_EQUAL(back[1].segments().size(), size_t(2));
    EXPECT(back[1].isClosed() && back[1].segments()[0].isLine());
}
