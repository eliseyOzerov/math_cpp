// Tests of Curves/Morph2D: making two paths alike, and blending them. Values are worked out by hand in
// the comments, or taken from graphics_cpp's golden file Curves2D/tests/golden/morph.json.
#include <vector>

#include "../Curves/Morph2D.h"
#include "../Curves/Shapes2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-3f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

Path2D polyline(const std::vector<Vector2>& points, bool closed = false) {
    Path2D path(points.front());
    for (size_t index = 1; index < points.size(); index++) path.addLine(points[index]);
    if (closed) path.close();
    return path;
}

/// The node positions (starts, then the last end) of a path's pieces.
std::vector<Vector2> nodesOf(const Path2D& path) {
    
    std::vector<Vector2> nodes;
    for (const auto& piece : Shapes2D::cubicsOf(path)) nodes.push_back(piece.start);
    nodes.push_back(Shapes2D::cubicsOf(path).back().end);
    return nodes;
}

} // namespace

TEST(alikeAddsNodesToTheShorterPathWithoutChangingItsShape) {
    // A line (0,0)-(10,0) against a tent (0,0)-(5,5)-(10,0): the tent's middle node is halfway along its
    // length (both sides are 7.07), so it is added halfway along the line, at (5,0).
    Path2D line = polyline({Vector2(0, 0), Vector2(10, 0)});
    Path2D tent = polyline({Vector2(0, 0), Vector2(5, 5), Vector2(10, 0)});
    MorphCorrespondence alike = Morph2D::alike(line, tent);
    EXPECT_EQUAL(alike.from.segments().size(), size_t(2));
    EXPECT_EQUAL(alike.to.segments().size(), size_t(2));
    EXPECT(alike.matched == (std::vector<bool>{true, false, true}));
    std::vector<Vector2> nodes = nodesOf(alike.from);
    EXPECT(isClose(nodes[0], Vector2(0, 0)) && isClose(nodes[1], Vector2(5, 0)) && isClose(nodes[2], Vector2(10, 0)));
    // Halfway between, the middle node is at (5, 2.5) and the ends stay.
    std::vector<Vector2> halfway = nodesOf(Morph2D::between(alike.from, alike.to, 0.5f));
    EXPECT(isClose(halfway[1], Vector2(5, 2.5f)) && isClose(halfway[0], Vector2(0, 0)) && isClose(halfway[2], Vector2(10, 0)));
}

TEST(alikeMergesTheExtraNodesOfAPathWithTooMany) {
    // Three nodes on a line against two on y = 10: the extra node (5,0) is equally far from both matches,
    // so the earlier wins and piece 0 shrinks to nothing on the target's first node.
    Path2D three = polyline({Vector2(0, 0), Vector2(5, 0), Vector2(10, 0)});
    Path2D two = polyline({Vector2(0, 10), Vector2(10, 10)});
    MorphCorrespondence alike = Morph2D::alike(three, two);
    EXPECT(alike.matched == (std::vector<bool>{true, false, true}));
    std::vector<Vector2> nodes = nodesOf(alike.to);
    EXPECT(isClose(nodes[0], Vector2(0, 10)) && isClose(nodes[1], Vector2(0, 10)) && isClose(nodes[2], Vector2(10, 10)));
    // At 1 the merged piece is dropped and the morph is the target; at 0 it is the original, three nodes.
    Path2D full = Morph2D::morph(three, two, 1);
    EXPECT_EQUAL(full.segments().size(), size_t(1));
    EXPECT(isClose(full.start, Vector2(0, 10)) && isClose(full.currentPoint(), Vector2(10, 10)));
    EXPECT_EQUAL(Morph2D::morph(three, two, 0).segments().size(), size_t(2));
}

TEST(alikeMatchesTheGoldenZigzagToTwoPieces) {
    // Golden alike case 100: a five piece zigzag against a degenerate piece then a line to (100,0).
    Path2D zigzag(Vector2(0, 0));
    zigzag.addCubic(Vector2(6.667f, 13.333f), Vector2(13.333f, 26.667f), Vector2(20, 40));
    zigzag.addCubic(Vector2(26.667f, 26.667f), Vector2(33.333f, 13.333f), Vector2(40, 0));
    zigzag.addCubic(Vector2(46.667f, 13.333f), Vector2(53.333f, 26.667f), Vector2(60, 40));
    zigzag.addCubic(Vector2(66.667f, 26.667f), Vector2(73.333f, 13.333f), Vector2(80, 0));
    zigzag.addCubic(Vector2(86.667f, 13.333f), Vector2(93.333f, 26.667f), Vector2(100, 40));
    Path2D target(Vector2(0, 0));
    target.addCubic(Vector2(0, 0), Vector2(0, 0), Vector2(0, 0));
    target.addCubic(Vector2(33.333f, 0), Vector2(66.667f, 0), Vector2(100, 0));
    MorphCorrespondence alike = Morph2D::alike(zigzag, target);
    EXPECT(alike.matched == (std::vector<bool>{true, false, true, false, false, true}));
    EXPECT_EQUAL(alike.to.segments().size(), size_t(5));
    EXPECT(isClose(nodesOf(alike.to)[3], Vector2(0, 0)));
    EXPECT(isClose(nodesOf(alike.to)[4], Vector2(100, 0)));
    EXPECT(isClose(nodesOf(alike.to)[5], Vector2(100, 0)));
}

TEST(morphingAPathIntoItselfChangesNothing) {
    Path2D oval = Shapes2D::oval(0, 0, 100, 200);
    for (float amount : {0.0f, 0.25f, 0.5f, 1.0f}) {
        Path2D same = Morph2D::morph(oval, oval, amount);
        EXPECT(same.isClosed());
        std::vector<CubicBezier<Vector2>> cubics = Shapes2D::cubicsOf(same), expected = Shapes2D::cubicsOf(oval);
        EXPECT_EQUAL(cubics.size(), expected.size());
        for (size_t index = 0; index < cubics.size() && index < expected.size(); index++)
            EXPECT(isClose(cubics[index].startHandle, expected[index].startHandle, 1e-3f) &&
                   isClose(cubics[index].end, expected[index].end, 1e-3f));
    }
}

TEST(morphFindsTheBestStartAndDirectionOfAClosedTarget) {
    // The same square written from another corner and the other way round matches node for node, so
    // halfway between a square and itself that way is the square.
    Path2D square = polyline({Vector2(0, 0), Vector2(10, 0), Vector2(10, 10), Vector2(0, 10)}, true);
    Path2D reversedElsewhere = polyline({Vector2(10, 10), Vector2(10, 0), Vector2(0, 0), Vector2(0, 10)}, true);
    MorphCorrespondence alike = Morph2D::alike(square, reversedElsewhere);
    EXPECT(isClose(alike.to.start, Vector2(0, 0)));
    EXPECT(alike.matched == (std::vector<bool>{true, true, true, true, true}));
    Path2D half = Morph2D::morph(square, reversedElsewhere, 0.5f);
    EXPECT(isApproximatelyEqual(half.signedArea(), 100.0f, 1e-2f));
}

TEST(morphMovesEveryPointTheSameShareOfTheWay) {
    // Golden morph case 3 at 0.25: an oval into a 100 by 190 box-like oval starting elsewhere; first piece's
    // start is 25% of the way from (50,0) to (60,0).
    Path2D oval = Shapes2D::oval(0, 0, 100, 200);
    Path2D other(Vector2(110, 95));
    other.addCubic(Vector2(110, 166.25f), Vector2(97.5f, 190), Vector2(60, 190));
    other.addCubic(Vector2(22.5f, 190), Vector2(10, 166.25f), Vector2(10, 95));
    other.addCubic(Vector2(10, 23.75f), Vector2(22.5f, 0), Vector2(60, 0));
    other.addCubic(Vector2(97.5f, 0), Vector2(110, 23.75f), Vector2(110, 95));
    other.close();
    Path2D quarter = Morph2D::morph(oval, other, 0.25f);
    std::vector<CubicBezier<Vector2>> cubics = Shapes2D::cubicsOf(quarter);
    EXPECT_EQUAL(cubics.size(), size_t(4));
    EXPECT(isClose(cubics[0].start, Vector2(52.5f, 0), 5e-3f));
    EXPECT(isClose(cubics[0].startHandle, Vector2(82.586f, 0), 5e-3f));
    EXPECT(isClose(cubics[0].endHandle, Vector2(102.5f, 39.515f), 5e-3f));
    EXPECT(isClose(cubics[0].end, Vector2(102.5f, 98.75f), 5e-3f));
}

TEST(betweenBlendsControlPointsAndDropsShrunkPieces) {
    Path2D from = polyline({Vector2(0, 0), Vector2(10, 0)});
    Path2D to = polyline({Vector2(0, 10), Vector2(10, 30)});
    Path2D third = Morph2D::between(from, to, 0.5f);
    EXPECT(isClose(third.start, Vector2(0, 5)) && isClose(third.currentPoint(), Vector2(10, 15)));
    // A piece that has shrunk to a point (both of its ends within the tolerance) is dropped.
    Path2D collapsing = polyline({Vector2(0, 0), Vector2(0, 0), Vector2(10, 0)});
    Path2D settled = polyline({Vector2(0, 0), Vector2(0, 0), Vector2(10, 0)});
    EXPECT_EQUAL(Morph2D::between(collapsing, settled, 0.5f).segments().size(), size_t(1));
    EXPECT_PROBLEM(Morph2D::between(from, collapsing, 0.5f), "same number of pieces");
}

TEST(morphRefusesWhatCannotMatch) {
    Path2D open = polyline({Vector2(0, 0), Vector2(10, 0)});
    Path2D closed = polyline({Vector2(0, 0), Vector2(10, 0), Vector2(10, 10)}, true);
    EXPECT_PROBLEM(Morph2D::alike(open, closed), "closed");
    EXPECT_PROBLEM(Morph2D::alike(Path2D(Vector2(0, 0)), open), "needs a piece");
}
