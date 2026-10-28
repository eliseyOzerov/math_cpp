// Tests of Curves/MeasuredPath2D and of the Path2D additions of slice S4: quarters, arches, joins,
// node kinds, reversed, startingAtNode and mapped. Values are worked out by hand in the comments.
#include <cmath>
#include <vector>

#include "../Curves/MeasuredPath2D.h"
#include "../Linear/AffineTransform2.h"
#include "../Curves/Path2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-4f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

/// An L: along x to (10, 0), then up to (10, 20). Lengths 10 and 20, total 30.
Path2D lShape() {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addLine(Vector2(10, 20));
    return path;
}

/// The square (0,0)…(100,100), counterclockwise, closed by its implicit closing line.
Path2D square() {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(100, 0));
    path.addLine(Vector2(100, 100));
    path.addLine(Vector2(0, 100));
    path.close();
    return path;
}

} // namespace

// MARK: Quarters, arches, joins

TEST(quarterLeavingVerticallyHasItsHandlesOnTheAxes) {
    // From (0,0) to (10,10) with tension 0.5: startHandle = (0, 5), endHandle = (10 + 0.5·(0 − 10), 10) = (5, 10).
    Path2D path(Vector2(0, 0));
    path.addQuarter(Vector2(10, 10), Heading::vertical, 0.5f);
    EXPECT(path.segments().size() == 1);
    EXPECT(isClose(path.segments()[0].startHandle, Vector2(0, 5)));
    EXPECT(isClose(path.segments()[0].endHandle, Vector2(5, 10)));
    // Leaving horizontally swaps the roles: (5, 0) and (10, 5).
    Path2D sideways(Vector2(0, 0));
    sideways.addQuarter(Vector2(10, 10), Heading::horizontal, 0.5f);
    EXPECT(isClose(sideways.segments()[0].startHandle, Vector2(5, 0)));
    EXPECT(isClose(sideways.segments()[0].endHandle, Vector2(10, 5)));
}

TEST(quarterWithTheCircleTensionFollowsACircle) {
    // (0,0) → (1,1) leaving upward is the quarter of the unit circle about (1, 0); its middle is at
    // 45°: (1 − cos 45°, sin 45°) = (0.29289, 0.70711).
    Path2D path(Vector2(0, 0));
    path.addQuarter(Vector2(1, 1));
    CubicBezier<Vector2> cubic(Vector2(0, 0), path.segments()[0].startHandle, path.segments()[0].endHandle, Vector2(1, 1));
    EXPECT(isClose(cubic.point(0.5f), Vector2(0.29289f, 0.70711f), 5e-4f));
    EXPECT(isApproximatelyEqual(Path2D::circleQuarterTension(), 0.5522847f, 1e-6f));
}

TEST(quartersCarryOnTheWayTheLastOneArrived) {
    // The first quarter leaves vertically and so arrives horizontally; the next one, with no heading
    // given, leaves horizontally: its start handle lies level with its start.
    Path2D path(Vector2(0, 0));
    path.addQuarter(Vector2(10, 10), Heading::vertical, 0.5f);
    EXPECT(path.arrivalHeading() == Heading::horizontal);
    path.addQuarter(Vector2(20, 0), std::nullopt, 0.5f);
    EXPECT(isClose(path.segments()[1].startHandle, Vector2(15, 10)));
    EXPECT(isClose(path.segments()[1].endHandle, Vector2(20, 5)));
    // After a line there is nothing to carry on: the default is vertical.
    Path2D afterLine(Vector2(0, 0));
    afterLine.addLine(Vector2(5, 5));
    EXPECT(!afterLine.arrivalHeading().has_value());
    afterLine.addQuarter(Vector2(10, 10), std::nullopt, 0.5f);
    EXPECT(isClose(afterLine.segments()[1].startHandle, Vector2(5, 7.5f)));
}

TEST(archRisesToThePeakHalfwayAndMirrors) {
    // (0,0) → (10,0) over a peak at y = 5: two quarters meeting at (5, 5), the first leaving upward.
    Path2D arch(Vector2(0, 0));
    arch.addArch(Vector2(10, 0), 5, 0.5f);
    EXPECT(arch.segments().size() == 2);
    EXPECT(isClose(arch.segments()[0].end(), Vector2(5, 5)));
    EXPECT(isClose(arch.segments()[1].end(), Vector2(10, 0)));
    EXPECT(isClose(arch.segments()[0].startHandle, Vector2(0, 2.5f)));
    // Property: mirrored about x = 5 the two halves swap, so the second half's handles are the first's mirrored.
    EXPECT(isClose(arch.segments()[1].startHandle, Vector2(7.5f, 5)));
    EXPECT(isClose(arch.segments()[1].endHandle, Vector2(10, 2.5f)));
}

TEST(joinedAddsALinkOnlyWhenThePathsDoNotMeet) {
    Path2D first(Vector2(0, 0));
    first.addLine(Vector2(10, 0));
    Path2D meeting(Vector2(10, 0));
    meeting.addLine(Vector2(10, 10));
    Path2D together = first.joined(meeting);
    EXPECT(together.segments().size() == 2);
    EXPECT(together.currentPoint() == Vector2(10, 10));
    Path2D apart(Vector2(20, 20));
    apart.addLine(Vector2(30, 20));
    Path2D linked = first.joined(apart);
    EXPECT(linked.segments().size() == 3);
    EXPECT(linked.segments()[1].end() == Vector2(20, 20));
    EXPECT_PROBLEM(Path2D(first).joined(square()), "open paths");
    // Marks follow their nodes, and the meeting node is cleared.
    Path2D marked = meeting;
    marked.setNodeKind(1, NodeKind::smooth);
    Path2D markedFirst = first;
    markedFirst.setNodeKind(1, NodeKind::symmetric);
    Path2D both = markedFirst.joined(marked);
    EXPECT(!both.nodeKind(1).has_value());
    EXPECT(both.nodeKind(2) == NodeKind::smooth);
}

// MARK: Node kinds

TEST(nodeKindsAreReadFromTheHandles) {
    // Node 1 at (10, 10): both handles 5 long along x, on opposite sides: symmetric.
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    path.addCubic(Vector2(15, 10), Vector2(20, 5), Vector2(20, 0));
    EXPECT(path.effectiveNodeKind(0) == NodeKind::corner);
    EXPECT(path.effectiveNodeKind(2) == NodeKind::corner);
    EXPECT(path.effectiveNodeKind(1) == NodeKind::symmetric);
    // Lengthening the leaving handle to 10 keeps it smooth but not symmetric.
    Path2D longer(Vector2(0, 0));
    longer.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    longer.addCubic(Vector2(20, 10), Vector2(20, 5), Vector2(20, 0));
    EXPECT(longer.effectiveNodeKind(1) == NodeKind::smooth);
    // A leaving handle that turns away is a corner.
    Path2D bent(Vector2(0, 0));
    bent.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    bent.addCubic(Vector2(10, 15), Vector2(20, 5), Vector2(20, 0));
    EXPECT(bent.effectiveNodeKind(1) == NodeKind::corner);
    // A mark wins over the handles.
    bent.setNodeKind(1, NodeKind::smooth);
    EXPECT(bent.effectiveNodeKind(1) == NodeKind::smooth);
    EXPECT_PROBLEM(bent.effectiveNodeKind(7), "node");
}

TEST(aLineBesideACubicCountsAsACubicWithHandlesAThirdAlong) {
    // The line (0,0)→(10,0) has its out handle at (6.67, 0): 3.33 from the node, the cubic's is 10 long.
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addCubic(Vector2(20, 0), Vector2(30, 10), Vector2(30, 20));
    EXPECT(path.effectiveNodeKind(1) == NodeKind::smooth);
}

TEST(movingASymmetricHandleTurnsItsPartnerTheSameLength) {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    path.addCubic(Vector2(15, 10), Vector2(20, 5), Vector2(20, 0));
    // Outgoing handle to (13, 14): offset (3, 4), length 5, direction (0.6, 0.8). The incoming handle
    // takes the opposite direction and the same length: (10 − 3, 10 − 4) = (7, 6).
    Path2D moved = path.withHandleMoved(1, HandleSide::outgoing, Vector2(13, 14));
    EXPECT(isClose(moved.segments()[1].startHandle, Vector2(13, 14)));
    EXPECT(isClose(moved.segments()[0].endHandle, Vector2(7, 6)));
    EXPECT(moved.nodeKind(1) == NodeKind::symmetric);
    // Property: the node and the other handles stay put, and the node is still symmetric.
    EXPECT(moved.segments()[0].end() == Vector2(10, 10) && isClose(moved.segments()[1].endHandle, Vector2(20, 5)));
    EXPECT(moved.effectiveNodeKind(1) == NodeKind::symmetric);
    // The ends have only one handle.
    EXPECT_PROBLEM(path.withHandleMoved(0, HandleSide::incoming, Vector2(1, 1)), "no handle");
}

TEST(aCornerNodeLeavesItsPartnerWhereItIs) {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    path.addCubic(Vector2(10, 15), Vector2(20, 5), Vector2(20, 0));
    Path2D moved = path.withHandleMoved(1, HandleSide::outgoing, Vector2(14, 14));
    EXPECT(isClose(moved.segments()[0].endHandle, Vector2(5, 10)));
    EXPECT(isClose(moved.segments()[1].startHandle, Vector2(14, 14)));
}

TEST(withNodeKindLinesUpHandlesToSmoothAndSymmetric) {
    // A corner at (10, 10): incoming handle offset (5, 0), outgoing (0, 5). Smooth along the average of
    // the two unit directions (the incoming handle's direction is +x, the outgoing +y): (1,1)/√2, keeping
    // each handle's length 5: incoming handle at the node minus 5·(0.7071, 0.7071) = (6.4645, 6.4645),
    // outgoing at the node plus the same.
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 5), Vector2(5, 10), Vector2(10, 10));
    path.addCubic(Vector2(10, 15), Vector2(20, 5), Vector2(20, 0));
    Path2D smooth = path.withNodeKind(NodeKind::smooth, std::vector<int>{1});
    EXPECT(isClose(smooth.segments()[0].endHandle, Vector2(6.46447f, 6.46447f), 1e-3f));
    EXPECT(isClose(smooth.segments()[1].startHandle, Vector2(13.53553f, 13.53553f), 1e-3f));
    EXPECT(smooth.nodeKind(1) == NodeKind::smooth);
    EXPECT(smooth.nodeKind(0) == std::nullopt);
    EXPECT_PROBLEM(path.withNodeKind(NodeKind::smooth, std::vector<int>{9}), "node");
    // Marking every node leaves the ends' handles alone.
    Path2D all = path.withNodeKind(NodeKind::symmetric);
    EXPECT(all.nodeKind(0) == NodeKind::symmetric && all.nodeKind(2) == NodeKind::symmetric);
    EXPECT(isClose(all.segments()[0].startHandle, Vector2(0, 5)));
}

// MARK: Reversed, startingAtNode, mapped

TEST(reversedRunsBackwardsAndTwiceGivesTheOriginal) {
    Path2D curve(Vector2(0, 0));
    curve.addLine(Vector2(10, 0));
    curve.addCubic(Vector2(15, 0), Vector2(20, 5), Vector2(20, 10));
    Path2D backwards = curve.reversed();
    EXPECT(backwards.start == Vector2(20, 10));
    EXPECT(backwards.segments()[0].startHandle == Vector2(20, 5) && backwards.segments()[0].endHandle == Vector2(15, 0));
    EXPECT(backwards.segments()[0].end() == Vector2(10, 0));
    EXPECT(backwards.segments()[1].isLine() && backwards.currentPoint() == Vector2(0, 0));
    EXPECT(backwards.reversed() == curve);
    // The signed area of a closed path changes sign, and the closing line becomes the first piece.
    Path2D triangle(Vector2(0, 0));
    triangle.addLine(Vector2(10, 0));
    triangle.addLine(Vector2(10, 10));
    triangle.close();
    EXPECT(isApproximatelyEqual(triangle.signedArea(), 50));
    EXPECT(isApproximatelyEqual(triangle.reversed().signedArea(), -50));
    EXPECT(triangle.reversed().segments().size() == 3 && triangle.reversed().start == Vector2(0, 0));
    EXPECT(triangle.reversed().segments()[0].end() == Vector2(10, 10));
}

TEST(reversedKeepsNodeMarksOnTheirNodes) {
    Path2D path(Vector2(0, 0));
    path.addLine(Vector2(10, 0));
    path.addLine(Vector2(10, 10));
    path.addLine(Vector2(0, 10));
    path.setNodeKind(1, NodeKind::smooth);
    Path2D backwards = path.reversed();
    EXPECT(backwards.nodeKind(2) == NodeKind::smooth);
    EXPECT(!backwards.nodeKind(1).has_value());
}

TEST(startingAtNodeKeepsTheOutlineAndMovesTheStart) {
    Path2D rotated = square().startingAtNode(2);
    EXPECT(rotated.start == Vector2(100, 100));
    EXPECT(rotated.isClosed() && rotated.segments().size() == 4);
    EXPECT(isApproximatelyEqual(rotated.signedArea(), 10000));
    EXPECT(rotated.segments()[0].end() == Vector2(0, 100));
    EXPECT(rotated.currentPoint() == Vector2(100, 100));
    EXPECT(square().startingAtNode(0).start == Vector2(0, 0));
    EXPECT_PROBLEM(lShape().startingAtNode(1), "closed");
    EXPECT_PROBLEM(square().startingAtNode(9), "no such node");
}

TEST(mappedMovesEveryPointAndKeepsClosureAndMarks) {
    Path2D path = square();
    path.setNodeKind(2, NodeKind::corner);
    AffineTransform2 move = AffineTransform2::translationBy(Vector2(1, 2));
    Path2D moved = path.mapped([&](Vector2 point) { return move.applied(point); });
    EXPECT(moved.start == Vector2(1, 2) && moved.currentPoint() == Vector2(1, 102));
    EXPECT(moved.isClosed() && moved.nodeKind(2) == NodeKind::corner);
    EXPECT(isApproximatelyEqual(moved.signedArea(), path.signedArea()));
}

// MARK: Lengths, locate, progress

TEST(lengthsAddUpAndLocateFindsThePiece) {
    MeasuredPath2D measured(lShape());
    EXPECT(isApproximatelyEqual(measured.length(), 30));
    EXPECT(isApproximatelyEqual(measured.pieceLengths()[0], 10) && isApproximatelyEqual(measured.pieceLengths()[1], 20));
    // 1/3 of 30 is exactly the corner: it belongs to the piece that ends there.
    PathPoint2 corner = measured.locate(1.0f / 3);
    EXPECT(corner.pieceIndex == 0 && isApproximatelyEqual(corner.parameter, 1, 1e-4f));
    // Half is 15 along: 5 into the second piece, a quarter of its 20.
    PathPoint2 halfway = measured.locate(0.5f);
    EXPECT(halfway.pieceIndex == 1 && isApproximatelyEqual(halfway.parameter, 0.25f) && isClose(halfway.point, Vector2(10, 5)));
    EXPECT(isApproximatelyEqual(measured.progress(1, 0.25f), 0.5f));
    // Held in 0…1.
    EXPECT(measured.locate(-3).point == Vector2(0, 0) && isClose(measured.locate(7).point, Vector2(10, 20)));
}

TEST(aMetricScalesLengthsButNotCoordinates) {
    // Horizontal runs count double: lengths 20 and 20. Half is the corner itself.
    MeasuredPath2D stretched(lShape(), Vector2(2, 1));
    EXPECT(isApproximatelyEqual(stretched.length(), 40));
    EXPECT(isClose(stretched.pointAt(0.5f), Vector2(10, 0)));
    EXPECT(isClose(stretched.pointAt(0.75f), Vector2(10, 10)));
    EXPECT_PROBLEM(MeasuredPath2D(lShape(), Vector2(0, 1)), "metric");
}

TEST(locateAndProgressAreInverseOnACurve) {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 30), Vector2(40, 30), Vector2(40, 0));
    path.addCubic(Vector2(40, -20), Vector2(60, -20), Vector2(70, 5));
    MeasuredPath2D measured(path);
    for (float share : {0.0f, 0.1f, 0.33f, 0.5f, 0.74f, 0.9f, 1.0f}) {
        PathPoint2 place = measured.locate(share);
        EXPECT(isApproximatelyEqual(measured.progress(place), share, 1e-4f));
    }
    // Equal steps of progress are equal steps of length: the distance walked is the share times the total.
    EXPECT(measured.progress(0, 1) < 1 && measured.progress(0, 1) > 0);
}

TEST(aQuarterCircleHasLengthPiRadiusOverTwo) {
    Path2D arc(Vector2(10, 0));
    arc.addArc(Vector2(0, 0), 10, Angle::degrees(0), Angle::degrees(90));
    MeasuredPath2D measured(arc);
    // 10·π/2 = 15.70796; the cubic approximation is within 3e-4 of the radius share.
    EXPECT(isApproximatelyEqual(measured.length(), 15.70796f, 0.01f));
    EXPECT(isClose(measured.pointAt(0.5f), Vector2(7.0711f, 7.0711f), 0.03f));
}

TEST(directionFollowsThePathAndThePathItsChord) {
    MeasuredPath2D measured(lShape());
    EXPECT(isClose(measured.directionAt(0.1f), Vector2(1, 0)));
    EXPECT(isClose(measured.directionAt(0.9f), Vector2(0, 1)));
    // The chord runs from (0,0) to (10,20): atan2(20, 10) = 63.435°.
    EXPECT(isApproximatelyEqual(measured.chordAngle().radians, std::atan2(20.0f, 10.0f), 1e-5f));
    // Stretched twice across, the tangent of a diagonal tilts towards x.
    Path2D diagonal(Vector2(0, 0));
    diagonal.addLine(Vector2(10, 10));
    EXPECT(isClose(MeasuredPath2D(diagonal, Vector2(2, 1)).directionAt(0.5f), Vector2(0.894427f, 0.447214f)));
}

TEST(nodesAreTheStartAndEachPiecesEnd) {
    MeasuredPath2D measured(lShape());
    std::vector<Vector2> nodes = measured.nodes();
    EXPECT(nodes.size() == 3 && nodes[1] == Vector2(10, 0) && nodes[2] == Vector2(10, 20));
    EXPECT(isApproximatelyEqual(measured.nodeProgress(1), 1.0f / 3));
    std::vector<float> distances = measured.nodeDistances();
    EXPECT(distances.size() == 3 && isApproximatelyEqual(distances[1], 10) && isApproximatelyEqual(distances[2], 30));
    // Half of the second piece alone is 10 + 10 = 20 of 30.
    EXPECT(isApproximatelyEqual(measured.progressWithin(1, 1, 0.5f), 20.0f / 30));
    EXPECT(isApproximatelyEqual(measured.progressWithin(-1, -1, 1), 1));
    EXPECT_PROBLEM(measured.progressWithin(1, 0, 0.5f), "not in the path");
    EXPECT_PROBLEM(measured.nodeProgress(5), "no such node");
}

TEST(aClosedPathMeasuresItsClosingLine) {
    MeasuredPath2D measured(square());
    EXPECT(measured.pieces().size() == 4);
    EXPECT(isApproximatelyEqual(measured.length(), 400));
    EXPECT(isClose(measured.pointAt(0.875f), Vector2(0, 50)));
}

// MARK: Levels and crossings

TEST(crossingsAtALevelComeInOrderAlongThePath) {
    MeasuredPath2D measured(square());
    std::vector<PathPoint2> crossings = measured.crossingsAtX(50);
    EXPECT(crossings.size() == 2);
    EXPECT(isClose(crossings[0].point, Vector2(50, 0)) && isClose(crossings[1].point, Vector2(50, 100)));
    // Along the path: 50 of 400, then bottom + right + half the top = 250 of 400.
    EXPECT(isApproximatelyEqual(measured.progress(crossings[0]), 0.125f, 1e-4f));
    EXPECT(isApproximatelyEqual(measured.progress(crossings[1]), 0.625f, 1e-4f));
    EXPECT(isClose(measured.crossingAtX(50, CrossingChoice::last()).point, Vector2(50, 100)));
    EXPECT(isClose(measured.crossingAtHeight(30, CrossingChoice::at(1)).point, Vector2(0, 30)));
    EXPECT_PROBLEM(measured.crossingAtHeight(500), "does not cross height 500");
    EXPECT_PROBLEM(measured.crossingAtHeight(30, CrossingChoice::at(2)), "does not cross");
}

TEST(aChordSpansTheFirstAndLastCrossing) {
    MeasuredPath2D measured(square());
    Path2D chord = measured.chordAtHeight(40);
    EXPECT(isClose(chord.start, Vector2(100, 40)) && isClose(chord.currentPoint(), Vector2(0, 40)) && chord.segments().size() == 1);
    Path2D upright = measured.chordAtX(25);
    EXPECT(isClose(upright.start, Vector2(25, 0)) && isClose(upright.currentPoint(), Vector2(25, 100)));
}

TEST(aLevelThroughANodeIsOneCrossing) {
    // The level y = 0 lies along the bottom edge, which crosses nothing; it meets the neighbouring sides only
    // at the corners (100, 0) and (0, 0), once each.
    MeasuredPath2D measured(square());
    std::vector<PathPoint2> touches = measured.crossingsAtHeight(0);
    EXPECT(touches.size() == 2 && isClose(touches[0].point, Vector2(100, 0)) && isClose(touches[1].point, Vector2(0, 0)));
    // A V: the level through its tip meets the two pieces at one point.
    Path2D vee(Vector2(0, 10));
    vee.addLine(Vector2(5, 0));
    vee.addLine(Vector2(10, 10));
    EXPECT(MeasuredPath2D(vee).crossingsAtHeight(0).size() == 1);
}

TEST(twoPathsCrossWhereTheirOutlinesMeet) {
    Path2D across(Vector2(-10, 50));
    across.addLine(Vector2(110, 50));
    std::vector<PathCrossing2> crossings = MeasuredPath2D(square()).crossings(MeasuredPath2D(across));
    EXPECT(crossings.size() == 2);
    // In order along the square: the right side (100, 50) at 150/400, then the left side (0, 50) at 350/400.
    EXPECT(isClose(crossings[0].point, Vector2(100, 50)) && isApproximatelyEqual(crossings[0].progress, 0.375f, 1e-4f));
    EXPECT(isClose(crossings[1].point, Vector2(0, 50)) && isApproximatelyEqual(crossings[1].progress, 0.875f, 1e-4f));
    // Along the line of 120: 110 to the right side, 10 to the left.
    EXPECT(isApproximatelyEqual(crossings[0].otherProgress, 110.0f / 120, 1e-4f));
    EXPECT(isApproximatelyEqual(crossings[1].otherProgress, 10.0f / 120, 1e-4f));
}

TEST(anApertureOpensTheLoopOnTheSideItFaces) {
    // Facing up (90°), offset 0.5: the cut is at y = 50 + 0.5·50 = 75. The longest run below it starts on the
    // left side at (0, 75), goes down, along the bottom and up the right side to (100, 75): 75 + 100 + 75.
    Path2D opened = MeasuredPath2D(square()).aperture(0.5f, Angle::degrees(90));
    EXPECT(!opened.isClosed());
    EXPECT(isClose(opened.start, Vector2(0, 75)) && isClose(opened.currentPoint(), Vector2(100, 75)));
    EXPECT(isApproximatelyEqual(MeasuredPath2D(opened).length(), 250, 1e-2f));
    EXPECT_PROBLEM(MeasuredPath2D(lShape()).aperture(0), "closed path");
    EXPECT_PROBLEM(MeasuredPath2D(square()).aperture(1.5f, Angle::degrees(90)), "does not open");
}

// MARK: Slice, cut, divided

TEST(sliceTakesTheStretchBetweenTwoProgresses) {
    // 0.25 of 30 is 7.5 along the first piece; 0.75 is 22.5: 12.5 into the second.
    Path2D part = MeasuredPath2D(lShape()).slice(0.25f, 0.75f);
    EXPECT(part.segments().size() == 2);
    EXPECT(isClose(part.start, Vector2(7.5f, 0)) && isClose(part.currentPoint(), Vector2(10, 12.5f)));
    EXPECT(part.segments()[0].isLine());
    // Property: the stretch's length is the difference of the progresses times the total.
    EXPECT(isApproximatelyEqual(MeasuredPath2D(part).length(), 15, 1e-3f));
    EXPECT_PROBLEM(MeasuredPath2D(lShape()).slice(0.5f, 0.5f), "nothing between");
    EXPECT_PROBLEM(MeasuredPath2D(lShape()).slice(0.8f, 0.2f), "nothing between");
}

TEST(sliceOfACurveStaysOnTheCurve) {
    Path2D path(Vector2(0, 0));
    path.addCubic(Vector2(0, 30), Vector2(40, 30), Vector2(40, 0));
    MeasuredPath2D measured(path);
    Path2D part = measured.slice(0.2f, 0.6f);
    EXPECT(isClose(part.start, measured.pointAt(0.2f), 1e-3f) && isClose(part.currentPoint(), measured.pointAt(0.6f), 1e-3f));
    EXPECT(isApproximatelyEqual(MeasuredPath2D(part).length(), 0.4f * measured.length(), 2e-3f * measured.length()));
}

TEST(cutAddsNodesWithoutChangingTheShape) {
    MeasuredPath2D measured(lShape());
    Path2D cut = measured.cut({0.5f});
    EXPECT(cut.segments().size() == 3);
    EXPECT(isClose(cut.segments()[1].end(), Vector2(10, 5)));
    EXPECT(isApproximatelyEqual(MeasuredPath2D(cut).length(), 30, 1e-3f));
    // A share on a node, or twice, adds nothing more.
    EXPECT(measured.cut({1.0f / 3, 0.0f, 1.0f}).segments().size() == 2);
    EXPECT(measured.cut({0.5f, 0.5f}).segments().size() == 3);
    EXPECT(measured.cut({}) == lShape());
    // Marks stay with their nodes; the new node is unmarked.
    Path2D marked = lShape();
    marked.setNodeKind(2, NodeKind::corner);
    Path2D markedCut = MeasuredPath2D(marked).cut({0.5f});
    EXPECT(markedCut.nodeKind(3) == NodeKind::corner && !markedCut.nodeKind(2).has_value());
}

TEST(dividedMakesEqualLengthPieces) {
    // Thirds of 30: the corner at 10, and a cut 10 into the second piece at (10, 10).
    Path2D thirds = MeasuredPath2D(lShape()).divided(3);
    EXPECT(thirds.segments().size() == 3);
    EXPECT(isClose(thirds.segments()[1].end(), Vector2(10, 10)));
    MeasuredPath2D measured(thirds);
    for (float length : measured.pieceLengths()) EXPECT(isApproximatelyEqual(length, 10, 1e-3f));
    // A closed square in eight: one cut in the middle of each side, closed and the same area.
    Path2D eighths = MeasuredPath2D(square()).divided(8);
    EXPECT(eighths.isClosed() && eighths.segments().size() == 8);
    EXPECT(isApproximatelyEqual(eighths.signedArea(), 10000, 1e-2f));
    EXPECT(MeasuredPath2D(lShape()).divided(1) == lShape());
    EXPECT_PROBLEM(MeasuredPath2D(lShape()).divided(0), "one or more");
}

TEST(measuredReversedAndStartedElsewhereKeepTheLength) {
    MeasuredPath2D measured(square());
    EXPECT(isApproximatelyEqual(measured.reversed().length(), 400));
    EXPECT(isApproximatelyEqual(measured.startingAtNode(1).length(), 400));
    EXPECT(measured.startingAtNode(1).path().start == Vector2(100, 0));
    EXPECT(isApproximatelyEqual(MeasuredPath2D(lShape()).mapped([](Vector2 point) { return point * 2; }).length(), 60));
}
