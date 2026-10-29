// Tests of Curves/Deform2D: the perspective map, the lattice and the puppet solver. Every expected
// number is worked out by hand or follows from a property that must always hold.
#include <cmath>
#include <vector>

#include "../Curves/Deform2D.h"
#include "Testing.h"

using namespace editor;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-4f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

const std::array<Vector2, 4> unitSquare = {Vector2(0, 0), Vector2(1, 0), Vector2(1, 1), Vector2(0, 1)};
/// A trapezoid: bottom edge 4 wide, top edge 2 wide and 2 higher.
const std::array<Vector2, 4> trapezoid = {Vector2(0, 0), Vector2(4, 0), Vector2(3, 2), Vector2(1, 2)};

} // namespace

// MARK: Homography

TEST(squareToTrapezoidSendsCornersToCornersAndCentreToTheDiagonalsCrossing) {
    // Heckbert: Δx1 = 1, Δx2 = −2, Δy1 = −2, Δy2 = 0, Σx = −2, Σy = 0 give g = 0, h = 1, a = 4, b = 2, e = 4:
    // (x, y) → ((4x + 2y) ÷ (y + 1), 4y ÷ (y + 1)). The centre goes to (3 ÷ 1.5, 2 ÷ 1.5) = (2, 4/3), where the
    // diagonals (0,0)-(3,2) and (4,0)-(1,2) cross (at s = 2/3 along each).
    Homography map = Homography::squareToQuadrilateral(trapezoid);
    for (int corner = 0; corner < 4; ++corner) EXPECT(isClose(map.applied(unitSquare[corner]), trapezoid[corner]));
    EXPECT(isClose(map.applied(Vector2(0.5f, 0.5f)), Vector2(2, 4.0f / 3.0f)));
    EXPECT(isApproximatelyEqual(map.entries[6], 0, 1e-6f) && isApproximatelyEqual(map.entries[7], 1, 1e-6f));
}

TEST(aParallelogramGivesAnAffineMap) {
    std::array<Vector2, 4> parallelogram = {Vector2(1, 1), Vector2(3, 1), Vector2(4, 3), Vector2(2, 3)};
    Homography map = Homography::squareToQuadrilateral(parallelogram);
    EXPECT(map.entries[6] == 0 && map.entries[7] == 0);
    EXPECT(isClose(map.applied(Vector2(0.5f, 0.5f)), Vector2(2.5f, 2)));
}

TEST(quadrilateralToItselfIsTheIdentityAndTheInverseUndoesTheMap) {
    Homography identity = Homography::fromQuadrilaterals(trapezoid, trapezoid);
    EXPECT(isClose(identity.applied(Vector2(1.7f, 0.9f)), Vector2(1.7f, 0.9f)));
    std::array<Vector2, 4> other = {Vector2(-1, 0), Vector2(5, 1), Vector2(6, 4), Vector2(-2, 3)};
    Homography map = Homography::fromQuadrilaterals(trapezoid, other);
    for (int corner = 0; corner < 4; ++corner) EXPECT(isClose(map.applied(trapezoid[corner]), other[corner]));
    Homography back = map.inverse();
    for (Vector2 point : {Vector2(1, 1), Vector2(2.5f, 0.3f), Vector2(0.2f, 1.9f)}) EXPECT(isClose(back.applied(map.applied(point)), point, 2e-4f));
}

TEST(aMapKeepsStraightLinesStraight) {
    Homography map = Homography::squareToQuadrilateral(trapezoid);
    Vector2 first = map.applied(Vector2(0.1f, 0.2f)), middle = map.applied(Vector2(0.3f, 0.5f)), last = map.applied(Vector2(0.5f, 0.8f));
    EXPECT(std::fabs((middle - first).cross(last - first)) < 1e-4f);
    EXPECT_PROBLEM(Homography::squareToQuadrilateral({Vector2(0, 0), Vector2(1, 0), Vector2(2, 0), Vector2(3, 0)}), "line");
}

// MARK: Lattice

TEST(anUndeformedLatticeIsTheIdentity) {
    Lattice2D lattice = Lattice2D::undeformed(Vector2(-1, 2), Vector2(5, 6), 4, 3);
    EXPECT(lattice.points.size() == 12);
    for (Vector2 point : {Vector2(-1, 2), Vector2(0.3f, 3.1f), Vector2(4.9f, 5.5f), Vector2(2, 4), Vector2(7, 9)}) {
        EXPECT(isClose(lattice.applied(point), point, 1e-4f));
    }
}

TEST(movingACornerControlPointMovesTheCornerAndAShiftMovesEverything) {
    Lattice2D lattice = Lattice2D::undeformed(Vector2(0, 0), Vector2(2, 2), 3, 3);
    lattice.points[8] = Vector2(3, 4);
    EXPECT(isClose(lattice.applied(Vector2(2, 2)), Vector2(3, 4)));
    EXPECT(isClose(lattice.applied(Vector2(0, 0)), Vector2(0, 0)));
    Lattice2D shifted = Lattice2D::undeformed(Vector2(0, 0), Vector2(2, 2), 3, 3);
    for (Vector2& point : shifted.points) point = point + Vector2(1, -2);
    EXPECT(isClose(shifted.applied(Vector2(0.7f, 1.3f)), Vector2(1.7f, -0.7f)));
    EXPECT_PROBLEM(Lattice2D::undeformed(Vector2(0, 0), Vector2(1, 1), 1, 3), "at least 2");
}

TEST(aRaisedMiddleRowOfAThreeRowLatticeLiftsTheCentreByHalfTheLift) {
    // Quadratic Bernstein weights at v = ½ are (¼, ½, ¼): lifting only the middle row by 2 lifts the centre by ½ · 2 = 1.
    Lattice2D lattice = Lattice2D::undeformed(Vector2(0, 0), Vector2(2, 2), 3, 3);
    for (int column = 0; column < 3; ++column) lattice.points[3 + column] = lattice.points[3 + column] + Vector2(0, 2);
    EXPECT(isClose(lattice.applied(Vector2(1, 1)), Vector2(1, 2)));
}

// MARK: Puppet

TEST(unmovedPinsLeaveThePlaneAsItWas) {
    Bounds2 box{Vector2(0, 0), Vector2(10, 6)};
    std::vector<Vector2> pins = {Vector2(2.3f, 1.6f), Vector2(7.9f, 4.2f), Vector2(5.13f, 3.3f)};
    PuppetMesh mesh(box, 20, pins);
    std::vector<Vector2> solved = mesh.solved(pins);
    for (int vertex = 0; vertex < mesh.vertexCount(); ++vertex) EXPECT(isClose(solved[vertex], mesh.restVertices()[vertex], 1e-4f));
    for (Vector2 point : {Vector2(1, 1), Vector2(6.2f, 3.7f), Vector2(9.9f, 5.9f)}) EXPECT(isClose(mesh.mapped(point, solved), point, 1e-4f));
}

TEST(aRigidTurnOfEveryPinTurnsThePlane) {
    Bounds2 box{Vector2(0, 0), Vector2(10, 6)};
    std::vector<Vector2> pins = {Vector2(2.3f, 1.6f), Vector2(7.9f, 4.2f), Vector2(5.13f, 3.3f), Vector2(8.6f, 1.2f)};
    const Angle turn = Angle::degrees(37);
    const Vector2 pivot(3, 2);
    std::vector<Vector2> targets;
    for (Vector2 pin : pins) targets.push_back(pin.rotated(turn, pivot) + Vector2(0.5f, -1));
    PuppetMesh mesh(box, 20, pins);
    std::vector<Vector2> solved = mesh.solved(targets);
    for (Vector2 point : {Vector2(1, 1), Vector2(6.2f, 3.7f), Vector2(9.9f, 5.9f), Vector2(0, 6)}) {
        EXPECT(isClose(mesh.mapped(point, solved), point.rotated(turn, pivot) + Vector2(0.5f, -1), 2e-3f));
    }
}

TEST(aSinglePinDraggedTranslatesThePlane) {
    Bounds2 box{Vector2(0, 0), Vector2(4, 4)};
    PuppetMesh mesh(box, 8, {Vector2(1.3f, 2.1f)});
    std::vector<Vector2> solved = mesh.solved({Vector2(2.3f, 1.1f)});
    EXPECT(isClose(mesh.mapped(Vector2(1.3f, 2.1f), solved), Vector2(2.3f, 1.1f), 1e-3f));
    EXPECT(isClose(mesh.mapped(Vector2(3.5f, 0.5f), solved), Vector2(4.5f, -0.5f), 1e-3f));
}

TEST(pinsReachTheirTargetsAndTheFarSideStaysPutWhenBending) {
    // A pin at the left end held, one at the right end lifted: the pinned points land exactly on their targets.
    Bounds2 box{Vector2(0, 0), Vector2(10, 2)};
    std::vector<Vector2> pins = {Vector2(0.4f, 1.15f), Vector2(9.6f, 0.95f)};
    PuppetMesh mesh(box, 40, pins);
    std::vector<Vector2> targets = {pins[0], Vector2(9.0f, 3.0f)};
    std::vector<Vector2> solved = mesh.solved(targets, 12);
    EXPECT(isClose(mesh.mapped(pins[0], solved), targets[0], 2e-3f));
    EXPECT(isClose(mesh.mapped(pins[1], solved), targets[1], 2e-3f));
    for (Vector2 vertex : solved) EXPECT(vertex.isFinite());
}

TEST(aPinOutsideTheMeshIsAProblem) {
    EXPECT_PROBLEM(PuppetMesh(Bounds2{Vector2(0, 0), Vector2(1, 1)}, 4, {Vector2(2, 2)}), "outside");
}
