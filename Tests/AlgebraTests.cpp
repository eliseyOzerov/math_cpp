// Ports of EditorTests/Math/Matrix4Tests.swift, Algebra/Matrix3Tests, Matrix4InverseTests,
// QuaternionTests, QuaternionAlignmentTests, Vector3TiltTests and the Transform tests of
// EditorTests/Scene (SceneTransformTests without JSON, RigidTransformTests). The Matrix4 camera
// functions are renderer-only, so the two inverse tests that need a view or a projection build
// theirs with the small helpers below (the same formulas as Matrix4+Camera.swift).
#include <cstdint>
#include <limits>
#include <vector>

#include "../Linear/Matrix3.h"
#include "../Linear/Matrix4.h"
#include "../Linear/Quaternion.h"
#include "../Linear/Vector3Tilt.h"
#include "../Linear/MathProblem.h"
#include "../Linear/Transform.h"
#include "Testing.h"

using namespace editor;
using editor::testing::hasComponents;
using editor::testing::isApproximatelyEqual;
using editor::testing::isApproximatelyEqualRelative;
using editor::testing::isInverse;

namespace {

constexpr float notANumber = std::numeric_limits<float>::quiet_NaN();
constexpr float infinity = std::numeric_limits<float>::infinity();
const Vector3 xAxis = Vector3::xAxis();
const Vector3 yAxis = Vector3::yAxis();
const Vector3 zAxis = Vector3::zAxis();

/// The lens of a camera, as in Matrix4+Camera.swift (Metal clip space, depth 0 to 1).
Matrix4 testPerspective(Angle fieldOfView, float aspectRatio, float nearPlane, float farPlane) {
    float yScale = 1 / (fieldOfView / 2).tangent();
    float xScale = yScale / aspectRatio;
    float zScale = farPlane / (nearPlane - farPlane);
    return Matrix4(Vector4(xScale, 0, 0, 0), Vector4(0, yScale, 0, 0), Vector4(0, 0, zScale, zScale * nearPlane),
                   Vector4(0, 0, -1, 0));
}

/// The placement of a camera, as in Matrix4+Camera.swift.
Matrix4 testLookAt(Vector3 eye, Vector3 target, Vector3 up) {
    Vector3 forward = (target - eye).normalized();
    Vector3 right = forward.cross(up).normalized();
    Vector3 trueUp = right.cross(forward);
    return Matrix4(Vector4(right.x, right.y, right.z, -right.dot(eye)),
                   Vector4(trueUp.x, trueUp.y, trueUp.z, -trueUp.dot(eye)),
                   Vector4(-forward.x, -forward.y, -forward.z, forward.dot(eye)), Vector4(0, 0, 0, 1));
}

/// A small deterministic generator (xorshift64*) for the random-pairs test.
class SeededRandom {
public:
    explicit SeededRandom(std::uint64_t seed) : state(seed * 0x9E3779B97F4A7C15ull + 1) {}

    /// A float in −1…1.
    float nextSigned() {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        std::uint64_t mixed = state * 0x2545F4914F6CDD1Dull;
        return static_cast<float>(static_cast<double>(mixed >> 11) / 4503599627370496.0 - 1.0);
    }

    /// A unit vector, drawn uniformly by rejecting points outside the unit ball.
    Vector3 nextUnitVector3() {
        while (true) {
            Vector3 candidate(nextSigned(), nextSigned(), nextSigned());
            float size = candidate.length();
            if (size > 0.1f && size <= 1) return candidate / size;
        }
    }

private:
    std::uint64_t state;
};

} // namespace

// MARK: Matrix4 basics

TEST(matrix4IdentityChangesNothing) {
    Vector3 point(1, 2, 3);
    EXPECT(Matrix4::identity().transformPoint(point) == point);
}

TEST(matrix4TranslationMovesPoints) {
    // (1, 2, 3) + (10, 20, 30) = (11, 22, 33)
    Matrix4 move = Matrix4::translation(Vector3(10, 20, 30));
    EXPECT(move.transformPoint(Vector3(1, 2, 3)) == Vector3(11, 22, 33));
}

TEST(matrix4TranslationDoesNotMoveDirections) {
    Matrix4 move = Matrix4::translation(Vector3(10, 20, 30));
    EXPECT(move.transformDirection(Vector3(1, 2, 3)) == Vector3(1, 2, 3));
}

TEST(matrix4ScaleMultipliesEachAxis) {
    // (1·2, 1·3, 1·4) = (2, 3, 4)
    Matrix4 stretch = Matrix4::scale(Vector3(2, 3, 4));
    EXPECT(stretch.transformPoint(Vector3(1, 1, 1)) == Vector3(2, 3, 4));
}

TEST(quarterTurnAroundZTakesXToY) {
    // The 2D rotation: x' = cos 90°·1 − sin 90°·0 = 0, y' = sin 90°·1 + cos 90°·0 = 1.
    Matrix4 turn = Matrix4::rotation(zAxis, Angle::degrees(90));
    EXPECT(isApproximatelyEqual(turn.transformPoint(xAxis), yAxis));
}

TEST(quarterTurnAroundXTakesYToZ) {
    Matrix4 turn = Matrix4::rotation(xAxis, Angle::degrees(90));
    EXPECT(isApproximatelyEqual(turn.transformPoint(yAxis), zAxis));
}

TEST(quarterTurnAroundYTakesZToX) {
    Matrix4 turn = Matrix4::rotation(yAxis, Angle::degrees(90));
    EXPECT(isApproximatelyEqual(turn.transformPoint(zAxis), xAxis));
}

TEST(negativeAngleTurnsTheOtherWay) {
    Matrix4 turn = Matrix4::rotation(zAxis, Angle::degrees(-90));
    EXPECT(isApproximatelyEqual(turn.transformPoint(xAxis), -yAxis));
}

TEST(rotationAxisLengthDoesNotMatter) {
    // (0, 0, 5) is normalized to (0, 0, 1), so this is the same as turning around z.
    Matrix4 turn = Matrix4::rotation(Vector3(0, 0, 5), Angle::degrees(90));
    EXPECT(isApproximatelyEqual(turn.transformPoint(xAxis), yAxis));
}

TEST(rotationAroundAZeroAxisIsRefused) {
    EXPECT_PROBLEM(Matrix4::rotation(Vector3(0, 0, 0), Angle::degrees(90)), "A rotation axis must have a direction");
}

TEST(thirdOfATurnAroundTheDiagonalCyclesTheAxes) {
    // Turning 120° around (1, 1, 1) swaps the axes in a cycle: x → y → z → x.
    Matrix4 turn = Matrix4::rotation(Vector3(1, 1, 1), Angle::degrees(120));
    EXPECT(isApproximatelyEqual(turn.transformPoint(xAxis), yAxis));
    EXPECT(isApproximatelyEqual(turn.transformPoint(yAxis), zAxis));
    EXPECT(isApproximatelyEqual(turn.transformPoint(zAxis), xAxis));
}

TEST(matrix4RotationKeepsLength) {
    Vector3 point(1, 2, 3);
    Matrix4 turn = Matrix4::rotation(Vector3(2, -1, 0.5f), Angle::degrees(73));
    EXPECT(isApproximatelyEqual(turn.transformPoint(point).length(), point.length()));
}

TEST(rightHandSideHappensFirst) {
    Matrix4 moveRight = Matrix4::translation(Vector3(1, 0, 0));
    Matrix4 twice = Matrix4::scale(Vector3(2, 2, 2));
    Vector3 point(1, 0, 0);

    // Scale first, then move: 1·2 + 1 = 3.
    EXPECT((moveRight * twice).transformPoint(point) == Vector3(3, 0, 0));
    // Move first, then scale: (1 + 1)·2 = 4.
    EXPECT((twice * moveRight).transformPoint(point) == Vector3(4, 0, 0));
}

TEST(multiplyingByIdentityChangesNothing) {
    Matrix4 turn = Matrix4::rotation(zAxis, Angle::degrees(30));
    Vector3 point(1, 2, 3);
    EXPECT((turn * Matrix4::identity()).transformPoint(point) == turn.transformPoint(point));
}

TEST(storageIsColumnByColumn) {
    // Written row by row, the translation sits in the last column. Stored column by column, it
    // must come out as the 4th stored column: (10, 20, 30, 1).
    Matrix4 move = Matrix4::translation(Vector3(10, 20, 30));
    EXPECT(move.column(3) == Vector4(10, 20, 30, 1));
}

TEST(matrix4IsSixtyFourBytes) {
    // 16 floats × 4 bytes, with no padding, so it can be copied straight to the GPU.
    EXPECT(sizeof(Matrix4) == 64);
}

// MARK: Matrix3

namespace {
/// The classic example with determinant 1 and a whole-number inverse.
Matrix3 matrix3Example() { return Matrix3(Vector3(1, 2, 3), Vector3(0, 1, 4), Vector3(5, 6, 0)); }
Matrix3 matrix3Other() { return Matrix3(Vector3(9, 8, 7), Vector3(6, 5, 4), Vector3(3, 2, 1)); }
} // namespace

TEST(transposeSwapsRowsAndColumns) {
    Matrix3 expected(Vector3(1, 0, 5), Vector3(2, 1, 6), Vector3(3, 4, 0));
    EXPECT(isApproximatelyEqual(matrix3Example().transposed(), expected));
}

TEST(transposingTwiceGivesTheOriginal) {
    EXPECT(isApproximatelyEqual(matrix3Example().transposed().transposed(), matrix3Example()));
}

TEST(transposeOfARotationIsItsInverse) {
    Matrix3 rotation = Matrix3::upperLeftOf(Matrix4::rotation(Vector3(1, -2, 3), Angle::degrees(57)));
    EXPECT(isApproximatelyEqual(rotation * rotation.transposed(), Matrix3::identity()));
}

TEST(matrix3DeterminantOfTheIdentityIsOne) { EXPECT(Matrix3::identity().determinant() == 1); }

TEST(matrix3DeterminantOfTheExample) {
    // row1 × row2 = (0, 1, 4) × (5, 6, 0) = (1·0 − 4·6, 4·5 − 0·0, 0·6 − 1·5) = (−24, 20, −5)
    // row0 · that = 1·(−24) + 2·20 + 3·(−5) = −24 + 40 − 15 = 1
    EXPECT(isApproximatelyEqual(matrix3Example().determinant(), 1));
}

TEST(determinantOfADiagonalIsTheProduct) {
    // 2 × 4 × 5 = 40
    Matrix3 diagonal(Vector3(2, 0, 0), Vector3(0, 4, 0), Vector3(0, 0, 5));
    EXPECT(isApproximatelyEqual(diagonal.determinant(), 40));
}

TEST(mirroringHasANegativeDeterminant) {
    Matrix3 mirror(Vector3(-1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));
    EXPECT(isApproximatelyEqual(mirror.determinant(), -1));
}

TEST(determinantOfAProductIsTheProductOfDeterminants) {
    Matrix3 other(Vector3(2, 0, 1), Vector3(1, 3, 0), Vector3(0, 1, 1)); // det = 2·3 − 0 + 1·1 = 7
    EXPECT(isApproximatelyEqual(other.determinant(), 7));
    EXPECT(isApproximatelyEqual((matrix3Example() * other).determinant(), 7, 1e-4f));
}

TEST(inverseOfTheExample) {
    // From the cross products: column 0 = (−24, 20, −5), column 1 = r2 × r0 / 1 =
    // (5,6,0) × (1,2,3) = (6·3 − 0·2, 0·1 − 5·3, 5·2 − 6·1) = (18, −15, 4),
    // column 2 = r0 × r1 = (1,2,3) × (0,1,4) = (2·4 − 3·1, 3·0 − 1·4, 1·1 − 2·0) = (5, −4, 1).
    Matrix3 expected(Vector3(-24, 18, 5), Vector3(20, -15, -4), Vector3(-5, 4, 1));
    std::optional<Matrix3> inverse = matrix3Example().inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, expected, 1e-4f));
}

TEST(inverseOfADiagonalIsTheReciprocals) {
    Matrix3 diagonal(Vector3(2, 0, 0), Vector3(0, 4, 0), Vector3(0, 0, 5));
    Matrix3 expected(Vector3(0.5f, 0, 0), Vector3(0, 0.25f, 0), Vector3(0, 0, 0.2f));
    std::optional<Matrix3> inverse = diagonal.inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, expected));
}

TEST(matrix3InverseTimesMatrixIsTheIdentityAtEveryScale) {
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Matrix3 turn = Matrix3::upperLeftOf(Matrix4::rotation(Vector3(1, -2, 3), Angle::degrees(40)));
        Matrix3 scaling = Matrix3::upperLeftOf(Matrix4::scale(Vector3(1, 2, 3) * scale));
        Matrix3 matrix = turn * scaling;
        std::optional<Matrix3> inverse = matrix.inverse();
        EXPECT(inverse.has_value());
        if (!inverse) continue;
        EXPECT(isApproximatelyEqual(*inverse * matrix, Matrix3::identity(), 1e-5f));
        EXPECT(isApproximatelyEqual(matrix * *inverse, Matrix3::identity(), 1e-5f));
    }
}

TEST(singularMatricesHaveNoInverse) {
    // Row 2 = row 0 + row 1: the rows lie in one plane.
    Matrix3 flat(Vector3(1, 2, 3), Vector3(4, 5, 6), Vector3(5, 7, 9));
    EXPECT(!flat.inverse().has_value());
    Matrix3 zeroRow(Vector3(1, 0, 0), Vector3(0, 0, 0), Vector3(0, 0, 1));
    EXPECT(!zeroRow.inverse().has_value());
}

TEST(matrix3SingularityIsJudgedRelativeToScale) {
    // The same flat matrix at 1 mm and 1 km scale must be rejected at both.
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Matrix3 flat(Vector3(1, 2, 3) * scale, Vector3(4, 5, 6) * scale, Vector3(5, 7, 9) * scale);
        EXPECT(!flat.inverse().has_value());
    }
}

TEST(tinyButHealthyMatrixHasAnInverse) {
    // Scale 0.001 in all axes: determinant 1e-9, far below any absolute 1e-6 tolerance.
    Matrix3 tiny = Matrix3::upperLeftOf(Matrix4::scale(Vector3(0.001f, 0.001f, 0.001f)));
    std::optional<Matrix3> inverse = tiny.inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(inverse->entry(0, 0), 1000, 0.01f));
}

TEST(normalMatrixOfARotationIsTheRotation) {
    // For a rotation, inverse-transpose = (transpose)-transpose = the rotation itself.
    Matrix4 model = Matrix4::rotation(Vector3(1, 1, 0), Angle::degrees(33));
    std::optional<Matrix3> normalMatrix = Matrix3::normalMatrix(model);
    EXPECT(normalMatrix.has_value());
    if (normalMatrix) EXPECT(isApproximatelyEqual(*normalMatrix, Matrix3::upperLeftOf(model)));
}

TEST(normalMatrixIgnoresTranslation) {
    Matrix4 model = Matrix4::translation(Vector3(1000, 2000, 3000)) * Matrix4::scale(Vector3(2, 1, 1));
    std::optional<Matrix3> normalMatrix = Matrix3::normalMatrix(model);
    EXPECT(normalMatrix.has_value());
    // Scale (2, 1, 1): inverse-transpose is diag(1/2, 1, 1).
    Matrix3 expected(Vector3(0.5f, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));
    if (normalMatrix) EXPECT(isApproximatelyEqual(*normalMatrix, expected));
}

TEST(normalStaysPerpendicularToTangentUnderNonUniformScale) {
    // Surface with tangent (1, 1, 0) and normal (−1, 1, 0): perpendicular, (−1)+1 = 0.
    Vector3 tangent(1, 1, 0);
    Vector3 normal(-1, 1, 0);
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Matrix4 model = Matrix4::rotation(Vector3(1, 2, 3), Angle::degrees(25)) * Matrix4::scale(Vector3(2, 1, 3) * scale);
        Vector3 newTangent = model.transformDirection(tangent);
        std::optional<Matrix3> normalMatrix = Matrix3::normalMatrix(model);
        EXPECT(normalMatrix.has_value());
        if (!normalMatrix) continue;
        Vector3 newNormal = normalMatrix->transform(normal);
        float cosine = newNormal.normalized().dot(newTangent.normalized());
        EXPECT(isApproximatelyEqual(cosine, 0));

        // The plain model matrix would have broken it.
        Vector3 wrongNormal = model.transformDirection(normal);
        EXPECT(!isApproximatelyEqual(wrongNormal.normalized().dot(newTangent.normalized()), 0, 0.01f));
    }
}

TEST(singularModelHasNoNormalMatrix) {
    EXPECT(!Matrix3::normalMatrix(Matrix4::scale(Vector3(1, 0, 1))).has_value());
}

TEST(matrix3ScaleSoSmallThatItsReciprocalOverflowsHasNoInverse) {
    float tiny = std::numeric_limits<float>::min() / 100; // about 1.2e-40, a subnormal float
    // 1/tiny is about 8e39, beyond float's maximum 3.4e38; the rows are perpendicular, so only the
    // finite-entries check catches it.
    Matrix3 matrix(Vector3(1, 0, 0), Vector3(0, tiny, 0), Vector3(0, 0, 1));
    EXPECT(!matrix.inverse().has_value());
}

TEST(addingMatricesAddsEntryByEntry) {
    // (1,2,3)+(9,8,7) = (10,10,10); (0,1,4)+(6,5,4) = (6,6,8); (5,6,0)+(3,2,1) = (8,8,1)
    Matrix3 expected(Vector3(10, 10, 10), Vector3(6, 6, 8), Vector3(8, 8, 1));
    EXPECT(isApproximatelyEqual(matrix3Example() + matrix3Other(), expected));
    EXPECT(isApproximatelyEqual(matrix3Example() + matrix3Other(), matrix3Other() + matrix3Example()));
}

TEST(subtractingMatricesSubtractsEntryByEntry) {
    // (1,2,3)−(9,8,7) = (−8,−6,−4); (0,1,4)−(6,5,4) = (−6,−4,0); (5,6,0)−(3,2,1) = (2,4,−1)
    Matrix3 expected(Vector3(-8, -6, -4), Vector3(-6, -4, 0), Vector3(2, 4, -1));
    EXPECT(isApproximatelyEqual(matrix3Example() - matrix3Other(), expected));
    EXPECT(isApproximatelyEqual(matrix3Example() - matrix3Example(), Matrix3::diagonal(Vector3(0, 0, 0))));
}

TEST(multiplyingAMatrixByANumberScalesEveryEntry) {
    Matrix3 expected(Vector3(2.5f, 5, 7.5f), Vector3(0, 2.5f, 10), Vector3(12.5f, 15, 0));
    EXPECT(isApproximatelyEqual(matrix3Example() * 2.5f, expected));
    EXPECT(isApproximatelyEqual(matrix3Example() * 1.0f, matrix3Example()));
}

TEST(diagonalPutsTheEntriesOnTheDiagonal) {
    Matrix3 expected(Vector3(2, 0, 0), Vector3(0, 3, 0), Vector3(0, 0, 4));
    EXPECT(isApproximatelyEqual(Matrix3::diagonal(Vector3(2, 3, 4)), expected));
    EXPECT(isApproximatelyEqual(Matrix3::diagonal(Vector3(1, 1, 1)), Matrix3::identity()));
}

// MARK: Matrix4 inverse

namespace {
const std::vector<Vector3> inverseAxes = {xAxis, zAxis, Vector3(1, 1, 1), Vector3(1, -2, 3)};
const std::vector<float> inverseAnglesInDegrees = {-120, -30, 0, 45, 90, 170};
} // namespace

TEST(inverseOfTheIdentityIsTheIdentity) {
    std::optional<Matrix4> inverse = Matrix4::identity().inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, Matrix4::identity()));
}

TEST(inverseOfATranslationIsTheOppositeTranslation) {
    std::optional<Matrix4> inverse = Matrix4::translation(Vector3(10, 20, 30)).inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, Matrix4::translation(Vector3(-10, -20, -30))));
}

TEST(inverseOfAScaleIsTheReciprocals) {
    // 1/2, 1/4, 1/5
    std::optional<Matrix4> inverse = Matrix4::scale(Vector3(2, 4, 5)).inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, Matrix4::scale(Vector3(0.5f, 0.25f, 0.2f))));
}

TEST(inverseOfAMixedMatrix) {
    // matrix = move (1, 2, 3) after a 90° turn around z: point p goes to R p + (1, 2, 3).
    // R takes x to y, so (1, 0, 0) goes to (1, 3, 3). The inverse must bring it back.
    Matrix4 matrix = Matrix4::translation(Vector3(1, 2, 3)) * Matrix4::rotation(zAxis, Angle::degrees(90));
    std::optional<Matrix4> inverse = matrix.inverse();
    EXPECT(inverse.has_value());
    if (!inverse) return;
    EXPECT(isApproximatelyEqual(matrix.transformPoint(xAxis), Vector3(1, 3, 3)));
    EXPECT(isApproximatelyEqual(inverse->transformPoint(Vector3(1, 3, 3)), xAxis));
    EXPECT(isApproximatelyEqual(inverse->transformPoint(Vector3(1, 2, 3)), Vector3(0, 0, 0)));
}

TEST(inverseOfAGeneralMatrixWithWholeNumbers) {
    // Rows (2 0 1 0)(0 1 0 3)(1 0 1 0)(0 0 0 1): the upper-left block [[2,1],[1,1]] in x,z has
    // determinant 2·1 − 1·1 = 1 and inverse [[1,−1],[−1,2]]. y: y' = y + 3, so the inverse gives
    // y = y' − 3.
    Matrix4 matrix(Vector4(2, 0, 1, 0), Vector4(0, 1, 0, 3), Vector4(1, 0, 1, 0), Vector4(0, 0, 0, 1));
    Matrix4 expected(Vector4(1, 0, -1, 0), Vector4(0, 1, 0, -3), Vector4(-1, 0, 2, 0), Vector4(0, 0, 0, 1));
    std::optional<Matrix4> inverse = matrix.inverse();
    EXPECT(inverse.has_value());
    if (inverse) EXPECT(isApproximatelyEqual(*inverse, expected));
}

TEST(matrixTimesInverseIsTheIdentityOnAGrid) {
    for (Vector3 axis : inverseAxes) {
        for (float degrees : inverseAnglesInDegrees) {
            for (float scale : {0.001f, 1.0f, 1000.0f}) {
                for (float distance : {0.0f, 1.0f, 1000.0f}) {
                    Matrix4 matrix = Matrix4::translation(Vector3(1, -2, 0.5f) * distance)
                                   * Matrix4::rotation(axis, Angle::degrees(degrees))
                                   * Matrix4::scale(Vector3(1, 2, 0.5f) * scale);
                    std::optional<Matrix4> inverse = matrix.inverse();
                    EXPECT(inverse.has_value());
                    if (!inverse) continue;
                    EXPECT(isInverse(matrix, *inverse));
                    EXPECT(isInverse(*inverse, matrix));
                }
            }
        }
    }
}

TEST(perspectiveTimesInverseIsTheIdentity) {
    for (float nearPlane : {0.01f, 0.1f, 1.0f}) {
        for (float farPlane : {10.0f, 100.0f, 1000.0f}) {
            for (float fieldOfView : {30.0f, 60.0f, 100.0f}) {
                Matrix4 projection = testPerspective(Angle::degrees(fieldOfView), 1.6f, nearPlane, farPlane);
                std::optional<Matrix4> inverse = projection.inverse();
                EXPECT(inverse.has_value());
                if (!inverse) continue;
                EXPECT(isInverse(projection, *inverse));
                EXPECT(isInverse(*inverse, projection));
            }
        }
    }
}

TEST(viewAndProjectionInverseTurnsClipPointsBackIntoTheWorld) {
    // The picking use: world → clip, then the inverse brings the point back.
    Matrix4 view = testLookAt(Vector3(500, -800, 300), Vector3(0, 0, 0), zAxis);
    Matrix4 projection = testPerspective(Angle::degrees(60), 1.5f, 1, 5000);
    Matrix4 combined = projection * view;
    std::optional<Matrix4> inverse = combined.inverse();
    EXPECT(inverse.has_value());
    if (!inverse) return;

    Vector3 worldPoint(20, 30, 10);
    Vector4 clip = combined.transform(Vector4(worldPoint, 1));
    Vector4 back = inverse->transform(clip);
    // A perspective inverse is ill-conditioned (the far/near ratio is 5000), so the point comes back
    // to about 1e-4 of its distance from the eye (about 1000 here), not 1e-6.
    float distanceFromEye = Vector3(500, -800, 300).distance(worldPoint);
    EXPECT(isApproximatelyEqual(back.xyz() / back.w, worldPoint, 1e-4f * distanceFromEye));
}

TEST(inverseOfARigidTransformIsTransposedRotationAndNegatedTranslation) {
    for (Vector3 axis : inverseAxes) {
        for (float degrees : inverseAnglesInDegrees) {
            for (float distance : {0.001f, 1.0f, 1000.0f}) {
                Vector3 offset = Vector3(1, -2, 0.5f) * distance;
                Matrix4 rotation = Matrix4::rotation(axis, Angle::degrees(degrees));
                Matrix4 rigid = Matrix4::translation(offset) * rotation;
                std::optional<Matrix4> inverse = rigid.inverse();
                EXPECT(inverse.has_value());
                if (!inverse) continue;

                // inverse(p) = Rᵀ (p − offset), with Rᵀ taken from the upper-left 3×3.
                Matrix3 transposedRotation = Matrix3::upperLeftOf(rotation).transposed();
                for (Vector3 point : {Vector3(0, 0, 0), Vector3(1, 2, 3) * distance}) {
                    Vector3 expected = transposedRotation.transform(point - offset);
                    EXPECT(isApproximatelyEqual(inverse->transformPoint(point), expected, 1e-5f * std::max(distance, 1.0f)));
                }
            }
        }
    }
}

TEST(inverseOfTheInverseIsTheOriginal) {
    // The tolerance is relative to the matrix's own largest entry: 1 mm and 1 km matrices (scale
    // and translation both 0.001, or both 1000) get proportionate allowances.
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Matrix4 matrix = Matrix4::translation(Vector3(5, 6, 7) * scale)
                       * Matrix4::rotation(Vector3(1, 2, 3), Angle::degrees(50)) * Matrix4::scale(Vector3(2, 3, 4) * scale);
        std::optional<Matrix4> inverse = matrix.inverse();
        EXPECT(inverse.has_value());
        if (!inverse) continue;
        std::optional<Matrix4> twice = inverse->inverse();
        EXPECT(twice.has_value());
        if (twice) EXPECT(isApproximatelyEqualRelative(*twice, matrix, 1e-5f));
    }
}

TEST(perspectiveTimesLookAtInvertsWithTheEyeFarAway) {
    // fov 60°, near 0.01, far 1e4, looking at the origin from 1 km and 10 km. The determinant is tiny
    // next to the product of the row lengths here, which a Hadamard test wrongly takes for singular.
    for (float distance : {1000.0f, 10000.0f}) {
        Vector3 eye = Vector3(0.6f, -0.8f, 0.0f) * distance;
        float nearPlane = 0.01f;
        Matrix4 projection = testPerspective(Angle::degrees(60), 1, nearPlane, 1e4f);
        Matrix4 view = testLookAt(eye, Vector3(0, 0, 0), zAxis);
        std::optional<Matrix4> inverse = (projection * view).inverse();
        EXPECT(inverse.has_value());
        if (!inverse) continue;

        // Screen centre at depth 0 (the near plane in Metal's 0…1 depth) is one near-plane distance
        // in front of the eye, well inside 1e-3 of the eye distance.
        Vector4 nearClip = inverse->transform(Vector4(0, 0, 0, 1));
        Vector3 nearPoint = nearClip.xyz() / nearClip.w;
        EXPECT(nearPoint.distance(eye) <= 1e-3f * distance);
        EXPECT(nearPoint.distance(eye) <= 2 * nearPlane);

        // Far end of the same ray (depth 1): the direction from the near point to it must be
        // forward. The far point is only as exact as float's 24 bits of depth at a far/near ratio of
        // 1e6, so allow 1e-2.
        Vector4 farClip = inverse->transform(Vector4(0, 0, 1, 1));
        Vector3 farPoint = farClip.xyz() / farClip.w;
        Vector3 forward = (Vector3(0, 0, 0) - eye).normalized();
        EXPECT(isApproximatelyEqual((farPoint - nearPoint).normalized(), forward, 1e-2f));
    }
}

TEST(flatteningScaleHasNoInverse) { EXPECT(!Matrix4::scale(Vector3(1, 0, 1)).inverse().has_value()); }

TEST(matrix4ScaleSoSmallThatItsReciprocalOverflowsHasNoInverse) {
    float tiny = std::numeric_limits<float>::min() / 100; // about 1.2e-40, a subnormal float
    // 1/tiny is about 8e39, beyond float's maximum 3.4e38, so the entry would be infinity. The
    // Hadamard ratio alone cannot see this: the rows stay perpendicular.
    EXPECT(!Matrix4::scale(Vector3(1, tiny, 1)).inverse().has_value());
    // 1e-12 is merely small: the reciprocal 1e12 is representable, so the inverse exists.
    EXPECT(Matrix4::scale(Vector3(1, 1e-12f, 1)).inverse().has_value());
}

TEST(matrixWithTwoEqualRowsHasNoInverse) {
    Matrix4 matrix(Vector4(1, 2, 3, 4), Vector4(5, 6, 7, 8), Vector4(1, 2, 3, 4), Vector4(0, 0, 0, 1));
    EXPECT(!matrix.inverse().has_value());
}

TEST(matrix4SingularityIsJudgedRelativeToScale) {
    // The same flattening at 1 mm and 1 km scale is rejected at both, while the uniformly tiny
    // scale (det 1e-12) is accepted.
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        EXPECT(!Matrix4::scale(Vector3(1, 0, 1) * scale).inverse().has_value());
        EXPECT(Matrix4::scale(Vector3(1, 1, 1) * scale).inverse().has_value());
    }
}

// MARK: Quaternion

namespace {
const std::vector<Vector3> quaternionAxes = {xAxis, yAxis, zAxis, Vector3(1, 1, 1), Vector3(1, -2, 3), Vector3(-3, 0.5f, 2)};
const std::vector<float> quaternionAnglesInDegrees = {-170, -90, -30, 0, 10, 45, 90, 135, 180, 270, 359};
} // namespace

TEST(quaternionQuarterTurnAroundZ) {
    // half angle 45°: vector part (0, 0, sin 45°), scalar part cos 45°; both are √2/2.
    Quaternion quaternion = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    float root = std::sqrt(0.5f);
    EXPECT(isApproximatelyEqual(quaternion.vectorPart, Vector3(0, 0, root)));
    EXPECT(isApproximatelyEqual(quaternion.scalarPart, root));
}

TEST(quaternionAxisLengthDoesNotMatter) {
    Quaternion longAxis = Quaternion::fromAxis(Vector3(0, 0, 7), Angle::degrees(90));
    Quaternion unit = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    EXPECT(hasComponents(longAxis, unit));
}

TEST(quaternionAroundAZeroAxisIsRefused) {
    EXPECT_PROBLEM(Quaternion::fromAxis(Vector3(0, 0, 0), Angle::degrees(10)), "A rotation axis must have a direction");
    EXPECT_PROBLEM(Quaternion(Vector3(0, 0, 0), 0).normalized(), "A zero quaternion cannot be normalized.");
}

TEST(identityRotatesNothing) { EXPECT(Quaternion::identity().rotate(Vector3(1, 2, 3)) == Vector3(1, 2, 3)); }

TEST(constructedQuaternionsHaveLengthOne) {
    for (Vector3 axis : quaternionAxes) {
        for (float degrees : quaternionAnglesInDegrees) {
            EXPECT(isApproximatelyEqual(Quaternion::fromAxis(axis, Angle::degrees(degrees)).length(), 1));
        }
    }
}

TEST(normalizedScalesToLengthOne) {
    // (0, 3, 0) with scalar 4 has length sqrt(9 + 16) = 5, so it becomes (0, 0.6, 0), 0.8.
    Quaternion normalized = Quaternion(Vector3(0, 3, 0), 4).normalized();
    EXPECT(isApproximatelyEqual(normalized.vectorPart, Vector3(0, 0.6f, 0)));
    EXPECT(isApproximatelyEqual(normalized.scalarPart, 0.8f));
}

TEST(twoQuarterTurnsAroundZMakeAHalfTurn) {
    // With q = (0, 0, r, r) and r = √2/2:
    //   vectorPart = r·(0, 0, r) + r·(0, 0, r) + 0 = (0, 0, 2r²) = (0, 0, 1)
    //   scalarPart = r·r − (0, 0, r)·(0, 0, r) = r² − r² = 0
    Quaternion quarter = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    Quaternion product = quarter * quarter;
    EXPECT(isApproximatelyEqual(product.vectorPart, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(product.scalarPart, 0));
}

TEST(productOrderIsRightHandSideFirst) {
    // aroundZ * aroundX turns around x first (x stays x), then around z (x → y): lands on y.
    // aroundX * aroundZ turns around z first (x → y), then around x (y → z): lands on z.
    Quaternion aroundX = Quaternion::fromAxis(xAxis, Angle::degrees(90));
    Quaternion aroundZ = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    EXPECT(isApproximatelyEqual((aroundZ * aroundX).rotate(xAxis), yAxis));
    EXPECT(isApproximatelyEqual((aroundX * aroundZ).rotate(xAxis), zAxis));
}

TEST(conjugateUndoesTheRotation) {
    // (u, s) * (−u, s) = (s·(−u) + s·u + u × (−u), s² + |u|²) = (0, 1).
    for (Vector3 axis : quaternionAxes) {
        Quaternion quaternion = Quaternion::fromAxis(axis, Angle::degrees(77));
        Quaternion product = quaternion * quaternion.conjugate();
        EXPECT(isApproximatelyEqual(product.vectorPart, Vector3(0, 0, 0)));
        EXPECT(isApproximatelyEqual(product.scalarPart, 1));
    }
}

TEST(quaternionProductMatchesMatrixProduct) {
    for (Vector3 firstAxis : quaternionAxes) {
        for (Vector3 secondAxis : quaternionAxes) {
            Quaternion first = Quaternion::fromAxis(firstAxis, Angle::degrees(50));
            Quaternion second = Quaternion::fromAxis(secondAxis, Angle::degrees(-120));
            EXPECT(isApproximatelyEqual((first * second).matrix(), first.matrix() * second.matrix()));
        }
    }
}

TEST(quaternionQuarterTurnAroundZTakesXToY) {
    Quaternion quaternion = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    EXPECT(isApproximatelyEqual(quaternion.rotate(xAxis), yAxis));
}

TEST(rotationMatchesTheMatrixFormulaOnAGrid) {
    for (Vector3 axis : quaternionAxes) {
        for (float degrees : quaternionAnglesInDegrees) {
            Angle angle = Angle::degrees(degrees);
            Quaternion quaternion = Quaternion::fromAxis(axis, angle);
            Matrix4 matrix = Matrix4::rotation(axis, angle);
            EXPECT(isApproximatelyEqual(quaternion.matrix(), matrix));

            // Vectors of 1 mm, 1 and 1 km: the tolerance grows with the vector's size.
            for (float size : {0.001f, 1.0f, 1000.0f}) {
                Vector3 vector = Vector3(1, -2, 0.5f) * size;
                float tolerance = 1e-5f * size;
                EXPECT(isApproximatelyEqual(quaternion.rotate(vector), matrix.transformDirection(vector), tolerance));
            }
        }
    }
}

TEST(quaternionRotationKeepsLength) {
    for (Vector3 axis : quaternionAxes) {
        for (float degrees : quaternionAnglesInDegrees) {
            Quaternion quaternion = Quaternion::fromAxis(axis, Angle::degrees(degrees));
            for (float size : {0.001f, 1.0f, 1000.0f}) {
                Vector3 vector = Vector3(1, -2, 0.5f) * size;
                EXPECT(isApproximatelyEqual(quaternion.rotate(vector).length(), vector.length(), 1e-5f * size));
            }
        }
    }
}

TEST(rotationKeepsTheAxisFixed) {
    for (Vector3 axis : quaternionAxes) {
        Quaternion quaternion = Quaternion::fromAxis(axis, Angle::degrees(123));
        EXPECT(isApproximatelyEqual(quaternion.rotate(axis), axis, 1e-5f * axis.length()));
    }
}

TEST(quaternionAndItsNegativeRotateTheSame) {
    for (Vector3 axis : quaternionAxes) {
        Quaternion quaternion = Quaternion::fromAxis(axis, Angle::degrees(100));
        Vector3 vector(3, -1, 2);
        EXPECT(isApproximatelyEqual(quaternion.rotate(vector), (-quaternion).rotate(vector)));
        EXPECT(isApproximatelyEqual(quaternion.matrix(), (-quaternion).matrix()));
    }
}

TEST(matrixOfAHalfTurnAroundZ) {
    // q = (0, 0, 1, 0): x = y = 0, z = 1, w = 0. The matrix is
    //   row 0: 1 − 2(y²+z²) = −1, 2(xy − wz) = 0, 2(xz + wy) = 0
    //   row 1: 2(xy + wz) = 0, 1 − 2(x²+z²) = −1, 2(yz − wx) = 0
    //   row 2: 2(xz − wy) = 0, 2(yz + wx) = 0, 1 − 2(x²+y²) = 1
    Matrix4 matrix = Quaternion(Vector3(0, 0, 1), 0).matrix();
    Matrix4 expected(Vector4(-1, 0, 0, 0), Vector4(0, -1, 0, 0), Vector4(0, 0, 1, 0), Vector4(0, 0, 0, 1));
    EXPECT(isApproximatelyEqual(matrix, expected));
}

TEST(slerpEndpointsAreTheInputs) {
    Quaternion start = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(20));
    Quaternion end = Quaternion::fromAxis(Vector3(-1, 0, 2), Angle::degrees(140));
    Quaternion atStart = Quaternion::slerp(start, end, 0);
    Quaternion atEnd = Quaternion::slerp(start, end, 1);
    // Both inputs have a positive scalar part (angles under 180°), so slerp returns them
    // themselves, not their negatives: compare component by component.
    EXPECT(hasComponents(atStart, start));
    EXPECT(hasComponents(atEnd, end));
}

TEST(slerpHalfwayBetweenZeroAndNinetyIsFortyFive) {
    // Halfway between 0° and 90° around z is 45° around z: x goes to (√2/2, √2/2, 0).
    Quaternion end = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    Quaternion halfway = Quaternion::slerp(Quaternion::identity(), end, 0.5f);
    float root = std::sqrt(0.5f);
    EXPECT(isApproximatelyEqual(halfway.rotate(xAxis), Vector3(root, root, 0)));
}

TEST(slerpTurnsAtConstantSpeed) {
    // For unit quaternions the dot product of neighbours is cos(half the step angle), so equal steps
    // of `amount` must give equal dot products.
    Quaternion start = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(20));
    Quaternion end = Quaternion::fromAxis(Vector3(-1, 0, 2), Angle::degrees(160));
    std::vector<Quaternion> steps;
    for (int index = 0; index <= 8; index += 1) steps.push_back(Quaternion::slerp(start, end, static_cast<float>(index) / 8));
    float firstStep = steps[0].dot(steps[1]);
    for (int index = 1; index < 8; index += 1) {
        EXPECT(isApproximatelyEqual(steps[index].dot(steps[index + 1]), firstStep));
    }
}

TEST(slerpStaysUnitLength) {
    Quaternion start = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(20));
    Quaternion end = Quaternion::fromAxis(Vector3(-1, 0, 2), Angle::degrees(160));
    for (int step = 0; step <= 20; step += 1) {
        EXPECT(isApproximatelyEqual(Quaternion::slerp(start, end, static_cast<float>(step) / 20).length(), 1));
    }
}

TEST(slerpTakesTheShorterArc) {
    // −end is the same rotation as end, but the long way round. Halfway must still be 45°.
    Quaternion end = Quaternion::fromAxis(zAxis, Angle::degrees(90));
    Quaternion halfway = Quaternion::slerp(Quaternion::identity(), -end, 0.5f);
    float root = std::sqrt(0.5f);
    EXPECT(isApproximatelyEqual(halfway.rotate(xAxis), Vector3(root, root, 0)));
}

TEST(slerpOfNearlyEqualRotationsIsStable) {
    // A tenth of a degree apart: sin φ is tiny, so the straight-blend path is used.
    Quaternion start = Quaternion::fromAxis(zAxis, Angle::degrees(30));
    Quaternion end = Quaternion::fromAxis(zAxis, Angle::degrees(30.1f));
    Quaternion halfway = Quaternion::slerp(start, end, 0.5f);
    EXPECT(isApproximatelyEqual(halfway.length(), 1));
    Quaternion expected = Quaternion::fromAxis(zAxis, Angle::degrees(30.05f));
    // Same sign (both scalar parts are cos(15°) > 0), so the components must match.
    EXPECT(hasComponents(halfway, expected));
    EXPECT(isApproximatelyEqual(halfway.rotate(xAxis), expected.rotate(xAxis)));
}

TEST(slerpBetweenEqualRotationsIsThatRotation) {
    Quaternion rotation = Quaternion::fromAxis(Vector3(1, 1, 0), Angle::degrees(70));
    EXPECT(hasComponents(Quaternion::slerp(rotation, rotation, 0.3f), rotation));
}

// MARK: Quaternion alignment

TEST(xToYIsAQuarterTurnAroundZ) {
    // a × b = (0, 0, 1), 1 + a·b = 1: (0, 0, 1, 1) normalized = (0, 0, √2/2, √2/2), which is the
    // quaternion for 90° around Z.
    Quaternion rotation = Quaternion::rotation(xAxis, yAxis);
    float root = std::sqrt(0.5f);
    EXPECT(isApproximatelyEqual(rotation.vectorPart, Vector3(0, 0, root)));
    EXPECT(isApproximatelyEqual(rotation.scalarPart, root));
}

TEST(zToXIsAQuarterTurnAroundY) {
    // z × x = y; the right-hand turn around +Y takes +Z to +X.
    Quaternion rotation = Quaternion::rotation(zAxis, xAxis);
    EXPECT(isApproximatelyEqual(rotation.rotate(zAxis), xAxis));
    EXPECT(isApproximatelyEqual(rotation.vectorPart, Vector3(0, std::sqrt(0.5f), 0)));
}

TEST(parallelDirectionsGiveTheIdentity) {
    Quaternion rotation = Quaternion::rotation(Vector3(0, 0, 3), Vector3(0, 0, 0.5f));
    EXPECT(isApproximatelyEqual(rotation.vectorPart, Vector3(0, 0, 0)));
    EXPECT(isApproximatelyEqual(rotation.scalarPart, 1));
}

TEST(oppositeDirectionsTurnHalfAroundAPerpendicularAxis) {
    // +Z to −Z: the axis must be perpendicular to Z; the scalar part is cos 90° = 0.
    Quaternion rotation = Quaternion::rotation(zAxis, -zAxis);
    EXPECT(rotation.scalarPart == 0);
    EXPECT(isApproximatelyEqual(rotation.vectorPart.length(), 1));
    EXPECT(isApproximatelyEqual(rotation.vectorPart.dot(zAxis), 0));
    EXPECT(isApproximatelyEqual(rotation.rotate(zAxis), -zAxis));
}

TEST(nearlyOppositeDirectionsStillMapOntoEachOther) {
    // The half-way formula is computed from a + b, so it keeps its digits right up to the fallback.
    // Swift measured |rotate(a) − b| at most 1.8e-7 (179.5°) over these angles, 0 from 179.999° on;
    // bound at about 3× that.
    for (float degrees : {179.0f, 179.5f, 179.9f, 179.99f, 179.999f, 179.9999f, 179.99999f, 180.0f}) {
        Angle tilt = Angle::degrees(degrees);
        Vector3 target(tilt.sine(), 0, tilt.cosine());
        Quaternion rotation = Quaternion::rotation(zAxis, target);
        EXPECT(rotation.rotate(zAxis).distance(target) < 6e-7f);
    }
    // Opposite but for a tiny sideways nudge, on both sides of the fallback (|a × b| ≤ 1e-7).
    // Swift measured at most 2.4e-7 (nudge 1e-7, the fallback's edge).
    for (float nudge : {1e-8f, 1e-7f, 3e-7f, 1e-6f, 1e-5f}) {
        Vector3 target(nudge, nudge, -1);
        Quaternion rotation = Quaternion::rotation(zAxis, target);
        EXPECT(rotation.rotate(zAxis).distance(target.normalized()) < 7e-7f);
    }
}

TEST(randomAndNearlyOppositePairsMapOntoEachOther) {
    // 20 000 random pairs, each also turned onto a direction within 1e-4 and 1e-6 of opposite.
    // Swift measured the worst |rotate(a) − b| over 100 000 pairs as 6.9e-7; the bound is 2e-6.
    SeededRandom random(5);
    float worst = 0;
    for (int pair = 0; pair < 20000; pair += 1) {
        Vector3 source = random.nextUnitVector3();
        Vector3 other = random.nextUnitVector3();
        for (Vector3 target : {other, -source + other * 1e-4f, -source + other * 1e-6f}) {
            Quaternion rotation = Quaternion::rotation(source, target);
            worst = std::max(worst, rotation.rotate(source).distance(target.normalized()));
        }
    }
    EXPECT(worst < 2e-6f);
}

namespace {
const std::vector<Vector3> alignmentDirections = {
    xAxis, yAxis, zAxis, -xAxis, -zAxis, Vector3(1, 1, 1), Vector3(1, -2, 3), Vector3(-3, 0.5f, 2),
    Vector3(0.01f, 0.02f, -1), Vector3(5, 5, 0),
};
} // namespace

TEST(resultIsAUnitQuaternionThatTurnsFromOntoTo) {
    for (Vector3 from : alignmentDirections) {
        for (Vector3 to : alignmentDirections) {
            Quaternion rotation = Quaternion::rotation(from, to);
            EXPECT(isApproximatelyEqual(rotation.length(), 1));
            EXPECT(isApproximatelyEqual(rotation.rotate(from.normalized()), to.normalized(), 2e-5f));
        }
    }
}

TEST(thisIsTheShortestTurn) {
    // The turn angle is 2·arccos(scalar part) and must equal the angle between the directions.
    for (Vector3 from : alignmentDirections) {
        for (Vector3 to : alignmentDirections) {
            Quaternion rotation = Quaternion::rotation(from, to);
            Angle turn = Angle::arcCosine(clamped(rotation.scalarPart, -1, 1)) * 2;
            Angle between = Angle::between(from, to);
            EXPECT(isApproximatelyEqual(turn.degrees(), between.degrees(), 0.05f));
        }
    }
}

TEST(alignmentInputLengthDoesNotMatter) {
    Quaternion reference = Quaternion::rotation(Vector3(1, 2, 3), Vector3(-2, 1, 0.5f));
    for (float factor : {1e-3f, 1e3f, 1e15f, 1e-15f}) {
        Quaternion scaled = Quaternion::rotation(Vector3(1, 2, 3) * factor, Vector3(-2, 1, 0.5f) * (1 / factor));
        EXPECT(hasComponents(scaled, reference, 2e-5f));
    }
}

TEST(zeroOrNonFiniteInputGivesNaN) {
    // NaN in y or z too: a max would quietly drop a NaN that is not its first argument.
    for (Vector3 bad : {Vector3(0, 0, 0), Vector3(notANumber, 0, 1), Vector3(0, notANumber, 1), Vector3(1, 0, notANumber),
                        Vector3(infinity, 0, 0)}) {
        Quaternion rotation = Quaternion::rotation(bad, xAxis);
        EXPECT(std::isnan(rotation.scalarPart) && std::isnan(rotation.vectorPart.x));
        Quaternion reverse = Quaternion::rotation(xAxis, bad);
        EXPECT(std::isnan(reverse.scalarPart));
    }
}

// MARK: Vector3 tilt

TEST(noTiltIsTheParent) {
    Vector3 direction = directionTiltedAwayFrom(Vector3(0, 0, 2), Angle::degrees(0), Angle::degrees(40));
    EXPECT(isApproximatelyEqual(direction, zAxis));
}

TEST(aroundUpTheTurnIsTheAngleFromX) {
    // Parent +Z: a right-angle tilt toward 90° is +Y; toward 0° is +X.
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(zAxis, Angle::degrees(90), Angle::degrees(90)), yAxis));
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(zAxis, Angle::degrees(90), Angle::degrees(0)), xAxis));
    // 30° from +Z toward +X: (sin 30°, 0, cos 30°).
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(zAxis, Angle::degrees(30), Angle::degrees(0)),
                                Vector3(0.5f, 0, 0.8660254f)));
}

TEST(aroundXTheTurnStartsWhereXGoes) {
    // The rotation taking +Z to +X turns 90° about +Y, which takes +X to −Z.
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(xAxis, Angle::degrees(90), Angle::degrees(0)), Vector3(0, 0, -1)));
    // Counterclockwise around +X, a quarter turn on from −Z is +Y.
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(xAxis, Angle::degrees(90), Angle::degrees(90)), yAxis));
}

TEST(aroundDownTheTurnStartsAtMinusX) {
    // The rotation taking +Z to −Z is a half turn about +Y, which takes +X to −X.
    Vector3 down(0, 0, -1);
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(down, Angle::degrees(90), Angle::degrees(0)), Vector3(-1, 0, 0)));
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(down, Angle::degrees(90), Angle::degrees(90)), yAxis));
}

TEST(tinyAndHugeParentsGiveTheSameDirection) {
    for (float scale : {1e-30f, 1e25f}) {
        EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(Vector3(scale, 0, 0), Angle::degrees(90), Angle::degrees(0)),
                                    Vector3(0, 0, -1)));
    }
}

TEST(noDirectionGivesNaN) {
    for (Vector3 parent : {Vector3(0, 0, 0), Vector3(infinity, 0, 0), Vector3(notANumber, 0, 1)}) {
        EXPECT(std::isnan(directionTiltedAwayFrom(parent, Angle::degrees(30), Angle::degrees(0)).x));
    }
}

TEST(negativeTiltFlipsTheTurn) {
    // −30° toward 0° is 30° toward 180°: (−sin 30°, 0, cos 30°).
    EXPECT(isApproximatelyEqual(directionTiltedAwayFrom(zAxis, Angle::degrees(-30), Angle::degrees(0)),
                                Vector3(-0.5f, 0, 0.8660254f)));
}

TEST(tiltResultIsUnitAndMakesTheTiltWithTheParent) {
    Vector3 parents[] = {Vector3(1, 2, 3), Vector3(-0.3f, 0.1f, -2), Vector3(0, 0, -1), Vector3(5, -1, 0.2f)};
    for (Vector3 parent : parents) {
        for (float tilt = 0; tilt <= 180; tilt += 25) {
            for (float turn = 0; turn < 360; turn += 50) {
                Vector3 direction = directionTiltedAwayFrom(parent, Angle::degrees(tilt), Angle::degrees(turn));
                EXPECT(isApproximatelyEqual(direction.length(), 1));
                float cosine = direction.dot(*parent.unitDirection());
                EXPECT(isApproximatelyEqual(cosine, Angle::degrees(tilt).cosine(), 1e-4f));
            }
        }
    }
}

// MARK: Transform

TEST(transformIdentityChangesNothing) {
    Transform transform = Transform::identity();
    EXPECT(isApproximatelyEqual(transform.matrix(), Matrix4::identity()));
    EXPECT(transform.inverseMatrix().has_value() && isApproximatelyEqual(*transform.inverseMatrix(), Matrix4::identity()));
    EXPECT(isApproximatelyEqual(transform.matrix().transformPoint(Vector3(1, 2, 3)), Vector3(1, 2, 3)));
}

TEST(scaleThenRotateThenTranslate) {
    Transform transform{.translation = Vector3(10, 20, 30),
                        .rotation = Quaternion::fromAxis(zAxis, Angle::degrees(90)),
                        .scale = Vector3(2, 3, 4)};
    // (1, 1, 1) → scale → (2, 3, 4) → quarter turn about Z, (x, y) → (−y, x) → (−3, 2, 4)
    // → translate → (7, 22, 34).
    Vector3 point = transform.matrix().transformPoint(Vector3(1, 1, 1));
    EXPECT(isApproximatelyEqual(point, Vector3(7, 22, 34), 1e-4f));
}

TEST(transformTranslationDoesNotMoveDirections) {
    Transform transform{.translation = Vector3(5, 5, 5), .scale = Vector3(2, 2, 2)};
    EXPECT(isApproximatelyEqual(transform.matrix().transformDirection(Vector3(1, 0, 0)), Vector3(2, 0, 0)));
}

TEST(transformInverseUndoesTheMatrix) {
    Transform transform{.translation = Vector3(1, -2, 3),
                        .rotation = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(70)),
                        .scale = Vector3(2, 0.5f, 4)};
    std::optional<Matrix4> inverse = transform.inverseMatrix();
    EXPECT(inverse.has_value());
    if (!inverse) return;
    EXPECT(isApproximatelyEqual(*inverse * transform.matrix(), Matrix4::identity(), 1e-5f));
    Vector3 point(0.3f, -4, 9);
    Vector3 roundTrip = inverse->transformPoint(transform.matrix().transformPoint(point));
    EXPECT(isApproximatelyEqual(roundTrip, point, 1e-4f));
}

TEST(flatteningScaleHasNoInverseOrNormalMatrix) {
    Transform flat{.scale = Vector3(1, 0, 1)};
    EXPECT(!flat.inverseMatrix().has_value());
    EXPECT(!flat.normalMatrix().has_value());
}

TEST(transformNormalMatrixKeepsNormalsPerpendicularUnderNonUniformScale) {
    Transform transform{.translation = Vector3(4, 5, 6),
                        .rotation = Quaternion::fromAxis(zAxis, Angle::degrees(30)),
                        .scale = Vector3(2, 1, 1)};
    // Before turning, stretching x by 2 maps the tangent (1, 1, 0) to (2, 1, 0) and the normal
    // (−1, 1, 0) to (−1/2, 1, 0): still perpendicular, 2·(−1/2) + 1 = 0. The rotation keeps that.
    Vector3 tangent(1, 1, 0);
    Vector3 normal(-1, 1, 0);
    Vector3 movedTangent = transform.matrix().transformDirection(tangent);
    std::optional<Matrix3> normalMatrix = transform.normalMatrix();
    EXPECT(normalMatrix.has_value());
    if (!normalMatrix) return;
    Vector3 movedNormal = normalMatrix->transform(normal);
    EXPECT(isApproximatelyEqual(movedTangent.dot(movedNormal), 0, 1e-5f));
    // The plain matrix would not have kept them perpendicular.
    EXPECT(std::fabs(movedTangent.dot(transform.matrix().transformDirection(normal))) > 0.5f);
}

TEST(aTransformScaleSoSmallThatItsReciprocalOverflowsHasNoInverse) {
    // A tenth of the smallest normal float (about 1.2e-39) is denormal; its reciprocal, about 8.5e38,
    // is above the largest float (3.4e38), so the inverse would hold infinity and NaN.
    float tinyScale = std::numeric_limits<float>::min() / 10;
    Transform tiny{.translation = Vector3(1, 2, 3), .scale = Vector3(tinyScale, 1, 1)};
    EXPECT(!tiny.inverseMatrix().has_value());
}

TEST(transformsAtMillimetreAndKilometreScalesInvertAndKeepNormalsPerpendicular) {
    for (float scale : {0.001f, 1.0f, 1000.0f}) {
        Transform transform{.translation = Vector3(3, -2, 5) * scale,
                            .rotation = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(70)),
                            .scale = Vector3(2, 1, 0.5f) * scale};
        std::optional<Matrix4> inverse = transform.inverseMatrix();
        EXPECT(inverse.has_value());
        std::optional<Matrix3> normalMatrix = transform.normalMatrix();
        EXPECT(normalMatrix.has_value());
        if (!inverse || !normalMatrix) continue;
        Vector3 point(0.3f, -0.4f, 0.9f);
        Vector3 roundTrip = inverse->transformPoint(transform.matrix().transformPoint(point));
        // Swift measured the round-trip error at scales 1e-3, 1, 1e3: 4.2e-7, 4.1e-7, 6.1e-7.
        // Tolerance 2e-6, about 3x the largest.
        EXPECT((roundTrip - point).length() < 2e-6f);

        Vector3 tangent(1, 1, 0);
        Vector3 normal = Vector3(-1, 1, 0).normalized();
        Vector3 movedTangent = transform.matrix().transformDirection(tangent).normalized();
        Vector3 movedNormal = normalMatrix->transform(normal).normalized();
        // Swift measured the dot product of the moved tangent and normal (0 when perpendicular):
        // 4.5e-8, 7.5e-9, 6.3e-8. Tolerance 2e-7, about 3x the largest.
        EXPECT(std::fabs(movedTangent.dot(movedNormal)) < 2e-7f);
    }
}

// MARK: Rigid transforms

namespace {
Quaternion quarterTurnAboutZ() { return Quaternion::fromAxis(zAxis, Angle::degrees(90)); }

/// The per-axis factors of the reflection that negates the coordinate of `axis`.
Vector3 reflectionFactors(MirrorAxis axis) {
    switch (axis) {
    case MirrorAxis::x: return Vector3(-1, 1, 1);
    case MirrorAxis::y: return Vector3(1, -1, 1);
    case MirrorAxis::z: return Vector3(1, 1, -1);
    }
    return Vector3(1, 1, 1);
}
} // namespace

TEST(composedTurnsTheChildsOffsetByTheParent) {
    // Parent: at (1, 0, 0), a quarter turn about z. Child: one step along its parent's x. A quarter
    // turn takes (1, 0, 0) to (0, 1, 0), so the child sits at (1, 1, 0).
    Transform parent{.translation = Vector3(1, 0, 0), .rotation = quarterTurnAboutZ()};
    Transform child{.translation = Vector3(1, 0, 0), .rotation = quarterTurnAboutZ()};
    Transform composed = parent.composed(child);
    EXPECT(isApproximatelyEqual(composed.translation, Vector3(1, 1, 0)));
    // Two quarter turns are a half turn: (0, 0, 1 | 0).
    EXPECT(isApproximatelyEqual(composed.rotation.vectorPart, Vector3(0, 0, 1)));
    EXPECT(isApproximatelyEqual(composed.rotation.scalarPart, 0));
}

TEST(composedMatrixIsTheProductOfTheMatrices) {
    Transform parent{.translation = Vector3(0.3f, -1, 2), .rotation = Quaternion::fromAxis(Vector3(1, 2, 3), Angle::degrees(40))};
    Transform child{.translation = Vector3(-2, 0.5f, 1), .rotation = Quaternion::fromAxis(Vector3(-1, 0, 2), Angle::degrees(110))};
    EXPECT(isApproximatelyEqual(parent.composed(child).matrix(), parent.matrix() * child.matrix(), 1e-5f));
}

TEST(rigidInverseOfAQuarterTurnAndAMove) {
    // Undo: turn back a quarter, (x, y) → (y, −x), then move back. −R⁻¹·(1, 2, 3) = −(2, −1, 3).
    Transform transform{.translation = Vector3(1, 2, 3), .rotation = quarterTurnAboutZ()};
    Transform inverse = transform.rigidInverse();
    EXPECT(isApproximatelyEqual(inverse.translation, Vector3(-2, 1, -3)));
    EXPECT(isApproximatelyEqual(inverse.rotation.vectorPart, Vector3(0, 0, -quarterTurnAboutZ().vectorPart.z)));
    for (Transform product : {transform.composed(inverse), inverse.composed(transform)}) {
        EXPECT(isApproximatelyEqual(product.matrix(), Matrix4::identity(), 1e-6f));
    }
}

TEST(mirroredAcrossXFlipsTheSideAndTheTurn) {
    // A frame at (+0.2, 0, 1) turned a quarter toward +y from +x: its mirror sits at (−0.2, 0, 1)
    // turned a quarter the other way.
    Transform left{.translation = Vector3(0.2f, 0, 1), .rotation = quarterTurnAboutZ()};
    Transform right = left.mirrored(MirrorAxis::x);
    EXPECT(right.translation == Vector3(-0.2f, 0, 1));
    EXPECT(right.rotation.vectorPart == Vector3(0, 0, -quarterTurnAboutZ().vectorPart.z));
    EXPECT(right.rotation.scalarPart == quarterTurnAboutZ().scalarPart);
}

TEST(mirroredIsTheReflectionOnBothSides) {
    for (MirrorAxis axis : {MirrorAxis::x, MirrorAxis::y, MirrorAxis::z}) {
        Transform transform{.translation = Vector3(0.4f, -1.5f, 2),
                            .rotation = Quaternion::fromAxis(Vector3(1, -2, 0.5f), Angle::degrees(65))};
        Matrix4 reflection = Matrix4::scale(reflectionFactors(axis));
        Matrix4 expected = reflection * transform.matrix() * reflection;
        EXPECT(isApproximatelyEqual(transform.mirrored(axis).matrix(), expected, 1e-5f));
        EXPECT(transform.mirrored(axis).mirrored(axis) == transform);
    }
}

TEST(reversesHandednessOnlyForAnOddNumberOfNegativeScales) {
    EXPECT(!Transform::identity().reversesHandedness());
    EXPECT(Transform{.scale = Vector3(-1, 1, 1)}.reversesHandedness());
    EXPECT(!Transform{.scale = Vector3(-1, -1, 1)}.reversesHandedness());
    EXPECT(Transform{.scale = Vector3(-2, -0.5f, -1)}.reversesHandedness());
}
