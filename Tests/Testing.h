// A small test runner: `TEST(name) { ... }` registers a test, `EXPECT(condition)` records a
// failure without stopping the test, and `EXPECT_PROBLEM(statement, words)` checks that the
// statement throws a problem whose sentence contains `words`. `make test` runs them all.
#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "../Linear/Vector3.h"
#include "../Linear/Matrix3.h"
#include "../Linear/Matrix4.h"
#include "../Linear/Quaternion.h"

namespace editor::testing {

struct Test {
    std::string name;
    std::function<void()> body;
};

inline std::vector<Test>& tests() {
    static std::vector<Test> all;
    return all;
}

inline int& failures() {
    static int count = 0;
    return count;
}

struct Registration {
    Registration(std::string name, std::function<void()> body) { tests().push_back({std::move(name), std::move(body)}); }
};

inline void fail(const char* file, int line, const std::string& what) {
    failures() += 1;
    std::cerr << file << ":" << line << ": " << what << "\n";
}

/// Whether two floats are within `tolerance` of each other.
inline bool isApproximatelyEqual(float first, float second, float tolerance = 1e-5f) {
    return std::fabs(first - second) <= tolerance;
}

inline bool isApproximatelyEqual(Vector3 first, Vector3 second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance)
        && isApproximatelyEqual(first.z, second.z, tolerance);
}

/// Whether every entry of two Matrix3 values is within `tolerance` of the matching entry.
inline bool isApproximatelyEqual(const Matrix3& first, const Matrix3& second, float tolerance = 1e-5f) {
    for (int row = 0; row < 3; row += 1) {
        for (int column = 0; column < 3; column += 1) {
            if (!isApproximatelyEqual(first.entry(row, column), second.entry(row, column), tolerance)) return false;
        }
    }
    return true;
}

/// Whether every entry of two Matrix4 values is within `tolerance` of the matching entry.
inline bool isApproximatelyEqual(const Matrix4& first, const Matrix4& second, float tolerance = 1e-5f) {
    for (int row = 0; row < 4; row += 1) {
        for (int column = 0; column < 4; column += 1) {
            if (!isApproximatelyEqual(first.entry(row, column), second.entry(row, column), tolerance)) return false;
        }
    }
    return true;
}

/// Whether every entry of two Matrix4 values is within `relativeTolerance` × (the largest absolute
/// entry of either matrix) of the matching entry, so one call suits 1 mm and 1 km matrices.
inline bool isApproximatelyEqualRelative(const Matrix4& first, const Matrix4& second, float relativeTolerance) {
    float largest = 0;
    for (int row = 0; row < 4; row += 1) {
        for (int column = 0; column < 4; column += 1) {
            largest = std::max({largest, std::fabs(first.entry(row, column)), std::fabs(second.entry(row, column))});
        }
    }
    return isApproximatelyEqual(first, second, relativeTolerance * largest);
}

/// Whether the vector part and the scalar part of two quaternions each match within `tolerance`.
/// Unlike a dot product this is tight, and it does not accept the negative quaternion, which is the
/// same rotation with different components.
inline bool hasComponents(const Quaternion& first, const Quaternion& second, float tolerance = 1e-5f) {
    return isApproximatelyEqual(first.vectorPart, second.vectorPart, tolerance)
        && isApproximatelyEqual(first.scalarPart, second.scalarPart, tolerance);
}

/// Whether `first × second` is the identity, entry by entry.
///
/// Each entry of the product is a sum of 4 products first[i,k] × second[k,j]. Rounding error in
/// such a sum is proportional to the size of its terms, so the tolerance for each entry is
/// `roundingMultiple × epsilon × Σ |first[i,k]| × |second[k,j]|`, never less than the size of the
/// expected entry (1 on the diagonal, 0 off it). A matrix with 1 km translations gets a looser
/// tolerance in exactly the entries that involve the translation, while the rotation block stays
/// tight. The default multiple of 64 is about 4 times the error of two matrices built by the
/// cofactor inverse, and far below the errors a wrong cofactor sign would give.
inline bool isInverse(const Matrix4& first, const Matrix4& second, float roundingMultiple = 64) {
    Matrix4 product = first * second;
    for (int row = 0; row < 4; row += 1) {
        for (int column = 0; column < 4; column += 1) {
            float expected = row == column ? 1.0f : 0.0f;
            float termSize = 0;
            for (int index = 0; index < 4; index += 1) {
                termSize += std::fabs(first.entry(row, index) * second.entry(index, column));
            }
            float tolerance = roundingMultiple * std::numeric_limits<float>::epsilon() * std::max(termSize, std::fabs(expected));
            if (!isApproximatelyEqual(product.entry(row, column), expected, tolerance)) return false;
        }
    }
    return true;
}

} // namespace editor::testing

#define EDITOR_JOIN_INNER(first, second) first##second
#define EDITOR_JOIN(first, second) EDITOR_JOIN_INNER(first, second)

#define TEST(name)                                                                                         \
    static void name();                                                                                    \
    static ::editor::testing::Registration EDITOR_JOIN(registration_, name)(#name, name);                 \
    static void name()

#define EXPECT(condition)                                                                                  \
    do {                                                                                                   \
        if (!(condition)) ::editor::testing::fail(__FILE__, __LINE__, "expected " #condition);            \
    } while (false)

#define EXPECT_EQUAL(actual, expected)                                                                     \
    do {                                                                                                   \
        auto actualValue = (actual);                                                                       \
        auto expectedValue = (expected);                                                                   \
        if (!(actualValue == expectedValue)) {                                                             \
            std::ostringstream message;                                                                    \
            message << "expected " #actual " == " #expected ", got " << actualValue;                     \
            ::editor::testing::fail(__FILE__, __LINE__, message.str());                                    \
        }                                                                                                  \
    } while (false)

#define EXPECT_PROBLEM(statement, words)                                                                   \
    do {                                                                                                   \
        try {                                                                                              \
            statement;                                                                                     \
            ::editor::testing::fail(__FILE__, __LINE__, "expected a problem from " #statement);           \
        } catch (const std::exception& problem) {                                                          \
            if (std::string(problem.what()).find(words) == std::string::npos) {                            \
                ::editor::testing::fail(                                                                   \
                    __FILE__, __LINE__, std::string("expected a problem with \"") + (words) + "\", got \"" \
                                            + problem.what() + "\"");                                      \
            }                                                                                              \
        }                                                                                                  \
    } while (false)
