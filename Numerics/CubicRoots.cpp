// Real roots of a cubic equation (Engine/Math/Numerics/CubicRoots.swift). The method is written
// out on `CubicRoots::solve` in CubicRoots.h.
#include "CubicRoots.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "../Linear/Scalar.h"
#include "QuadraticRoots.h"

namespace editor {

namespace {

/// The binary exponent of a finite non-zero float: floor(log2 |value|), as Swift's `Float.exponent`.
int binaryExponent(float value) { return std::ilogb(value); }

/// The spacing between `value` and the next float of larger magnitude, as Swift's `Float.ulp`
/// (for the largest float, the gap below it).
float unitInTheLastPlace(float value) {
    float magnitude = std::fabs(value);
    if (magnitude == 0) return std::ldexp(1.0f, -149);
    int exponent = std::max(std::ilogb(magnitude), -126);
    return std::ldexp(1.0f, exponent - 23);
}

/// The roots of the quadratic that the cubic becomes when its leading term is zero.
std::vector<float> rootsOfQuadratic(float quadratic, float linear, float constant) {
    QuadraticRoots roots = QuadraticRoots::solve(quadratic, linear, constant);
    switch (roots.kind) {
    case QuadraticRoots::Kind::none: return {};
    case QuadraticRoots::Kind::one: return {roots.root()};
    case QuadraticRoots::Kind::two: return {roots.smaller, roots.larger};
    }
    return {};
}

std::vector<float> sortedWithoutDuplicates(std::vector<float> roots) {
    std::sort(roots.begin(), roots.end());
    std::vector<float> result;
    for (float root : roots) {
        if (result.empty() || root != result.back()) result.push_back(root);
    }
    return result;
}

/// How far outside the interval a root may be computed and still count as on its end: 4 ulps of
/// the larger finite end (0 if both ends are infinite).
float endSlack(float lowerBound, float upperBound) {
    bool foundFiniteEnd = false;
    float largest = 0;
    for (float end : {lowerBound, upperBound}) {
        if (!std::isfinite(end)) continue;
        float magnitude = std::fabs(end);
        if (!foundFiniteEnd || magnitude > largest) largest = magnitude;
        foundFiniteEnd = true;
    }
    if (!foundFiniteEnd) return 0;
    return 4 * unitInTheLastPlace(largest);
}

/// A quadratic a·u² + linear·u + constant left after dividing out a root of the cubic.
struct DeflatedQuadratic {
    double linear;
    double constant;
};

struct DividedCubic {
    double value;
    double slope;
    double linear;
    double constant;
};

/// `A·u³ + B·u² + C·u + D` in `double`: the float cubic with t = 2ᵏ·u and the whole equation
/// multiplied by a power of two, so its largest root and its leading coefficient are both about 1
/// (step 1 of `CubicRoots::solve`).
class ScaledCubic {
public:
    /// Scales exactly by powers of two. With e(x) the binary exponent of x, the root bound
    /// max(|b/a|, √|c/a|, ∛|d/a|) is about 2^max((e(b) − e(a))/1, (e(c) − e(a))/2, (e(d) − e(a))/3),
    /// rounded up; then a·2³ᵏ sets the overall power of two. Float exponents run −149…127, so every
    /// exponent here stays far inside double's −1074…1023.
    ScaledCubic(float cubic, float quadratic, float linear, float constant) {
        int cubicExponent = binaryExponent(cubic);
        const std::pair<float, int> lowerTerms[] = {{quadratic, 1}, {linear, 2}, {constant, 3}};
        bool anyTerm = false;
        int largestExponent = 0;
        for (const auto& [coefficient, degreesBelowCubic] : lowerTerms) {
            if (coefficient == 0) continue;
            int termExponent = static_cast<int>(std::ceil(
                static_cast<double>(binaryExponent(coefficient) - cubicExponent) / static_cast<double>(degreesBelowCubic)));
            if (!anyTerm || termExponent > largestExponent) largestExponent = termExponent;
            anyTerm = true;
        }
        rootExponent = anyTerm ? largestExponent : 0;
        int overallExponent = cubicExponent + 3 * rootExponent;
        auto scaled = [&](float coefficient, int power) {
            return std::ldexp(static_cast<double>(coefficient), power * rootExponent - overallExponent);
        };
        a = scaled(cubic, 3);
        b = scaled(quadratic, 2);
        c = scaled(linear, 1);
        d = scaled(constant, 0);
    }

    /// t = 2ᵏ·u: a root of the scaled cubic back in the caller's units.
    double unscaled(double root) const { return std::ldexp(root, rootExponent); }

    /// Steps 2 and 3 of `CubicRoots::solve`: Kahan's Newton iteration to one real root, and the
    /// quadratic left after dividing it out.
    std::pair<double, DeflatedQuadratic> outerRootAndDeflatedQuadratic() const {
        // u = 0 is a root: the quadratic is what is left after taking out the factor u.
        if (d == 0) return {0, DeflatedQuadratic{b, c}};

        double current = -(b / a) / 3;
        DividedCubic division = divided(current);
        double valueOverLeading = division.value / a;
        double direction = valueOverLeading < 0 ? -1 : 1;
        double reach = std::cbrt(std::fabs(valueOverLeading));
        double negativeSlopeOverLeading = -division.slope / a;
        if (negativeSlopeOverLeading > 0) {
            reach = 1.324718 * std::max(reach, std::sqrt(negativeSlopeOverLeading));
        }
        double next = current - direction * reach;
        // Newton steps from outside; each moves against `direction`, towards the root. A step that
        // does not (the root is reached, to the last bit) ends it. Dividing the step by a hair more
        // than 1 keeps rounding from carrying it past the root (Kahan). The limit is only a guard.
        int steps = 0;
        while (next != current && steps < 200) {
            current = next;
            division = divided(current);
            next = division.slope == 0 ? current : current - (division.value / division.slope) / 1.000000000000001;
            steps += 1;
            if (direction * next <= direction * current) break;
        }
        double root = current;
        if (!(std::fabs(a) * root * root > std::fabs(d / root))) {
            return {root, DeflatedQuadratic{division.linear, division.constant}};
        }
        // The big root: b₁ and c₂ from the low end, without cancellation.
        double constantTerm = -d / root;
        return {root, DeflatedQuadratic{(constantTerm - c) / root, constantTerm}};
    }

    /// Step 4 of `CubicRoots::solve`: the real roots of a·u² + linear·u + constant, with a
    /// grazing double root kept when rounding made the discriminant negative.
    std::vector<double> realRoots(DeflatedQuadratic quadratic) const {
        double halfLinear = -quadratic.linear / 2;
        double discriminant = halfLinear * halfLinear - a * quadratic.constant;
        double vertex = halfLinear / a;
        if (!(discriminant >= 0)) {
            bool isDoubleRoot = isZeroWithinRounding(vertex) && isFlatWithinRounding(vertex);
            return isDoubleRoot ? std::vector<double>{vertex} : std::vector<double>{};
        }
        double stableSum = halfLinear + (halfLinear >= 0 ? std::sqrt(discriminant) : -std::sqrt(discriminant));
        // stableSum = 0 only when halfLinear and the discriminant are both 0: a double root at 0.
        if (stableSum == 0) return {0};
        return {quadratic.constant / stableSum, stableSum / a};
    }

    /// Step 5 of `CubicRoots::solve`: `roots` (ascending) with each run of neighbours that the
    /// cubic cannot separate, zero within rounding at their midpoint, replaced by the midpoint.
    std::vector<double> mergingRootsTooCloseToSeparate(const std::vector<double>& roots) const {
        std::vector<double> merged;
        for (double root : roots) {
            if (!merged.empty()) {
                double previous = merged.back();
                double midpoint = (previous + root) / 2;
                if (previous == root || isZeroWithinRounding(midpoint)) {
                    merged.back() = midpoint;
                    continue;
                }
            }
            merged.push_back(root);
        }
        return merged;
    }

private:
    double a = 0, b = 0, c = 0, d = 0;
    /// k in t = 2ᵏ·u.
    int rootExponent = 0;

    /// One step of synthetic division at `u`: f(x) = (x − u)·(a·x² + b₁·x + c₂) + f(u), and the
    /// slope f′(u), all from Horner's rule.
    DividedCubic divided(double u) const {
        double leading = a * u;
        double linear = leading + b;
        double constant = linear * u + c;
        return DividedCubic{constant * u + d, (leading + linear) * u + constant, linear, constant};
    }

    double value(double u) const { return divided(u).value; }
    double slope(double u) const { return divided(u).slope; }

    /// |a·u³| + |b·u²| + |c·u| + |d|: what the rounding error of `value` is measured against.
    double termSize(double u) const {
        return std::fabs(a * u * u * u) + std::fabs(b * u * u) + std::fabs(c * u) + std::fabs(d);
    }

    /// |3a·u²| + |2b·u| + |c|: the same for `slope`.
    double slopeTermSize(double u) const { return std::fabs(3 * a * u * u) + std::fabs(2 * b * u) + std::fabs(c); }

    /// Whether the cubic at `u` is zero as far as `double` evaluation can tell.
    bool isZeroWithinRounding(double u) const {
        return std::fabs(value(u)) <= CubicRoots::roundingAllowance * std::numeric_limits<double>::epsilon() * termSize(u);
    }

    /// Whether the slope at `u` is zero as far as `double` evaluation can tell.
    bool isFlatWithinRounding(double u) const {
        return std::fabs(slope(u))
            <= CubicRoots::roundingAllowance * std::numeric_limits<double>::epsilon() * slopeTermSize(u);
    }
};

} // namespace

std::vector<float> CubicRoots::solve(float cubic, float quadratic, float linear, float constant) {
    if (!(std::isfinite(cubic) && std::isfinite(quadratic) && std::isfinite(linear) && std::isfinite(constant))) {
        return {};
    }
    if (cubic == 0) return rootsOfQuadratic(quadratic, linear, constant);

    ScaledCubic scaled(cubic, quadratic, linear, constant);
    auto [outerRoot, deflated] = scaled.outerRootAndDeflatedQuadratic();
    std::vector<double> roots = {outerRoot};
    for (double root : scaled.realRoots(deflated)) roots.push_back(root);
    std::sort(roots.begin(), roots.end());
    std::vector<double> merged = scaled.mergingRootsTooCloseToSeparate(roots);
    std::vector<float> unscaled;
    for (double root : merged) {
        float asFloat = static_cast<float>(scaled.unscaled(root));
        if (std::isfinite(asFloat)) unscaled.push_back(asFloat);
    }
    return sortedWithoutDuplicates(unscaled);
}

std::vector<float> CubicRoots::solve(float cubic, float quadratic, float linear, float constant, float lowerBound,
                                     float upperBound) {
    float slack = endSlack(lowerBound, upperBound);
    std::vector<float> nearInterval;
    for (float root : solve(cubic, quadratic, linear, constant)) {
        if (root >= lowerBound - slack && root <= upperBound + slack) {
            nearInterval.push_back(clamped(root, lowerBound, upperBound));
        }
    }
    return sortedWithoutDuplicates(nearInterval);
}

} // namespace editor
