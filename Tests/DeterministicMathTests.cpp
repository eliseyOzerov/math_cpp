// The deterministic tier's math functions (Numerics/DeterministicMath.h): special values, values
// worked out by hand, identities, and a sweep of 100,000 inputs per function checked against the
// system's libm to within 1 ulp (2 for the few functions documented as such). The sweep prints
// the largest difference it saw for each function.
#include <bit>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>

#include "../Numerics/DeterministicMath.h"
#include "Testing.h"

namespace deterministic = editor::deterministic;

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double quietNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double largestDouble = std::numeric_limits<double>::max();
constexpr double smallestSubnormal = std::numeric_limits<double>::denorm_min();
constexpr double pi = 3.14159265358979323846;

uint64_t bitsOf(double value) { return std::bit_cast<uint64_t>(value); }

bool isNaN(double value) { return value != value; }
bool isPositiveZero(double value) { return bitsOf(value) == 0; }
bool isNegativeZero(double value) { return bitsOf(value) == 0x8000000000000000ULL; }

/// How many representable doubles lie between `first` and `second` (0 when they are equal, 1 for
/// neighbours; ±0 count as one number). Two NaNs are 0 apart, a NaN and a number are far apart.
int64_t ulpDistance(double first, double second) {
    if (isNaN(first) || isNaN(second)) return (isNaN(first) && isNaN(second)) ? 0 : INT64_MAX;
    auto ordered = [](double value) -> int64_t {
        const uint64_t bits = bitsOf(value);
        const int64_t magnitude = static_cast<int64_t>(bits & 0x7fffffffffffffffULL);
        return (bits >> 63) != 0 ? -magnitude : magnitude;
    };
    const int64_t difference = ordered(first) - ordered(second);
    return difference < 0 ? -difference : difference;
}

/// `actual` is within `ulps` representable doubles of `expected`.
bool isWithinUlps(double actual, double expected, int64_t ulps) { return ulpDistance(actual, expected) <= ulps; }

/// The same stream of numbers on every machine, for the sweeps.
class Stream {
public:
    explicit Stream(uint64_t seed) : state(seed) {}

    uint64_t next() {
        state += 0x9e3779b97f4a7c15ULL;
        uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }

    /// Uniform in [0, 1).
    double unit() { return static_cast<double>(next() >> 11) * 0x1p-53; }
    double between(double low, double high) { return low + (high - low) * unit(); }
    /// ±10^e with e uniform in [lowExponent, highExponent].
    double decades(double lowExponent, double highExponent) {
        const double magnitude = std::pow(10.0, between(lowExponent, highExponent));
        return (next() & 1) != 0 ? -magnitude : magnitude;
    }

private:
    uint64_t state;
};

constexpr int sweepCount = 100000;

/// Compares `ours` with `system` over `sweepCount` inputs from `input(index, stream)`, fails if
/// any result is more than `allowedUlps` apart, and prints the largest distance.
template <typename Input, typename Ours, typename System>
void sweep(const std::string& name, uint64_t seed, int64_t allowedUlps, Input input, Ours ours, System system) {
    Stream stream(seed);
    int64_t largest = 0;
    int failedInputs = 0;
    for (int index = 0; index < sweepCount; index += 1) {
        const auto arguments = input(index, stream);
        const int64_t distance = std::apply([&](auto... values) { return ulpDistance(ours(values...), system(values...)); }, arguments);
        if (distance > largest) largest = distance;
        if (distance > allowedUlps && failedInputs < 3) {
            failedInputs += 1;
            std::apply([&](auto... values) {
                std::cerr << name << " off by " << distance << " ulp at " << std::setprecision(17);
                ((std::cerr << values << " "), ...);
                std::cerr << "ours " << ours(values...) << " system " << system(values...) << "\n";
            }, arguments);
        }
    }
    std::cout << "deterministic::" << name << ": max " << largest << " ulp from the system over " << sweepCount << " inputs\n";
    EXPECT(largest <= allowedUlps);
}

/// An argument of sin, cos and tan: near the reduction boundaries, small, medium and huge.
double angleInput(int index, Stream& stream) {
    switch (index % 5) {
        case 0: return stream.between(-20.0, 20.0);
        case 1: {  // within a few ulps of a multiple of π/4, where the reduction changes case
            const double multiple = std::floor(stream.between(-2000.0, 2000.0)) * (pi / 4);
            return multiple + stream.between(-1e-9, 1e-9);
        }
        case 2: return stream.decades(-9, 9);  // up to 1e9
        case 3: return stream.between(-1.6, 1.6);
        default: return stream.between(-1e6, 1e6);
    }
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Special values.
// ---------------------------------------------------------------------------------------------

TEST(DeterministicSinCosTanSpecialValues) {
    EXPECT(isPositiveZero(deterministic::sin(0.0)));
    EXPECT(isNegativeZero(deterministic::sin(-0.0)));
    EXPECT(isPositiveZero(deterministic::tan(0.0)));
    EXPECT(isNegativeZero(deterministic::tan(-0.0)));
    EXPECT(deterministic::cos(0.0) == 1.0 && deterministic::cos(-0.0) == 1.0);
    for (double value : {infinity, -infinity, quietNaN}) {
        EXPECT(isNaN(deterministic::sin(value)));
        EXPECT(isNaN(deterministic::cos(value)));
        EXPECT(isNaN(deterministic::tan(value)));
    }
    // Tiny arguments: sin(x) = x, cos(x) = 1, and the smallest subnormal survives.
    EXPECT(deterministic::sin(smallestSubnormal) == smallestSubnormal);
    EXPECT(deterministic::tan(-smallestSubnormal) == -smallestSubnormal);
    EXPECT(deterministic::cos(smallestSubnormal) == 1.0);
    // The largest double goes through the multi-word reduction.
    EXPECT(std::fabs(deterministic::sin(largestDouble)) <= 1.0);
    EXPECT(isWithinUlps(deterministic::sin(largestDouble), std::sin(largestDouble), 1));
    EXPECT(isWithinUlps(deterministic::cos(largestDouble), std::cos(largestDouble), 1));
    EXPECT(isWithinUlps(deterministic::tan(-largestDouble), std::tan(-largestDouble), 1));
}

TEST(DeterministicSinCosOfHugeArguments) {
    // Worked out with 500-digit arithmetic: the arguments are exact doubles, so these are exact
    // sines and cosines of those doubles, rounded to the nearest double.
    EXPECT(deterministic::sin(1e22) == -0.8522008497671888);
    EXPECT(deterministic::cos(1e22) == 0.523214785395139);
    EXPECT(isWithinUlps(deterministic::tan(1e22), -1.6287782256068988, 1));
    const double twoToTheThousand = 0x1p1000;
    EXPECT(deterministic::sin(twoToTheThousand) == -0.15920170308624243);
    EXPECT(deterministic::cos(twoToTheThousand) == 0.9872460775989135);
    EXPECT(deterministic::tan(twoToTheThousand) == -0.16125837995065806);
    // Odd symmetry carries over to the large-argument path.
    EXPECT(deterministic::sin(-1e22) == 0.8522008497671888);
    EXPECT(deterministic::cos(-twoToTheThousand) == deterministic::cos(twoToTheThousand));
}

TEST(DeterministicAsinAcosAtanSpecialValues) {
    EXPECT(isPositiveZero(deterministic::asin(0.0)));
    EXPECT(isNegativeZero(deterministic::asin(-0.0)));
    EXPECT(deterministic::asin(1.0) == pi / 2);
    EXPECT(deterministic::asin(-1.0) == -pi / 2);
    EXPECT(isNaN(deterministic::asin(1.0000000000000002)));
    EXPECT(isNaN(deterministic::asin(-infinity)));
    EXPECT(isNaN(deterministic::asin(quietNaN)));

    EXPECT(isPositiveZero(deterministic::acos(1.0)));
    EXPECT(deterministic::acos(-1.0) == pi);
    EXPECT(deterministic::acos(0.0) == pi / 2);
    EXPECT(deterministic::acos(-0.0) == pi / 2);
    EXPECT(isNaN(deterministic::acos(-1.0000000000000002)));
    EXPECT(isNaN(deterministic::acos(infinity)));
    EXPECT(isNaN(deterministic::acos(quietNaN)));

    EXPECT(isPositiveZero(deterministic::atan(0.0)));
    EXPECT(isNegativeZero(deterministic::atan(-0.0)));
    EXPECT(deterministic::atan(infinity) == pi / 2);
    EXPECT(deterministic::atan(-infinity) == -pi / 2);
    EXPECT(deterministic::atan(largestDouble) == pi / 2);
    EXPECT(deterministic::atan(smallestSubnormal) == smallestSubnormal);
    EXPECT(isNaN(deterministic::atan(quietNaN)));
}

TEST(DeterministicAtan2QuadrantsAndSignedZeros) {
    // Every quadrant, at a point whose angle is easy to check: (±1, ±1) and (±1, ±2).
    EXPECT(deterministic::atan2(1.0, 1.0) * 4 == pi);
    EXPECT(deterministic::atan2(1.0, -1.0) * 4 == 3 * pi);
    EXPECT(deterministic::atan2(-1.0, -1.0) * 4 == -3 * pi);
    EXPECT(deterministic::atan2(-1.0, 1.0) * 4 == -pi);
    EXPECT(isWithinUlps(deterministic::atan2(2.0, 1.0) + deterministic::atan2(1.0, 2.0), pi / 2, 1));
    // Signed zeros (C99 Annex F).
    EXPECT(isPositiveZero(deterministic::atan2(0.0, 1.0)));
    EXPECT(isNegativeZero(deterministic::atan2(-0.0, 1.0)));
    EXPECT(isPositiveZero(deterministic::atan2(0.0, 0.0)));
    EXPECT(isNegativeZero(deterministic::atan2(-0.0, 0.0)));
    EXPECT(deterministic::atan2(0.0, -0.0) == pi);
    EXPECT(deterministic::atan2(-0.0, -0.0) == -pi);
    EXPECT(deterministic::atan2(0.0, -1.0) == pi);
    EXPECT(deterministic::atan2(-0.0, -1.0) == -pi);
    EXPECT(deterministic::atan2(1.0, 0.0) == pi / 2);
    EXPECT(deterministic::atan2(1.0, -0.0) == pi / 2);
    EXPECT(deterministic::atan2(-1.0, 0.0) == -pi / 2);
    // Infinities.
    EXPECT(isPositiveZero(deterministic::atan2(1.0, infinity)));
    EXPECT(isNegativeZero(deterministic::atan2(-1.0, infinity)));
    EXPECT(deterministic::atan2(1.0, -infinity) == pi);
    EXPECT(deterministic::atan2(-1.0, -infinity) == -pi);
    EXPECT(deterministic::atan2(infinity, 1.0) == pi / 2);
    EXPECT(deterministic::atan2(-infinity, 1.0) == -pi / 2);
    EXPECT(deterministic::atan2(infinity, infinity) * 4 == pi);
    EXPECT(deterministic::atan2(infinity, -infinity) * 4 == 3 * pi);
    EXPECT(deterministic::atan2(-infinity, -infinity) * 4 == -3 * pi);
    EXPECT(deterministic::atan2(-infinity, infinity) * 4 == -pi);
    // Ratios beyond 2^60 and below 2^-60.
    EXPECT(deterministic::atan2(1e300, 1e-300) == pi / 2);
    EXPECT(isPositiveZero(deterministic::atan2(1e-300, 1e300)));
    EXPECT(deterministic::atan2(1e-300, -1e300) == pi);
    EXPECT(isNaN(deterministic::atan2(quietNaN, 1.0)));
    EXPECT(isNaN(deterministic::atan2(1.0, quietNaN)));
    EXPECT(isNaN(deterministic::atan2(quietNaN, quietNaN)));
}

TEST(DeterministicExpAndLogSpecialValues) {
    EXPECT(deterministic::exp(0.0) == 1.0 && deterministic::exp(-0.0) == 1.0);
    EXPECT(deterministic::exp(infinity) == infinity);
    EXPECT(isPositiveZero(deterministic::exp(-infinity)));
    EXPECT(isNaN(deterministic::exp(quietNaN)));
    EXPECT(deterministic::exp(709.78271289338397) == 1.7976931348622732e308);  // the largest argument that fits
    EXPECT(deterministic::exp(709.79) == infinity);
    EXPECT(deterministic::exp(1e300) == infinity);
    EXPECT(isPositiveZero(deterministic::exp(-746.0)));
    EXPECT(deterministic::exp(-745.0) == smallestSubnormal);  // e^-745 = 0.55 of the smallest subnormal
    EXPECT(isWithinUlps(deterministic::exp(-740.0), std::exp(-740.0), 1));  // a subnormal result
    EXPECT(deterministic::exp(1e-300) == 1.0);

    EXPECT(isPositiveZero(deterministic::expm1(0.0)));
    EXPECT(isNegativeZero(deterministic::expm1(-0.0)));
    EXPECT(deterministic::expm1(infinity) == infinity);
    EXPECT(deterministic::expm1(-infinity) == -1.0);
    EXPECT(deterministic::expm1(-100.0) == -1.0);
    EXPECT(deterministic::expm1(1000.0) == infinity);
    EXPECT(isNaN(deterministic::expm1(quietNaN)));
    EXPECT(deterministic::expm1(smallestSubnormal) == smallestSubnormal);
    EXPECT(deterministic::expm1(1e-20) == 1e-20);

    EXPECT(deterministic::log(0.0) == -infinity);
    EXPECT(deterministic::log(-0.0) == -infinity);
    EXPECT(isNaN(deterministic::log(-1.0)));
    EXPECT(isNaN(deterministic::log(-infinity)));
    EXPECT(deterministic::log(infinity) == infinity);
    EXPECT(isNaN(deterministic::log(quietNaN)));
    EXPECT(isPositiveZero(deterministic::log(1.0)));
    EXPECT(isWithinUlps(deterministic::log(smallestSubnormal), std::log(smallestSubnormal), 1));  // about −744.44
    EXPECT(isWithinUlps(deterministic::log(largestDouble), std::log(largestDouble), 1));

    EXPECT(isPositiveZero(deterministic::log1p(0.0)));
    EXPECT(isNegativeZero(deterministic::log1p(-0.0)));
    EXPECT(deterministic::log1p(-1.0) == -infinity);
    EXPECT(isNaN(deterministic::log1p(-1.0000000000000002)));
    EXPECT(isNaN(deterministic::log1p(-infinity)));
    EXPECT(deterministic::log1p(infinity) == infinity);
    EXPECT(isNaN(deterministic::log1p(quietNaN)));
    EXPECT(deterministic::log1p(1e-300) == 1e-300);
    EXPECT(isWithinUlps(deterministic::log1p(largestDouble), std::log1p(largestDouble), 1));
}

TEST(DeterministicSinhSpecialValues) {
    EXPECT(isPositiveZero(deterministic::sinh(0.0)));
    EXPECT(isNegativeZero(deterministic::sinh(-0.0)));
    EXPECT(deterministic::sinh(infinity) == infinity);
    EXPECT(deterministic::sinh(-infinity) == -infinity);
    EXPECT(isNaN(deterministic::sinh(quietNaN)));
    EXPECT(deterministic::sinh(smallestSubnormal) == smallestSubnormal);
    EXPECT(isWithinUlps(deterministic::sinh(710.0), std::sinh(710.0), 2));  // between exp's limit and sinh's
    EXPECT(isWithinUlps(deterministic::sinh(710.4), std::sinh(710.4), 2));
    EXPECT(deterministic::sinh(711.0) == infinity);
    EXPECT(deterministic::sinh(-711.0) == -infinity);
}

TEST(DeterministicHypotSpecialValues) {
    EXPECT(deterministic::hypot(3.0, 4.0) == 5.0);
    EXPECT(deterministic::hypot(-3.0, 4.0) == 5.0);
    EXPECT(deterministic::hypot(5.0, 12.0) == 13.0);
    EXPECT(deterministic::hypot(2.0, 3.0, 6.0) == 7.0);
    EXPECT(deterministic::hypot(-1.0, 2.0, -2.0) == 3.0);
    EXPECT(isPositiveZero(deterministic::hypot(0.0, -0.0)));
    EXPECT(isPositiveZero(deterministic::hypot(-0.0, -0.0, 0.0)));
    EXPECT(deterministic::hypot(0.0, -7.0) == 7.0);
    EXPECT(deterministic::hypot(1.0, 0.0, 0.0) == 1.0);
    // Infinity wins over NaN.
    EXPECT(deterministic::hypot(infinity, quietNaN) == infinity);
    EXPECT(deterministic::hypot(quietNaN, -infinity) == infinity);
    EXPECT(deterministic::hypot(quietNaN, 1.0, -infinity) == infinity);
    EXPECT(deterministic::hypot(infinity, 1.0) == infinity);
    EXPECT(isNaN(deterministic::hypot(quietNaN, 1.0)));
    EXPECT(isNaN(deterministic::hypot(1.0, 2.0, quietNaN)));
    // No overflow or underflow in the squares.
    EXPECT(isWithinUlps(deterministic::hypot(3e200, 4e200), 5e200, 1));
    EXPECT(isWithinUlps(deterministic::hypot(3e-200, 4e-200), 5e-200, 1));
    EXPECT(isWithinUlps(deterministic::hypot(3e-320, 4e-320), 5e-320, 1));  // subnormal inputs and output
    EXPECT(deterministic::hypot(smallestSubnormal, 0.0) == smallestSubnormal);
    EXPECT(deterministic::hypot(largestDouble, 1.0) == largestDouble);
    EXPECT(deterministic::hypot(largestDouble, largestDouble) == infinity);  // 1.414·max does not fit
    EXPECT(isWithinUlps(deterministic::hypot(1e300, 1e300, 1e300), 1.7320508075688772e300, 1));
    EXPECT(isWithinUlps(deterministic::hypot(1e-310, 2e-310, 2e-310), 3e-310, 1));
}

// ---------------------------------------------------------------------------------------------
// Values worked out by hand: the correctly rounded results (500-digit arithmetic).
// ---------------------------------------------------------------------------------------------

TEST(DeterministicKnownValues) {
    EXPECT(deterministic::sin(1.0) == 0.8414709848078965);
    EXPECT(deterministic::cos(1.0) == 0.5403023058681398);
    EXPECT(deterministic::tan(1.0) == 1.5574077246549023);
    EXPECT(deterministic::log(2.0) == 0.6931471805599453);
    EXPECT(deterministic::log(10.0) == 2.302585092994046);
    EXPECT(deterministic::exp(10.0) == 22026.465794806718);
    EXPECT(deterministic::sinh(1.0) == 1.1752011936438014);
    EXPECT(deterministic::expm1(0.1) == 0.10517091807564763);
    EXPECT(deterministic::log1p(0.1) == 0.09531017980432487);
    // fdlibm's exp(1) is 2.7182818284590455, one ulp above the correctly rounded e: a known
    // property of the algorithm, and within its 1 ulp promise.
    EXPECT(isWithinUlps(deterministic::exp(1.0), 2.718281828459045, 1));
    EXPECT(isWithinUlps(deterministic::asin(0.5), pi / 6, 1));
    EXPECT(isWithinUlps(deterministic::acos(0.5), pi / 3, 1));
    EXPECT(isWithinUlps(deterministic::atan(2.0), 1.1071487177940904, 1));
    // π from four times the arctangent of 1, and from atan2.
    EXPECT(deterministic::atan(1.0) * 4 == pi);
    EXPECT(deterministic::atan2(1.0, 1.0) * 4 == pi);
    EXPECT(deterministic::acos(-1.0) == pi);
    // The argument π/2 as a double is not exactly π/2: cos of it is the gap, 6.123233995736766e-17.
    EXPECT(deterministic::cos(pi / 2) == 6.123233995736766e-17);
    EXPECT(deterministic::sin(pi) == 1.2246467991473532e-16);
    EXPECT(deterministic::sin(pi / 6) == 0.49999999999999994);
}

TEST(DeterministicIdentities) {
    Stream stream(7);
    for (int index = 0; index < 2000; index += 1) {
        const double angle = stream.between(-100.0, 100.0);
        const double sine = deterministic::sin(angle);
        const double cosine = deterministic::cos(angle);
        EXPECT(std::fabs(sine * sine + cosine * cosine - 1.0) < 4e-16);
        EXPECT(deterministic::sin(-angle) == -sine);   // odd
        EXPECT(deterministic::cos(-angle) == cosine);  // even
        EXPECT(deterministic::atan(deterministic::tan(angle / 70.0)) - angle / 70.0 < 1e-15);
        const double unit = stream.between(-1.0, 1.0);
        EXPECT(deterministic::asin(-unit) == -deterministic::asin(unit));
        EXPECT(std::fabs(deterministic::asin(unit) + deterministic::acos(unit) - pi / 2) < 1e-15);
        const double positive = stream.between(0.001, 1000.0);
        // log has a relative error of 1.1e-16 of its result, and exp magnifies an absolute error.
        EXPECT(std::fabs(deterministic::exp(deterministic::log(positive)) / positive - 1.0) < (std::fabs(deterministic::log(positive)) + 1.0) * 2.5e-16);
        const double small = stream.between(-0.5, 0.5);
        EXPECT(std::fabs(deterministic::expm1(small) - (deterministic::exp(small) - 1.0)) < 2e-16);
        EXPECT(deterministic::sinh(-small) == -deterministic::sinh(small));
        const double first = stream.decades(-5, 5);
        const double second = stream.decades(-5, 5);
        EXPECT(deterministic::hypot(first, second) == deterministic::hypot(second, first));
        EXPECT(deterministic::hypot(first, second) == deterministic::hypot(-first, -second));
        EXPECT(deterministic::hypot(first, second, 0.0) == deterministic::hypot(first, second));
        // The angle of (x, y) from atan2, sent back through cos and sin, gives x and y over the length.
        const double angleOfPoint = deterministic::atan2(second, first);
        const double length = deterministic::hypot(first, second);
        EXPECT(std::fabs(length * deterministic::cos(angleOfPoint) - first) <= 1e-15 * length);
        EXPECT(std::fabs(length * deterministic::sin(angleOfPoint) - second) <= 1e-15 * length);
    }
}

TEST(DeterministicHypotScalesExactlyByPowersOfTwo) {
    // With no rounding in the scaling, hypot(s·x, s·y, s·z) is s·hypot(x, y, z) for s = 2ᵏ,
    // from the subnormal range to just below overflow.
    Stream stream(21);
    for (int index = 0; index < 2000; index += 1) {
        const double x = stream.between(-1.0, 1.0) + 2.0;
        const double y = stream.between(-1.0, 1.0);
        const double z = stream.between(-1.0, 1.0);
        const int exponent = static_cast<int>(stream.between(-1000.0, 1000.0));
        const double scale = std::ldexp(1.0, exponent);
        EXPECT(deterministic::hypot(x * scale, y * scale) == deterministic::hypot(x, y) * scale);
        EXPECT(deterministic::hypot(x * scale, y * scale, z * scale) == deterministic::hypot(x, y, z) * scale);
    }
}

TEST(DeterministicNaNsAreAlwaysTheSameNaN) {
    const uint64_t canonical = 0x7ff8000000000000ULL;
    const double negativeNaN = std::bit_cast<double>(0xfff8000000000001ULL);
    EXPECT(bitsOf(deterministic::sin(infinity)) == canonical);
    EXPECT(bitsOf(deterministic::sin(negativeNaN)) == canonical);
    EXPECT(bitsOf(deterministic::asin(2.0)) == canonical);
    EXPECT(bitsOf(deterministic::log(-1.0)) == canonical);
    EXPECT(bitsOf(deterministic::log1p(-2.0)) == canonical);
    EXPECT(bitsOf(deterministic::atan2(negativeNaN, 1.0)) == canonical);
    EXPECT(bitsOf(deterministic::exp(negativeNaN)) == canonical);
    EXPECT(bitsOf(deterministic::sinh(negativeNaN)) == canonical);
    EXPECT(bitsOf(deterministic::hypot(negativeNaN, 1.0)) == canonical);
}

// ---------------------------------------------------------------------------------------------
// Agreement with the system's libm over 100,000 inputs per function.
// ---------------------------------------------------------------------------------------------

TEST(DeterministicSweepSinCosTan) {
    sweep("sin", 1, 1, [](int index, Stream& stream) { return std::make_tuple(angleInput(index, stream)); },
        [](double x) { return deterministic::sin(x); }, [](double x) { return std::sin(x); });
    sweep("cos", 2, 1, [](int index, Stream& stream) { return std::make_tuple(angleInput(index, stream)); },
        [](double x) { return deterministic::cos(x); }, [](double x) { return std::cos(x); });
    // Apple's tan is itself up to 2 ulp off for some arguments (for -8.133981743885883 it gives
    // 3.4776023055332717; the true value, 3.4776023055332708, is what fdlibm returns), so the
    // allowance against it is 3.
    sweep("tan", 3, 3, [](int index, Stream& stream) { return std::make_tuple(angleInput(index, stream)); },
        [](double x) { return deterministic::tan(x); }, [](double x) { return std::tan(x); });
}

TEST(DeterministicSweepInverseTrigonometry) {
    auto unitInput = [](int index, Stream& stream) {
        switch (index % 4) {
            case 0: return std::make_tuple(stream.between(-1.0, 1.0));
            case 1: return std::make_tuple(1.0 - std::fabs(stream.decades(-16, 0)));  // near 1
            case 2: return std::make_tuple(-1.0 + std::fabs(stream.decades(-16, 0)));  // near −1
            default: return std::make_tuple(stream.decades(-12, 0));  // near 0
        }
    };
    sweep("asin", 4, 1, unitInput, [](double x) { return deterministic::asin(x); }, [](double x) { return std::asin(x); });
    sweep("acos", 5, 1, unitInput, [](double x) { return deterministic::acos(x); }, [](double x) { return std::acos(x); });
    sweep("atan", 6, 1,
        [](int index, Stream& stream) {
            return std::make_tuple(index % 2 == 0 ? stream.between(-5.0, 5.0) : stream.decades(-10, 20));
        },
        [](double x) { return deterministic::atan(x); }, [](double x) { return std::atan(x); });
    sweep("atan2", 7, 2,
        [](int index, Stream& stream) {
            if (index % 2 == 0) return std::make_tuple(stream.between(-10.0, 10.0), stream.between(-10.0, 10.0));
            return std::make_tuple(stream.decades(-30, 30), stream.decades(-30, 30));
        },
        [](double y, double x) { return deterministic::atan2(y, x); }, [](double y, double x) { return std::atan2(y, x); });
}

TEST(DeterministicSweepExponentialsAndLogarithms) {
    sweep("exp", 8, 1,
        [](int index, Stream& stream) {
            switch (index % 4) {
                case 0: return std::make_tuple(stream.between(-745.0, 709.7));
                case 1: return std::make_tuple(stream.between(-2.0, 2.0));
                case 2: return std::make_tuple(stream.between(-0.7, 0.7));  // around the reduction boundaries ln2/2
                default: return std::make_tuple(stream.decades(-20, 1));
            }
        },
        [](double x) { return deterministic::exp(x); }, [](double x) { return std::exp(x); });
    sweep("expm1", 9, 1,
        [](int index, Stream& stream) {
            switch (index % 4) {
                case 0: return std::make_tuple(stream.between(-40.0, 40.0));
                case 1: return std::make_tuple(stream.between(-1.0, 1.0));
                case 2: return std::make_tuple(stream.between(-709.0, 709.0));
                default: return std::make_tuple(stream.decades(-20, 0));
            }
        },
        [](double x) { return deterministic::expm1(x); }, [](double x) { return std::expm1(x); });
    sweep("log", 10, 1,
        [](int index, Stream& stream) {
            switch (index % 4) {
                case 0: return std::make_tuple(std::fabs(stream.decades(-300, 300)));
                case 1: return std::make_tuple(1.0 + stream.between(-0.01, 0.01));
                case 2: return std::make_tuple(std::fabs(stream.decades(-323, -300)));  // subnormal
                default: return std::make_tuple(stream.between(0.0, 100.0));
            }
        },
        [](double x) { return deterministic::log(x); }, [](double x) { return std::log(x); });
    sweep("log1p", 11, 1,
        [](int index, Stream& stream) {
            switch (index % 4) {
                case 0: return std::make_tuple(-1.0 + std::fabs(stream.decades(-15, 0)));
                case 1: return std::make_tuple(stream.between(-0.5, 10.0));
                case 2: return std::make_tuple(stream.decades(-20, 0));
                default: return std::make_tuple(std::fabs(stream.decades(0, 300)));
            }
        },
        [](double x) { return deterministic::log1p(x); }, [](double x) { return std::log1p(x); });
    sweep("sinh", 12, 2,
        [](int index, Stream& stream) {
            switch (index % 4) {
                case 0: return std::make_tuple(stream.between(-710.4, 710.4));
                case 1: return std::make_tuple(stream.between(-30.0, 30.0));
                case 2: return std::make_tuple(stream.between(-1.0, 1.0));
                default: return std::make_tuple(stream.decades(-20, 0));
            }
        },
        [](double x) { return deterministic::sinh(x); }, [](double x) { return std::sinh(x); });
}

TEST(DeterministicSweepHypot) {
    sweep("hypot(x, y)", 13, 1,
        [](int index, Stream& stream) {
            if (index % 2 == 0) return std::make_tuple(stream.between(-100.0, 100.0), stream.between(-100.0, 100.0));
            return std::make_tuple(stream.decades(-300, 300), stream.decades(-300, 300));
        },
        [](double x, double y) { return deterministic::hypot(x, y); }, [](double x, double y) { return std::hypot(x, y); });
    sweep("hypot(x, y, z)", 14, 1,
        [](int index, Stream& stream) {
            if (index % 2 == 0) return std::make_tuple(stream.between(-100.0, 100.0), stream.between(-100.0, 100.0), stream.between(-100.0, 100.0));
            const double scale = std::fabs(stream.decades(-100, 100));  // beyond that the system's own x*x+y*y+z*z overflows or underflows
            return std::make_tuple(scale * stream.between(-1.0, 1.0), scale * stream.between(-1.0, 1.0), scale * stream.between(-1.0, 1.0));
        },
        [](double x, double y, double z) { return deterministic::hypot(x, y, z); }, [](double x, double y, double z) { return std::hypot(x, y, z); });
}
