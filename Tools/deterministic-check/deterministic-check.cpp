// Cross-platform check of the deterministic tier (Numerics/DeterministicMath.h).
//
// For every function it feeds a fixed, seeded list of 200,000 inputs and prints one FNV-1a 64
// hash of all the result bit patterns. Build it for each platform (README.md), run it, and the
// printed lines must be identical everywhere; a different hash means a different bit somewhere.
//
// The inputs are made with integer arithmetic and exact floating-point steps only (no library
// math), so every platform sees the same inputs.
#include <bit>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "../../Numerics/DeterministicMath.h"

namespace deterministic = editor::deterministic;

namespace {

constexpr int inputCount = 200000;
constexpr double pi = 3.14159265358979323846;

/// A seeded stream of numbers: splitmix64.
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

    /// Uniform in [0, 1), exactly: 53 random bits times 2⁻⁵³.
    double unit() { return static_cast<double>(next() >> 11) * 0x1p-53; }

    double between(double low, double high) { return low + (high - low) * unit(); }

    /// A random sign times a random mantissa in [1, 2) times 2^e for e uniform in [low, high]
    /// (normal numbers only: -1022 <= low <= high <= 1023).
    double scaled(int lowExponent, int highExponent) {
        const uint64_t bits = next();
        const uint64_t exponent = static_cast<uint64_t>(1023 + lowExponent + static_cast<int>((bits >> 52) % static_cast<uint64_t>(highExponent - lowExponent + 1)));
        return std::bit_cast<double>((bits & 0x800fffffffffffffULL) | (exponent << 52));
    }

    /// Any finite double, from the raw bits.
    double anyFinite() {
        for (;;) {
            const double value = std::bit_cast<double>(next());
            if (value - value == 0.0) return value;
        }
    }

    double positive(double value) { return value < 0 ? -value : value; }

private:
    uint64_t state;
};

/// The values every function must also handle, fed first.
constexpr double specials[] = {
    0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, 1e22, 0x1p1000, -0x1p1000, 1e300, -1e300,
    std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
    std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min(),
    -std::numeric_limits<double>::denorm_min(), std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(),
    709.782712893384, 709.79, -745.13, -746.0, 710.4758600739439, 1.5707963267948966, 3.141592653589793,
};
constexpr int specialCount = sizeof(specials) / sizeof(specials[0]);

/// Folds the bits of each result into an FNV-1a 64 hash.
class Hash {
public:
    void add(double value) {
        const uint64_t bits = std::bit_cast<uint64_t>(value);
        for (int byte = 0; byte < 8; byte += 1) {
            state ^= (bits >> (8 * byte)) & 0xff;
            state *= 0x100000001b3ULL;
        }
    }
    uint64_t value() const { return state; }

private:
    uint64_t state = 0xcbf29ce484222325ULL;
};

void report(const char* name, const Hash& hash) {
    std::printf("%-8s %d inputs  fnv1a64 %016llx\n", name, inputCount + specialCount, static_cast<unsigned long long>(hash.value()));
}

/// The argument of sin, cos or tan for input number `index`.
double angle(int index, Stream& stream) {
    switch (index % 5) {
        case 0: return stream.between(-20.0, 20.0);
        case 1: return static_cast<double>(static_cast<int>(stream.between(-2000.0, 2000.0))) * (pi / 4) + stream.between(-1e-9, 1e-9);
        case 2: return stream.scaled(-30, 40);  // up to 2⁴¹, about 2e12
        case 3: return stream.anyFinite();      // includes arguments up to the largest double
        default: return stream.between(-1e6, 1e6);
    }
}

/// The argument of asin or acos.
double unitInterval(int index, Stream& stream) {
    switch (index % 4) {
        case 0: return stream.between(-1.0, 1.0);
        case 1: return 1.0 - stream.positive(stream.scaled(-55, -1));
        case 2: return -1.0 + stream.positive(stream.scaled(-55, -1));
        default: return stream.scaled(-60, -1);
    }
}

double exponentArgument(int index, Stream& stream) {
    switch (index % 4) {
        case 0: return stream.between(-745.5, 710.5);
        case 1: return stream.between(-2.0, 2.0);
        case 2: return stream.between(-40.0, 40.0);
        default: return stream.scaled(-60, 3);
    }
}

double logArgument(int index, Stream& stream) {
    switch (index % 4) {
        case 0: return stream.positive(stream.anyFinite());
        case 1: return 1.0 + stream.between(-0.01, 0.01);
        case 2: return stream.between(0.0, 100.0);
        default: return stream.positive(stream.scaled(-1022, 1023));
    }
}

double log1pArgument(int index, Stream& stream) {
    switch (index % 4) {
        case 0: return -1.0 + stream.positive(stream.scaled(-52, -1));
        case 1: return stream.between(-0.5, 10.0);
        case 2: return stream.scaled(-60, -1);
        default: return stream.positive(stream.scaled(0, 1023));
    }
}

/// Hashes `function` over the specials and then `inputCount` inputs from `input`.
template <typename Input, typename Function>
void check(const char* name, uint64_t seed, Input input, Function function) {
    Stream stream(seed);
    Hash hash;
    for (double special : specials) hash.add(function(special));
    for (int index = 0; index < inputCount; index += 1) hash.add(function(input(index, stream)));
    report(name, hash);
}

}  // namespace

int main() {
    check("sin", 1, angle, [](double x) { return deterministic::sin(x); });
    check("cos", 2, angle, [](double x) { return deterministic::cos(x); });
    check("tan", 3, angle, [](double x) { return deterministic::tan(x); });
    check("asin", 4, unitInterval, [](double x) { return deterministic::asin(x); });
    check("acos", 5, unitInterval, [](double x) { return deterministic::acos(x); });
    check("atan", 6, [](int index, Stream& stream) { return index % 2 == 0 ? stream.between(-5.0, 5.0) : stream.scaled(-60, 80); },
        [](double x) { return deterministic::atan(x); });
    check("exp", 8, exponentArgument, [](double x) { return deterministic::exp(x); });
    check("expm1", 9, exponentArgument, [](double x) { return deterministic::expm1(x); });
    check("log", 10, logArgument, [](double x) { return deterministic::log(x); });
    check("log1p", 11, log1pArgument, [](double x) { return deterministic::log1p(x); });
    check("sinh", 12, exponentArgument, [](double x) { return deterministic::sinh(x); });

    // atan2 and hypot take pairs and triples: the specials against each other, then random ones.
    {
        Stream stream(7);
        Hash hash;
        for (double first : specials) for (double second : specials) hash.add(deterministic::atan2(first, second));
        for (int index = 0; index < inputCount; index += 1) {
            const double y = index % 2 == 0 ? stream.between(-10.0, 10.0) : stream.scaled(-1000, 1000);
            const double x = index % 2 == 0 ? stream.between(-10.0, 10.0) : stream.scaled(-1000, 1000);
            hash.add(deterministic::atan2(y, x));
        }
        report("atan2", hash);
    }
    {
        Stream stream(13);
        Hash hash;
        for (double first : specials) for (double second : specials) hash.add(deterministic::hypot(first, second));
        for (int index = 0; index < inputCount; index += 1) {
            hash.add(deterministic::hypot(index % 2 == 0 ? stream.between(-100.0, 100.0) : stream.scaled(-1022, 1023), index % 2 == 0 ? stream.between(-100.0, 100.0) : stream.scaled(-1022, 1023)));
        }
        report("hypot2", hash);
    }
    {
        Stream stream(14);
        Hash hash;
        for (double first : specials) for (double second : specials) hash.add(deterministic::hypot(first, second, first - second));
        for (int index = 0; index < inputCount; index += 1) {
            const bool nearOne = index % 2 == 0;
            const double x = nearOne ? stream.between(-100.0, 100.0) : stream.scaled(-1022, 1023);
            const double y = nearOne ? stream.between(-100.0, 100.0) : stream.scaled(-1022, 1023);
            const double z = nearOne ? stream.between(-100.0, 100.0) : stream.scaled(-1022, 1023);
            hash.add(deterministic::hypot(x, y, z));
        }
        report("hypot3", hash);
    }
    return 0;
}
