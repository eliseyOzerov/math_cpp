// A source of repeatable random numbers (Engine/Math/Numerics/SeededRandom.swift).
#include "SeededRandom.h"

#include <cmath>

#include "../Linear/GeometryProblem.h"
#include "../Linear/Scalar.h"

namespace editor {

std::uint64_t SeededRandom::nextUInt64() {
    state += 0x9E3779B97F4A7C15ull;
    std::uint64_t mixed = state;
    mixed = (mixed ^ (mixed >> 30)) * 0xBF58476D1CE4E5B9ull;
    mixed = (mixed ^ (mixed >> 27)) * 0x94D049BB133111EBull;
    return mixed ^ (mixed >> 31);
}

float SeededRandom::nextUnitFloat() { return static_cast<float>(nextUInt64() >> 40) * unitFloatStep; }

float SeededRandom::nextFloat(float lowerBound, float upperBound) {
    return linearBlend(lowerBound, upperBound, nextUnitFloat());
}

Angle SeededRandom::nextAngle(Angle start, Angle end) {
    return Angle{linearBlend(start.radians, end.radians, nextUnitFloat())};
}

std::int64_t SeededRandom::nextInt(std::int64_t lowerBound, std::int64_t upperBound) {
    if (!(lowerBound < upperBound)) throw GeometryProblem("A random integer needs a non-empty range.");
    // The bit pattern of the wrapped difference is the true width, which is below 2⁶⁴.
    std::uint64_t valueCount = static_cast<std::uint64_t>(upperBound) - static_cast<std::uint64_t>(lowerBound);
    auto multiply = [valueCount](std::uint64_t random) {
        return static_cast<unsigned __int128>(random) * valueCount;
    };
    unsigned __int128 product = multiply(nextUInt64());
    if (static_cast<std::uint64_t>(product) < valueCount) {
        std::uint64_t rejectionLimit = (0 - valueCount) % valueCount; // 2⁶⁴ mod valueCount
        while (static_cast<std::uint64_t>(product) < rejectionLimit) product = multiply(nextUInt64());
    }
    std::uint64_t high = static_cast<std::uint64_t>(product >> 64);
    return static_cast<std::int64_t>(static_cast<std::uint64_t>(lowerBound) + high);
}

Vector3 SeededRandom::nextUnitVector3() {
    float height = nextFloat(-1, 1);
    Angle heading = nextAngle(Angle{}, Angle::degrees(360));
    float ringRadius = std::sqrt(1 - height * height);
    return Vector3(ringRadius * heading.cosine(), ringRadius * heading.sine(), height);
}

} // namespace editor
