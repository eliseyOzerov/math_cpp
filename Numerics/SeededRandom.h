// A source of repeatable random numbers (Engine/Math/Numerics/SeededRandom.swift).
#pragma once

#include <cstdint>

#include "../Linear/Angle.h"
#include "../Linear/Vector3.h"

namespace editor {

/// A source of repeatable random numbers: give it the same **seed** and it produces the same
/// sequence on every run, and the same sequence as the Swift engine's `SeededRandom`.
///
/// The generator is **SplitMix64** (Steele, Lea & Flood, *Fast splittable pseudorandom number
/// generators*, OOPSLA 2014). It keeps one 64-bit number, the state. Each call adds a fixed odd
/// constant to the state (the "golden gamma", 2⁶⁴ divided by the golden ratio) and then scrambles
/// a copy of it so that its bits look unrelated to the previous output:
///
///     state  = state + 0x9E3779B97F4A7C15              (wrapping, modulo 2⁶⁴)
///     z      = state
///     z      = (z ^ (z >> 30)) × 0xBF58476D1CE4E5B9
///     z      = (z ^ (z >> 27)) × 0x94D049BB133111EB
///     output = z ^ (z >> 31)
///
/// Because the state is one number and all arithmetic is exact integer arithmetic that wraps
/// around, `nextUInt64()` does not depend on the processor or compiler. `nextUnitFloat()`,
/// `nextFloat`, `nextAngle` and `nextInt` add only exact conversions and IEEE-754 basic
/// arithmetic. `nextUnitVector3()` also calls `sin`, `cos` and the square root; the system maths
/// library does not promise correctly rounded `sin` and `cos`.
///
/// Conventions: a `SeededRandom` is a value: copying it copies the position in the sequence, and
/// both copies then produce the same numbers. Any seed is allowed, including 0. Not for
/// cryptography.
///
/// Use when: any recipe that needs randomness (branch angles, rock bumps, scattered leaves) but must give the same model for the same seed
/// Tags: random, RNG, pseudo-random, seed, SplitMix64, repeatable randomness, Random, System.Random, rand()
/// Node: inside random
class SeededRandom {
public:
    /// The generator whose first output is determined by `seed`.
    explicit SeededRandom(std::uint64_t seed) : state(seed) {}

    bool operator==(const SeededRandom&) const = default;

    /// The next 64 random bits, uniformly distributed over all 2⁶⁴ values.
    ///
    /// How: SplitMix64 as described on the type. Reference outputs for seed 0 are
    /// 0xE220A8397B1DCDAF, 0x6E789E6AA1B965F4, 0x06C45D188009454F.
    ///
    /// Use when: you need raw random bits, or are building another kind of random number
    /// Tags: random integer, random bits, next UInt64, SplitMix64, raw random
    /// Node: inside random
    std::uint64_t nextUInt64();

    /// A random number from 0 up to but **not including** 1, every multiple of 2⁻²⁴ equally likely.
    ///
    /// How: float(nextUInt64() >> 40) / 16 777 216. A float holds 24 significant bits, so the top
    /// 24 bits of the output are converted exactly and the division by 2²⁴ is exact too. The
    /// largest result is 1 − 2⁻²⁴, so 1 itself never occurs.
    ///
    /// Use when: a random fraction, a probability test (< 0.3 is true 30% of the time)
    /// Tags: random float, uniform random, unit interval, random 0 to 1, Random.value, rand()
    /// Node: inside random
    float nextUnitFloat();

    /// A random number between `lowerBound` and `upperBound`, evenly spread (Swift
    /// `nextFloat(in:)` with a closed range).
    ///
    /// How: lower + (upper − lower) × nextUnitFloat(). The unit float is below 1, so a result
    /// equal to `upper` can only appear through rounding in the multiplication.
    ///
    /// Conventions: both ends finite and `upper − lower` must not overflow; otherwise the result
    /// may be infinite or NaN.
    ///
    /// Use when: a random length, scale or offset between two limits
    /// Tags: random range, random between, uniform float in range, Random.Range, lerp random
    /// Node: random
    float nextFloat(float lowerBound, float upperBound);

    /// A random angle between `start` and `end`, evenly spread. The two may come in either
    /// order: from 180° to 90° gives an angle between 90° and 180°.
    ///
    /// How: start + (end − start) × nextUnitFloat() on the radians, the same blend as `nextFloat`.
    ///
    /// Use when: a random branch tilt or leaf rotation; `nextAngle(Angle{}, Angle::degrees(360))` for a random heading around an axis
    /// Tags: random angle, random rotation, random heading, random degrees, Random.Range angle
    /// Node: inside random
    Angle nextAngle(Angle start, Angle end);

    /// A random whole number in `lowerBound ..< upperBound` (Swift `nextInt(in:)` with a
    /// half-open range), every value equally likely. An empty range throws a `GeometryProblem`
    /// (Swift traps).
    ///
    /// Method: Lemire's multiply-and-shift (Lemire 2019, *Fast random integer generation in an
    /// interval*). With `count` values in the range, multiply a random 64-bit number x by `count`
    /// as a 128-bit product; the high 64 bits are a number in 0 ..< count. Draws whose low half is
    /// below 2⁶⁴ mod count are thrown away and x is drawn again, which leaves every result exactly
    /// equally likely.
    ///
    /// Conventions: the range may span all of `std::int64_t`: the count of values is the unsigned
    /// difference of the bounds, and the offset is added back with wrapping arithmetic.
    ///
    /// Use when: picking one of N things (a branch count, an entry of a list)
    /// Tags: random integer, random int in range, unbiased, no modulo bias, Random.Range int
    /// Node: random
    std::int64_t nextInt(std::int64_t lowerBound, std::int64_t upperBound);

    /// A random direction: a point on the unit sphere, every area of the sphere equally likely.
    ///
    /// Method: Archimedes' hat-box theorem says the height z of a uniformly random point on the
    /// sphere is itself uniform on −1 … 1. So pick z uniformly, pick the heading φ around the Z
    /// axis uniformly, and put the point on the circle of radius √(1 − z²) at that height:
    ///
    ///     x = √(1 − z²) · cos φ      y = √(1 − z²) · sin φ      z
    ///
    /// Conventions: the result has length 1 (to rounding). Z-up.
    ///
    /// Use when: a random growth direction, a random axis, scattering points over a sphere
    /// Tags: random direction, random unit vector, uniform on sphere, sphere point picking, Random.onUnitSphere
    /// Node: random
    Vector3 nextUnitVector3();

private:
    std::uint64_t state;

    /// The gap between neighbouring results of `nextUnitFloat()`: 2⁻²⁴.
    static constexpr float unitFloatStep = 1.0f / 16777216.0f;
};

} // namespace editor
