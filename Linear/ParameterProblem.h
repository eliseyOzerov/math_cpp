// Readable complaints about bad numbers (Engine/Scene/ParameterProblem.swift). A shape or recipe
// read from a file must never stop the program, so before building it checks its numbers with
// these and reports the first one that is out of range, in a sentence that names the thing and
// the parameter. The Swift `Decodable` helper at the end of that file is not ported: the core
// reads JSON with its own parser.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "Angle.h"

namespace editor {

/// Sentences that say what is wrong with one parameter, or nothing when it is fine.
///
/// Each check takes the value, the parameter's name and the name of the shape or recipe it
/// belongs to, and returns for example "tree needs height to be a positive number, but it is -2.0.".
/// Chain several with `first` so the first problem wins.
///
/// Use when: validating a primitive's or a recipe's parameters that came from a file or an agent
/// Tags: validation, parameter check, bad input, error message, range check, problem
/// Node: none (error reporting)
struct ParameterProblem {
    /// A sentence, or nothing when the check passes.
    using Problem = std::optional<std::string>;

    /// A closed range of numbers, lowerBound ≤ x ≤ upperBound (Swift `ClosedRange<Float>`).
    struct FloatRange {
        float lowerBound;
        float upperBound;

        bool contains(float value) const { return value >= lowerBound && value <= upperBound; }
    };

    /// A closed range of whole numbers (Swift `ClosedRange<Int>`).
    struct IntRange {
        int lowerBound;
        int upperBound;

        bool contains(int value) const { return value >= lowerBound && value <= upperBound; }
    };

    /// The sizes a recipe accepts for any length, in metres: 0.1 mm to 10 km. Wide enough for a
    /// grain of sand or a mountain, narrow enough that sums, squares and ratios of lengths stay far
    /// from `float` overflow and rounding to zero.
    static constexpr FloatRange lengthRange{1e-4f, 1e4f};

    /// The first sentence in `problems` that is not empty.
    ///
    /// Use when: reporting one complaint out of a list of checks
    /// Tags: first problem, first error, check list, validation order
    /// Node: none (error reporting)
    static Problem first(const std::vector<Problem>& problems);

    /// "… needs `parameter` to be a positive number …" unless `value` is positive and finite, then
    /// "… from 0.0001 to 10000.0 …" unless it is inside `lengthRange`.
    ///
    /// Use when: a recipe's size, radius, thickness or tolerance (a length in metres) comes from a file
    /// Tags: length check, size bound, metres, upper bound, overflow guard, finite
    /// Node: none (error reporting)
    static Problem length(float value, const std::string& parameter, const std::string& owner);

    /// "… needs `parameter` of at least … of `referenceName` …" unless `value` ≥ `share · reference`:
    /// a size that must not be vanishingly small next to the size it is part of.
    ///
    /// How: a wall `1e-9` thick on a 1 m mug is a valid positive number, but `1 − 1e-9` rounds to 1
    /// in `float` and the geometry built on it collapses.
    ///
    /// Use when: a small part (a wall, a rounding, a handle) is measured against the whole object
    /// Tags: relative floor, minimum share, proportion check, degenerate size, rounding guard
    /// Node: none (error reporting)
    static Problem atLeast(float value, float share, float reference, const std::string& referenceName,
                           const std::string& parameter, const std::string& owner);

    /// "… would have about `estimate` `what` …" unless `estimate` ≤ `limit`: a cap on the size of
    /// what a recipe builds, so a file cannot ask for a model that exhausts memory.
    ///
    /// Use when: several counts multiply into a mesh size (rings × sides × limbs)
    /// Tags: budget, face budget, size limit, memory guard, estimate
    /// Node: none (error reporting)
    static Problem budget(int estimate, int limit, const std::string& what, const std::string& owner);

    /// "… needs `parameter` to be a positive number …" unless `value` is finite and above 0.
    ///
    /// Use when: a size, a radius, a length or a tolerance must be greater than zero
    /// Tags: positive, greater than zero, finite, size check, radius check
    /// Node: none (error reporting)
    static Problem positive(float value, const std::string& parameter, const std::string& owner);

    /// "… needs `parameter` of at least `minimum` …" unless `value` ≥ `minimum`.
    ///
    /// Use when: a number of segments, divisions or samples has a lower limit
    /// Tags: count, minimum count, at least, segments check, integer bound
    /// Node: none (error reporting)
    static Problem count(int value, int atLeast, const std::string& parameter, const std::string& owner);

    /// "… needs `parameter` from `range.lowerBound` to `range.upperBound` …" unless `range`
    /// contains `value`.
    ///
    /// Use when: a count with both a lower and an upper limit, such as recursion depth
    /// Tags: integer range, between, bounded count, depth limit, inclusive range
    /// Node: none (error reporting)
    static Problem count(int value, IntRange range, const std::string& parameter, const std::string& owner);

    /// "… needs `parameter` from `range.lowerBound` to `range.upperBound` …" unless `value` is
    /// finite and inside the closed `range`.
    ///
    /// Use when: a fraction from 0 to 1, a ratio, a share of a length
    /// Tags: number range, between, fraction check, ratio check, finite, inclusive range
    /// Node: none (error reporting)
    static Problem number(float value, FloatRange range, const std::string& parameter, const std::string& owner);

    /// "… needs `parameter` from a° to b° …" unless `angle` is finite and inside `degrees`.
    ///
    /// Use when: a branch angle, a tilt or a spread that must stay inside limits
    /// Tags: angle range, degrees check, between angles, tilt limit
    /// Node: none (error reporting)
    static Problem angle(Angle angle, FloatRange degrees, const std::string& parameter, const std::string& owner);

    /// `message` when `condition` is false, otherwise nothing: a check between two parameters that
    /// the single-value checks cannot express ("the wall must be thinner than the radius"). The
    /// message is only built when the condition fails, like Swift's autoclosure.
    ///
    /// Use when: one parameter must be smaller than another, or a sum must fit inside a size
    /// Tags: relation check, cross-parameter validation, constraint, requirement
    /// Node: none (error reporting)
    static Problem unless(bool condition, const std::function<std::string()>& message);
};

} // namespace editor
