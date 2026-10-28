#include "ParameterProblem.h"

#include <cmath>

#include "NumberText.h"

namespace editor {

ParameterProblem::Problem ParameterProblem::first(const std::vector<Problem>& problems) {
    for (const Problem& problem : problems) {
        if (problem) return problem;
    }
    return std::nullopt;
}

ParameterProblem::Problem ParameterProblem::length(float value, const std::string& parameter, const std::string& owner) {
    Problem notPositive = positive(value, parameter, owner);
    return notPositive ? notPositive : number(value, lengthRange, parameter, owner);
}

ParameterProblem::Problem ParameterProblem::atLeast(float value, float share, float reference,
                                                    const std::string& referenceName, const std::string& parameter,
                                                    const std::string& owner) {
    if (value >= share * reference) return std::nullopt;
    return owner + " needs " + parameter + " of at least " + numberText(share) + " of " + referenceName + " ("
        + numberText(share * reference) + "), but it is " + numberText(value) + ".";
}

ParameterProblem::Problem ParameterProblem::budget(int estimate, int limit, const std::string& what, const std::string& owner) {
    if (estimate <= limit) return std::nullopt;
    return owner + " would have about " + std::to_string(estimate) + " " + what + ", but at most " + std::to_string(limit)
        + " are allowed; lower its counts.";
}

ParameterProblem::Problem ParameterProblem::positive(float value, const std::string& parameter, const std::string& owner) {
    if (std::isfinite(value) && value > 0) return std::nullopt;
    return owner + " needs " + parameter + " to be a positive number, but it is " + numberText(value) + ".";
}

ParameterProblem::Problem ParameterProblem::count(int value, int atLeast, const std::string& parameter, const std::string& owner) {
    if (value >= atLeast) return std::nullopt;
    return owner + " needs " + parameter + " of at least " + std::to_string(atLeast) + ", but it is " + std::to_string(value) + ".";
}

ParameterProblem::Problem ParameterProblem::count(int value, IntRange range, const std::string& parameter, const std::string& owner) {
    if (range.contains(value)) return std::nullopt;
    return owner + " needs " + parameter + " from " + std::to_string(range.lowerBound) + " to "
        + std::to_string(range.upperBound) + ", but it is " + std::to_string(value) + ".";
}

ParameterProblem::Problem ParameterProblem::number(float value, FloatRange range, const std::string& parameter, const std::string& owner) {
    if (std::isfinite(value) && range.contains(value)) return std::nullopt;
    return owner + " needs " + parameter + " from " + numberText(range.lowerBound) + " to " + numberText(range.upperBound)
        + ", but it is " + numberText(value) + ".";
}

ParameterProblem::Problem ParameterProblem::angle(Angle angle, FloatRange degrees, const std::string& parameter, const std::string& owner) {
    float value = angle.degrees();
    if (std::isfinite(value) && degrees.contains(value)) return std::nullopt;
    return owner + " needs " + parameter + " from " + numberText(degrees.lowerBound) + "° to " + numberText(degrees.upperBound)
        + "°, but it is " + numberText(value) + "°.";
}

ParameterProblem::Problem ParameterProblem::unless(bool condition, const std::function<std::string()>& message) {
    if (condition) return std::nullopt;
    return message();
}

} // namespace editor
