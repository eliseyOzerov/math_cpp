// Springs and the exact damped oscillator (Engine/Math/Physics/DampedSpring.swift).
#include "DampedSpring.h"

#include <cmath>

namespace editor {

Vector3 DampedSpring::forceOnFirst(
    Vector3 firstPosition, Vector3 firstVelocity, Vector3 secondPosition, Vector3 secondVelocity) const {
    Vector3 separation = firstPosition - secondPosition;
    float length = separation.length();
    // `!(length <= 0)` rather than `length > 0`, so a NaN position stays NaN instead of
    // quietly passing as "coincident".
    if (length <= 0) return Vector3(0, 0, 0);
    Vector3 direction = separation / length;
    float lengtheningSpeed = (firstVelocity - secondVelocity).dot(direction);
    return direction * -(stiffness * (length - restLength) + damping * lengtheningSpeed);
}

float DampedSpring::storedEnergy(float atLength) const {
    float stretch = atLength - restLength;
    return 0.5f * stiffness * stretch * stretch;
}

namespace {

/// How close to 1 the damping ratio must be to count as critical.
constexpr double criticalBand = 1e-9;

struct Solution {
    double position;
    double velocity;
};

double preciseNaturalFrequency(const DampedOscillator& oscillator) {
    return std::sqrt(static_cast<double>(oscillator.stiffness) / static_cast<double>(oscillator.mass));
}

double preciseDampingRatio(const DampedOscillator& oscillator) {
    return static_cast<double>(oscillator.damping)
         / (2 * std::sqrt(static_cast<double>(oscillator.stiffness) * static_cast<double>(oscillator.mass)));
}

/// Position and velocity at `time`, in `double`.
Solution solution(const DampedOscillator& oscillator, double time) {
    double start = oscillator.initialPosition, speed = oscillator.initialVelocity;
    double omega = preciseNaturalFrequency(oscillator);
    double ratio = preciseDampingRatio(oscillator);
    double decay = ratio * omega;

    if (std::fabs(ratio - 1) <= criticalBand) {
        double slope = speed + omega * start;
        double envelope = std::exp(-omega * time);
        return {envelope * (start + slope * time), envelope * (slope - omega * (start + slope * time))};
    }
    if (ratio < 1) {
        double ringing = omega * std::sqrt(1 - ratio * ratio);
        double sineWeight = (speed + decay * start) / ringing;
        double envelope = std::exp(-decay * time);
        double cosine = std::cos(ringing * time), sine = std::sin(ringing * time);
        return {envelope * (start * cosine + sineWeight * sine),
                envelope * (speed * cosine - (ringing * start + decay * sineWeight) * sine)};
    }
    double spread = omega * std::sqrt(ratio * ratio - 1);
    double fastRoot = -decay + spread, slowRoot = -decay - spread;
    double fastWeight = (speed - slowRoot * start) / (fastRoot - slowRoot);
    double slowWeight = (fastRoot * start - speed) / (fastRoot - slowRoot);
    double fastTerm = fastWeight * std::exp(fastRoot * time), slowTerm = slowWeight * std::exp(slowRoot * time);
    return {fastTerm + slowTerm, fastRoot * fastTerm + slowRoot * slowTerm};
}

} // namespace

float DampedOscillator::naturalFrequency() const { return static_cast<float>(preciseNaturalFrequency(*this)); }

float DampedOscillator::dampingRatio() const { return static_cast<float>(preciseDampingRatio(*this)); }

DampingRegime DampedOscillator::regime() const {
    double ratio = preciseDampingRatio(*this);
    if (std::fabs(ratio - 1) <= criticalBand) return DampingRegime::criticallyDamped;
    return ratio < 1 ? DampingRegime::underdamped : DampingRegime::overdamped;
}

float DampedOscillator::positionAt(float time) const {
    return static_cast<float>(solution(*this, static_cast<double>(time)).position);
}

float DampedOscillator::velocityAt(float time) const {
    return static_cast<float>(solution(*this, static_cast<double>(time)).velocity);
}

float DampedOscillator::energyAt(float time) const {
    Solution state = solution(*this, static_cast<double>(time));
    return static_cast<float>(0.5 * static_cast<double>(mass) * state.velocity * state.velocity
                              + 0.5 * static_cast<double>(stiffness) * state.position * state.position);
}

} // namespace editor
