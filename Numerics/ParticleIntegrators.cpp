// Particle integrators (Engine/Math/Physics/ParticleIntegrators.swift).
#include "ParticleIntegrators.h"

namespace editor {

float ParticleState::kineticEnergy(float mass) const { return 0.5f * mass * velocity.dot(velocity); }

ParticleState ParticleState::steppedBySemiImplicitEuler(
    float timeStep, const std::function<Vector3(Vector3, Vector3)>& acceleration) const {
    // `!(timeStep <= 0)` rather than `timeStep > 0`, so a NaN step is not mistaken for "no time".
    if (timeStep <= 0) return *this;
    Vector3 newVelocity = velocity + acceleration(position, velocity) * timeStep;
    return ParticleState(position + newVelocity * timeStep, newVelocity);
}

ParticleState ParticleState::steppedBySemiImplicitEuler(float timeStep, const std::function<Vector3(Vector3)>& acceleration) const {
    return steppedBySemiImplicitEuler(timeStep, [&acceleration](Vector3 position, Vector3) { return acceleration(position); });
}

ParticleState ParticleState::steppedByVelocityVerlet(
    float timeStep, const std::function<Vector3(Vector3, Vector3)>& acceleration) const {
    if (timeStep <= 0) return *this;
    Vector3 oldAcceleration = acceleration(position, velocity);
    Vector3 newPosition = position + velocity * timeStep + oldAcceleration * (0.5f * timeStep * timeStep);
    Vector3 predictedVelocity = velocity + oldAcceleration * timeStep;
    Vector3 newAcceleration = acceleration(newPosition, predictedVelocity);
    Vector3 newVelocity = velocity + (oldAcceleration + newAcceleration) * (0.5f * timeStep);
    return ParticleState(newPosition, newVelocity);
}

ParticleState ParticleState::steppedByVelocityVerlet(float timeStep, const std::function<Vector3(Vector3)>& acceleration) const {
    return steppedByVelocityVerlet(timeStep, [&acceleration](Vector3 position, Vector3) { return acceleration(position); });
}

} // namespace editor
