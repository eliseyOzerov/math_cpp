// A pose of a skeleton (Engine/Skeleton/Pose.swift).
#pragma once

#include <map>

#include "../Linear/Quaternion.h"

namespace editor {

/// A pose of a skeleton: how far each bone is turned away from its rest orientation, and
/// optionally where the whole figure is moved.
///
/// A bone's rotation is applied in the bone's **own local frame**, on top of its rest rotation (see
/// `Skeleton::worldTransforms`). A bone with no entry keeps its rest orientation. Rotations must be
/// unit quaternions.
///
/// Use when: bending a character's joints; a frame of an animation
/// Tags: pose, joint angles, bone rotations, animation frame, FK pose, skeleton pose
/// Node: inside poseSkeleton
struct Pose {
    /// The extra rotation of the bones that are turned, by bone index.
    std::map<int, Quaternion> rotations;
    /// How far every root bone's head is moved from where the rest pose puts it, in the world.
    Vector3 rootTranslation = Vector3(0, 0, 0);

    /// The extra rotation of bone `index`, the identity if it is not turned.
    ///
    /// Use when: reading one joint of a pose
    /// Tags: bone rotation, joint rotation, pose lookup
    /// Node: inside poseSkeleton
    Quaternion rotation(int index) const {
        auto found = rotations.find(index);
        return found == rotations.end() ? Quaternion::identity() : found->second;
    }

    bool operator==(const Pose&) const = default;
};

} // namespace editor
