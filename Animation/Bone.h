// One bone of a skeleton (Engine/Skeleton/Bone.swift).
#pragma once

#include <optional>
#include <string>

#include "../Linear/Transform.h"

namespace editor {

/// One bone of a skeleton: a straight stick from a **head** (the joint it turns about) to a
/// **tail** (where the next bone's joint usually sits), placed relative to its parent bone.
///
/// The bone's **local frame** has its origin at the head, its **+Z axis along the bone** toward the
/// tail, and its +X axis as the "side" axis (the bone's roll):
///
///            tail  ● ─ ─ ─ ─ ┐        +Z points from head to tail,
///                  ▲  length │        the tail is `(0, 0, length)` in the bone's own frame
///                  │  +Z     │
///                  │     +X  │        +X is the side axis: the direction the bone "faces",
///            head  ●────▶    ┘        which fixes what a twist about the bone means
///
/// `restTransform` places this frame inside the parent's frame (or in the world for a root):
/// a translation (the head's position) and a rotation. Its scale must stay 1, a bone is rigid.
/// The skeleton's conventions are the engine's: Z up, right-handed, metres.
///
/// Use when: describing one joint and the bone after it, in a character or any jointed rig
/// Tags: bone, joint, rig, armature, skeleton bone, local frame, bind pose, Blender bone, rest pose
/// Node: bone
struct Bone {
    /// The bone's name, unique inside its skeleton. A name ending in `.L` or `.R` marks the left or
    /// right side (Blender's convention).
    std::string name;
    /// The index of the parent bone in the skeleton, or nothing for a root. Always smaller than
    /// this bone's own index, so parents come first.
    std::optional<int> parent;
    /// Where the bone's head is and how its frame is turned, relative to the parent's frame (the
    /// world for a root), when the skeleton is at rest. Scale must be 1.
    Transform restTransform;
    /// The distance from head to tail, in metres. Positive.
    float length = 0;

    /// A bone with the given placement. `rotation` defaults to no turn.
    Bone(std::string name, std::optional<int> parent, Vector3 translation, Quaternion rotation, float length)
        : name(std::move(name)), parent(parent), restTransform{.translation = translation, .rotation = rotation},
          length(length) {}

    /// The tail in the bone's own frame: `length` along +Z.
    ///
    /// Use when: finding where a bone ends once its world transform is known
    /// Tags: bone tail, local tail, bone end, bone vector
    /// Node: inside boneEnds
    Vector3 localTail() const { return Vector3(0, 0, length); }

    /// Where the tail is in the world, given the bone's world matrix (head frame to world).
    ///
    /// How: `world · (0, 0, length)`.
    ///
    /// Use when: reading a bone's end from the result of forward kinematics
    /// Tags: tail position, bone end point, world tail
    /// Node: inside boneEnds
    Vector3 tailPosition(const Matrix4& world) const { return world.transformPoint(localTail()); }

    /// Where the head is in the world, given the bone's world matrix: its translation column.
    ///
    /// How: `world · (0, 0, 0)`.
    ///
    /// Use when: reading a bone's joint position from the result of forward kinematics
    /// Tags: head position, joint position, world head
    /// Node: inside boneEnds
    Vector3 headPosition(const Matrix4& world) const { return world.transformPoint(Vector3(0, 0, 0)); }

    bool operator==(const Bone&) const = default;
};

} // namespace editor
