// A skeleton is a list of bones, parents before children (Engine/Skeleton/Skeleton.swift and
// Skeleton+Building.swift). Forward kinematics turns a pose into the world position of every
// bone in one pass over that list.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "Bone.h"
#include "Pose.h"

namespace editor {

/// A hierarchy of bones with a rest pose and forward kinematics.
///
/// Bones are stored parent-before-child, so any pass over the array in index order sees a parent's
/// result before its children need it. Conventions are those of `Bone`: Z up, right-handed, metres,
/// a bone's local +Z along the bone.
///
/// Use when: posing a character, or placing the parts of any jointed model
/// Tags: skeleton, rig, armature, forward kinematics, FK, bone hierarchy, joint chain, pose
/// Node: bone, mirrorBones, poseSkeleton, boneEnds
///
/// (`MirrorAxis`, the plane a skeleton is mirrored across, is declared in `Math/Transform.h`.)
struct Skeleton {
    /// The bones, parents before children.
    std::vector<Bone> bones;

    /// The most bones `problem` allows: far more than any character needs, and small enough that
    /// a skeleton from a file cannot ask for a huge array.
    static constexpr int maximumBoneCount = 1024;

    // MARK: Checking

    /// What is wrong with this skeleton, in a sentence naming the bone, or nothing if forward
    /// kinematics can use it. Rejects more than `maximumBoneCount` bones; a parent index that is
    /// not smaller than the bone's own index (also negative); a name used twice; a length that is
    /// not positive and finite; a rest translation or rotation that is not finite, a rotation
    /// that is not close to unit length, and a scale other than 1. Non-finite values are
    /// rejected explicitly because NaN fails every comparison.
    ///
    /// Use when: checking a skeleton that came from a file or an agent before posing it
    /// Tags: validate skeleton, check bones, bad rig, skeleton problem, reject invalid input
    /// Node: inside bone, mirrorBones
    std::optional<std::string> problem() const;

    // MARK: Looking up

    /// The index of the bone called `name`, or nothing if there is none.
    ///
    /// Use when: finding "forearm.L" in order to pose it
    /// Tags: bone by name, find bone, lookup bone, get bone index
    /// Node: inside bone, poseSkeleton, boneEnds
    std::optional<int> index(const std::string& name) const;

    /// The indices of the bones whose parent is `index`, in increasing order.
    ///
    /// Use when: walking down a limb; finding what a joint carries
    /// Tags: child bones, bone children, hierarchy, descendants one level
    /// Node: none (a hierarchy lookup; a graph names the bones it means)
    std::vector<int> children(int index) const;

    // MARK: Forward kinematics

    /// The world matrix of every bone (bone-frame points to world points) in `pose`.
    ///
    /// How: one pass in index order, `world[i] = world[parent] · rest[i] · R(pose[i])`, with
    /// `rest[i]` the bone's `restTransform.matrix()`, `R` the matrix of the pose's rotation for
    /// that bone, and for a root `world[parent]` replaced by a move of `pose.rootTranslation`.
    /// Each bone costs two matrix products, so a skeleton of `n` bones costs `O(n)`.
    ///
    /// Conventions: parents come first (`problem()` is empty); a parent index that is not before
    /// the bone is treated as no parent, so bad input cannot read past the array. Pose rotations
    /// must be unit.
    ///
    /// Use when: drawing a posed character; finding where a hand is
    /// Tags: forward kinematics, FK, world transforms, bone matrices, skin matrices, pose to world
    /// Node: inside boneEnds, bone
    std::vector<Matrix4> worldTransforms(const Pose& pose = Pose()) const;

    /// The world matrix of every bone at rest: `worldTransforms` of the empty pose.
    ///
    /// Use when: drawing the bind pose; measuring the rest proportions
    /// Tags: rest pose, bind pose, rest world transforms, T-pose
    /// Node: inside boneEnds
    std::vector<Matrix4> restWorldTransforms() const;

    /// Where bone `index`'s head (its joint) is in the world in `pose`. Runs a whole pass; to read
    /// many bones, call `worldTransforms` once and use `Bone::headPosition`.
    ///
    /// Use when: asking where one joint is, for example to aim a camera at a hand
    /// Tags: joint position, head position, bone position, world position
    /// Node: boneEnds
    Vector3 headPosition(int index, const Pose& pose = Pose()) const;

    /// Where bone `index`'s tail is in the world in `pose`: the head plus `length` along the
    /// bone's local +Z, turned to the world.
    ///
    /// Use when: asking where a fingertip or the top of the head is
    /// Tags: tail position, bone end, end effector position, world position
    /// Node: boneEnds
    Vector3 tailPosition(int index, const Pose& pose = Pose()) const;

    // MARK: Mirroring

    /// The name with its side swapped: a trailing `.L` becomes `.R` and the other way round
    /// (Blender's convention). Any other name is returned unchanged.
    ///
    /// Use when: naming the twin of a bone on the other side
    /// Tags: mirror name, left right name, .L .R, symmetric bone name, flip side
    /// Node: inside mirrorBones
    static std::string mirroredName(const std::string& name);

    /// The bone as it would be in the mirror: its rest transform reflected and its name's side
    /// swapped. The parent index and length are kept.
    ///
    /// How: the rest transform becomes `M · rest · M`, where `M` is the reflection that negates
    /// the coordinate of `axis` (`Transform::mirrored`). The bone's +Z (along the bone) maps to
    /// its mirror image; its side axis +X flips, so a mirrored roll stays mirrored.
    ///
    /// Conventions: this is exact only for a parent frame that is itself symmetric (`M P M = P`):
    /// the centre-line bones, or the already-mirrored parent. Negation is exact in floating
    /// point, so mirroring twice gives the original bone bit for bit.
    ///
    /// Use when: building the right arm from the left one
    /// Tags: mirror bone, reflect bone, symmetric twin, flip bone, left to right
    /// Node: inside mirrorBones
    static Bone mirroredBone(const Bone& bone, MirrorAxis axis);

    /// The whole skeleton reflected across `axis`: every bone mirrored (`mirroredBone`) in the same
    /// place of the array, so the parents stay valid. Left bones become right bones and the other
    /// way round. Mirroring twice gives the original.
    ///
    /// Use when: checking that a rig is symmetric; flipping a pose or a rig
    /// Tags: mirror skeleton, reflect rig, flip skeleton, symmetric copy
    /// Node: mirrorBones
    Skeleton mirrored(MirrorAxis axis) const;

    /// This skeleton plus a mirrored twin of every `.L` bone, appended after the existing bones.
    ///
    /// How: bones are visited in index order; the twin of a `.L` bone is `mirroredBone`, with the
    /// parent replaced by the parent's twin when the parent is a `.L` bone and kept otherwise (a
    /// centre-line parent has no twin). Because a parent is visited before its children the
    /// twin's parent index is already known, and stays smaller than the twin's own index. A
    /// recipe therefore describes the centre line and one side only.
    ///
    /// Conventions: the centre-line bones must be symmetric across `axis` (a translation with
    /// zero component along `axis`, a rotation about that axis only or a half turn about an axis
    /// across it), otherwise the right side hangs from a different frame than the left.
    ///
    /// Use when: finishing a character after its left side has been placed
    /// Tags: add mirrored bones, symmetrize, build right side, copy left to right
    /// Node: mirrorBones
    Skeleton addingMirroredTwins(MirrorAxis axis) const;

    /// Whether every bone has a twin across `axis`: a bone of the mirrored name with the same
    /// length, the mirrored rest transform, and a parent that is the mirrored name of the
    /// parent. Exact comparison, no tolerance, except that a rotation also matches its negative
    /// (the same turn).
    ///
    /// Use when: checking that a rig is exactly symmetric before relying on it for mirrored posing
    /// Tags: is symmetric, symmetry check, left right match, rig symmetry
    /// Node: none (a check for tests and validation, not a modelling step)
    bool isSymmetric(MirrorAxis axis) const;

    // MARK: Building (Skeleton+Building.swift)

    /// How bone `index`'s frame is turned in the world at rest: its rest rotation after those of
    /// all its parents.
    ///
    /// How: `q_world[i] = q_world[parent] · q_rest[i]`, root first, the rotation part of
    /// `world[i] = world[parent] · rest[i]` (bones are rigid, so a world matrix is only a move
    /// and this rotation).
    ///
    /// Conventions: a parent index that is not before the bone counts as no parent, as in
    /// `worldTransforms`. The result is unit when every rest rotation is.
    ///
    /// Use when: turning a world direction into a bone's own frame; placing a child bone by world positions
    /// Tags: world rotation, rest rotation, bone orientation, accumulated rotation, global rotation
    /// Node: inside bone
    Quaternion restWorldRotation(int index) const;

    /// This skeleton with one more bone at the end, placed by its head and tail in the world at
    /// rest and turned `roll` about itself.
    ///
    /// How, with `d = tail − head` and the parent's world rotation `P` and head `h_p` (identity
    /// and the origin for a root):
    ///
    ///     world rotation  W = rotate(d, roll) · rotation(from: +Z, to: d)
    ///     rest rotation     = P⁻¹ · W
    ///     rest translation  = P⁻¹ · (head − h_p)
    ///     length            = |d|
    ///
    /// The shortest turn from +Z onto the bone's direction comes first, then the roll about the
    /// bone; `P⁻¹` (the conjugate) carries both from the world into the parent's frame. So with
    /// `roll` 0 a bone pointing up keeps +X as its side axis, and any other bone's side axis is
    /// +X carried along by that shortest turn (a bone along +X has its side axis pointing down).
    ///
    /// Conventions: `parent` is an index already in the skeleton, or nothing for a root; `head`
    /// and `tail` are in metres, Z up; the roll follows the right-hand rule about head → tail. A
    /// head equal to the tail has no direction: the bone is added unturned with length 0, which
    /// `problem` reports.
    ///
    /// Use when: laying out a rig from joint positions; adding a bone between two landmarks
    /// Tags: add bone, place bone, bone from head and tail, edit bone, extrude bone, armature
    /// Node: bone
    Skeleton addingBone(const std::string& name, std::optional<int> parent, Vector3 head, Vector3 tail, Angle roll) const;

    /// The skeleton with `pose` made its rest pose: at rest it stands exactly as this skeleton
    /// stands in `pose`.
    ///
    /// How: every bone's rest rotation becomes `q_rest[i] · q_pose[i]`, and every root's rest
    /// translation moves by `pose.rootTranslation`. Then `world[i] = world[p] · rest'[i]` equals
    /// `world[p] · rest[i] · R(pose[i])` of `worldTransforms`, bone by bone.
    ///
    /// Conventions: pose rotations must be unit; a parent index that is not before the bone
    /// counts as a root, as in `worldTransforms`.
    ///
    /// Use when: bending a rig into a posture before building muscles or a body on it; posing in several steps, each on the result of the last
    /// Tags: apply pose, bake pose, pose as rest, set rest pose, posed skeleton
    /// Node: poseSkeleton
    Skeleton posed(const Pose& pose) const;

    /// How far from 1 the length of a rest rotation may be. A unit quaternion written as text and
    /// read back is off by about 1e-7; this leaves room for that and for hand-rounded values.
    static constexpr float unitLengthTolerance = 1e-3f;

    bool operator==(const Skeleton&) const = default;
};

} // namespace editor
