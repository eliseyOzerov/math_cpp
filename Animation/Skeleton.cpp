// Checking, forward kinematics, mirroring and building a skeleton (Engine/Skeleton/Skeleton.swift
// and Skeleton+Building.swift). Every loop runs in Swift's order, so the matrices agree bit for bit.

#include "Skeleton.h"

#include <cmath>
#include <map>
#include <set>

#include "../Linear/NumberText.h"
#include "../Linear/ParameterProblem.h"

namespace editor {

// MARK: - Checking

std::optional<std::string> Skeleton::problem() const {
    const std::string owner = "skeleton";
    if (auto tooMany = ParameterProblem::budget(static_cast<int>(bones.size()), maximumBoneCount, "bones", owner)) {
        return tooMany;
    }
    std::set<std::string> seenNames;
    for (size_t position = 0; position < bones.size(); ++position) {
        int index = static_cast<int>(position);
        const Bone& bone = bones[position];
        std::string label = "bone " + std::to_string(index) + " (" + bone.name + ")";
        if (bone.parent && (*bone.parent < 0 || *bone.parent >= index)) {
            return owner + " needs the parent of " + label + " to be a bone before it, but it is "
                + std::to_string(*bone.parent) + ".";
        }
        if (!seenNames.insert(bone.name).second) {
            return owner + " has two bones named " + bone.name + "; names must be unique.";
        }
        if (auto lengthProblem = ParameterProblem::length(bone.length, "length of " + label, owner)) {
            return lengthProblem;
        }
        const Transform& transform = bone.restTransform;
        if (!transform.translation.isFinite()) {
            return owner + " needs the rest translation of " + label + " to be finite, but it is "
                + vectorText(transform.translation) + ".";
        }
        float rotationLength = transform.rotation.length();
        if (!(std::isfinite(rotationLength) && std::fabs(rotationLength - 1) <= unitLengthTolerance)) {
            return owner + " needs the rest rotation of " + label + " to be a unit quaternion, but its length is "
                + numberText(rotationLength) + ".";
        }
        if (!(transform.scale == Vector3(1, 1, 1))) {
            return owner + " needs the rest scale of " + label + " to be 1, but it is " + vectorText(transform.scale) + ".";
        }
    }
    return std::nullopt;
}

// MARK: - Looking up

std::optional<int> Skeleton::index(const std::string& name) const {
    for (size_t position = 0; position < bones.size(); ++position) {
        if (bones[position].name == name) return static_cast<int>(position);
    }
    return std::nullopt;
}

std::vector<int> Skeleton::children(int index) const {
    std::vector<int> result;
    for (size_t position = 0; position < bones.size(); ++position) {
        if (bones[position].parent == index) result.push_back(static_cast<int>(position));
    }
    return result;
}

// MARK: - Forward kinematics

std::vector<Matrix4> Skeleton::worldTransforms(const Pose& pose) const {
    std::vector<Matrix4> worlds;
    worlds.reserve(bones.size());
    for (size_t position = 0; position < bones.size(); ++position) {
        int index = static_cast<int>(position);
        const Bone& bone = bones[position];
        // A parent that is not before the bone counts as no parent, so bad input cannot read past
        // the array.
        Matrix4 parentWorld = (bone.parent && *bone.parent >= 0 && *bone.parent < index)
            ? worlds[*bone.parent]
            : Matrix4::translation(pose.rootTranslation);
        worlds.push_back(parentWorld * bone.restTransform.matrix() * pose.rotation(index).matrix());
    }
    return worlds;
}

std::vector<Matrix4> Skeleton::restWorldTransforms() const { return worldTransforms(); }

Vector3 Skeleton::headPosition(int index, const Pose& pose) const {
    return bones[index].headPosition(worldTransforms(pose)[index]);
}

Vector3 Skeleton::tailPosition(int index, const Pose& pose) const {
    return bones[index].tailPosition(worldTransforms(pose)[index]);
}

// MARK: - Mirroring

namespace {

bool hasSuffix(const std::string& name, const std::string& suffix) {
    return name.size() >= suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace

std::string Skeleton::mirroredName(const std::string& name) {
    if (hasSuffix(name, ".L")) return name.substr(0, name.size() - 2) + ".R";
    if (hasSuffix(name, ".R")) return name.substr(0, name.size() - 2) + ".L";
    return name;
}

Bone Skeleton::mirroredBone(const Bone& bone, MirrorAxis axis) {
    Bone twin = bone;
    twin.name = mirroredName(bone.name);
    twin.restTransform = bone.restTransform.mirrored(axis);
    return twin;
}

Skeleton Skeleton::mirrored(MirrorAxis axis) const {
    Skeleton result;
    for (const Bone& bone : bones) result.bones.push_back(mirroredBone(bone, axis));
    return result;
}

Skeleton Skeleton::addingMirroredTwins(MirrorAxis axis) const {
    std::vector<Bone> result = bones;
    std::map<int, int> twinIndex;
    for (size_t position = 0; position < bones.size(); ++position) {
        const Bone& bone = bones[position];
        if (!hasSuffix(bone.name, ".L")) continue;
        Bone twin = mirroredBone(bone, axis);
        if (bone.parent) {
            auto parentTwin = twinIndex.find(*bone.parent);
            if (parentTwin != twinIndex.end()) twin.parent = parentTwin->second;
        }
        twinIndex[static_cast<int>(position)] = static_cast<int>(result.size());
        result.push_back(twin);
    }
    return Skeleton{result};
}

bool Skeleton::isSymmetric(MirrorAxis axis) const {
    for (const Bone& bone : bones) {
        Bone expected = mirroredBone(bone, axis);
        std::optional<int> twinIndex = index(expected.name);
        if (!twinIndex) return false;
        const Bone& twin = bones[*twinIndex];
        std::optional<std::string> parentName;
        if (bone.parent) parentName = mirroredName(bones[*bone.parent].name);
        std::optional<std::string> twinParentName;
        if (twin.parent) twinParentName = bones[*twin.parent].name;
        // `q` and `−q` are the same turn: a half turn mirrors onto its own negative.
        bool sameRotation = twin.restTransform.rotation == expected.restTransform.rotation
            || twin.restTransform.rotation == -expected.restTransform.rotation;
        if (!(twin.length == expected.length && twin.restTransform.translation == expected.restTransform.translation
              && sameRotation && twinParentName == parentName)) {
            return false;
        }
    }
    return true;
}

// MARK: - Building

Quaternion Skeleton::restWorldRotation(int index) const {
    Quaternion rotation = bones[index].restTransform.rotation;
    int current = index;
    while (bones[current].parent && *bones[current].parent >= 0 && *bones[current].parent < current) {
        int parent = *bones[current].parent;
        rotation = bones[parent].restTransform.rotation * rotation;
        current = parent;
    }
    return rotation;
}

Skeleton Skeleton::addingBone(const std::string& name, std::optional<int> parent, Vector3 head, Vector3 tail, Angle roll) const {
    Vector3 direction = tail - head;
    // A bone with no direction cannot be turned onto it: keep it unturned, and its length of 0
    // (or NaN) is what `problem` reports.
    Quaternion worldRotation = Quaternion::identity();
    if (std::optional<Vector3> axis = direction.unitDirection()) {
        worldRotation = Quaternion::fromAxis(*axis, roll) * Quaternion::rotation(Vector3::zAxis(), *axis);
    }
    Quaternion parentRotation = Quaternion::identity();
    Vector3 parentHead = Vector3(0, 0, 0);
    if (parent) {
        parentRotation = restWorldRotation(*parent);
        parentHead = headPosition(*parent);
    }
    Quaternion toParent = parentRotation.conjugate();
    Bone bone(name, parent, toParent.rotate(head - parentHead), (toParent * worldRotation).normalized(), direction.length());
    Skeleton result{bones};
    result.bones.push_back(bone);
    return result;
}

Skeleton Skeleton::posed(const Pose& pose) const {
    std::vector<Bone> result = bones;
    for (size_t position = 0; position < result.size(); ++position) {
        int index = static_cast<int>(position);
        result[position].restTransform.rotation =
            (result[position].restTransform.rotation * pose.rotation(index)).normalized();
        bool isRoot = result[position].parent ? (*result[position].parent < 0 || *result[position].parent >= index) : true;
        if (isRoot) result[position].restTransform.translation += pose.rootTranslation;
    }
    return Skeleton{result};
}

} // namespace editor
