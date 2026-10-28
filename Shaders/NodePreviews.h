// Graphs that show one output of one node kind, for compiling and drawing every kind: the contact
// sheets the tests draw, and the pictures on the add dial's discs (graphics_metal's
// Graph/NodePreviews.swift). Kinds a preview needs that the catalog does not have yet (Image,
// Sample Texture, Raymarch, the SDF fills...) are skipped, so a family can be added without
// touching this file; the tables below name them for when it arrives.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "ShaderCompiler.h"

namespace editor::shaders {

/// What an `any` input is fed in a preview graph: nothing (so a number), or a vector or colour.
enum class Feeding { numbers, vectors, colors };

std::string feedingName(Feeding feeding);
const std::vector<Feeding>& allFeedings();

/// The node and output that show a preview, and the graph holding them.
///
/// Use when: compiling or drawing what one output of a kind makes
/// Tags: node preview, contact sheet, kind preview
/// Node: none (shader graph)
struct Preview {
    NodeKind kind;
    NodePort output;
    Feeding feeding = Feeding::numbers;
    ShaderGraph graph;
    std::string node = "subject";
    std::string port;

    /// "add.value", with the feeding in brackets unless it is numbers.
    std::string name() const;
    CompiledShader compile() const;
};

/// One preview per output of `kind`, with its `any` inputs fed as `feeding` and every texture
/// input fed an Image (when the catalog has one).
std::vector<Preview> previews(const NodeKind& kind, Feeding feeding = Feeding::numbers);

/// Every kind's every output, with `any` inputs fed numbers, vectors and colours.
std::vector<Preview> allPreviews();

/// The shader that pictures what `kind` makes: its first output, its inputs fed as a preview feeds
/// them, or nothing when it has no output to show or does not compile. A 2D distance is shown
/// filled, as the shape it describes, rather than as a grey ramp; a curve, whose distance is zero
/// only on the curve itself, is shown as a thick stroke. A simulation keeps its state through
/// Previous Frame and Feedback Output and shows its view, so drawing the shader frame after frame
/// runs it.
///
/// Use when: drawing the picture on a node or a menu disc
/// Tags: preview shader, node thumbnail
/// Node: none (shader graph)
std::optional<CompiledShader> previewShader(const NodeKind& kind);

} // namespace editor::shaders
