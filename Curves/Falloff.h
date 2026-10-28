// How strongly an edit reaches a node at a given distance from its centre: the falloff shapes of
// proportional editing, named as Blender names them (graphics_cpp curves `FalloffName`).
#pragma once

#include <cmath>

#include "../Linear/Scalar.h"

namespace editor {

/// A falloff shape: how an influence fades from full strength at the centre to nothing at the
/// edge of its reach.
///
/// Use when: moving or smoothing nodes with a soft selection; fading an effect with distance
/// Tags: falloff, proportional editing, soft selection, influence curve, Blender falloff
/// Node: planned (falloff)
enum class Falloff {
    /// Smooth S-shaped fade, level at both ends: 1 − smoothStep(share).
    smooth,
    /// Straight line down: 1 − share.
    linear,
    /// Bulges out like a dome (a circle's quarter): √(1 − share²).
    sphere,
    /// Drops fast at first, then levels: √(1 − share).
    root,
    /// Drops fast at the centre: (1 − share)².
    sharp,
    /// No fade: 1 everywhere inside the reach.
    constant,
};

/// The weight (1 = full strength, 0 = none) of `falloff` at a `share` of the reach: 0 at the
/// centre, 1 at the edge.
///
/// How: `share` is held in 0…1, then the formula of each shape is applied (see `Falloff`):
///
///     1 │●╲╲╲╲        smooth, linear, sphere, root and sharp all start at 1
///       │   ╲ ╲╲      and fall to 0 at share = 1 (constant stays at 1)
///     0 │─────●──
///       0    share   1
///
/// Conventions: the share is distance / reach, so callers decide what lies outside the reach (a
/// weight of 0). A NaN share gives NaN, except for `constant`.
///
/// Source: Blender's proportional-editing falloff names; formulas from graphics_cpp curves
/// selection.cpp, with the share held in 0…1 (the original left it unheld).
///
/// Use when: weighting how far a soft edit spreads from the picked nodes
/// Tags: falloff weight, influence, proportional edit curve, smooth, sphere, root, sharp, linear
/// Node: planned (falloff)
inline float falloffWeight(Falloff falloff, float share) {
    float held = clamped(share, 0, 1);
    switch (falloff) {
    case Falloff::smooth: return 1 - smoothStep(held);
    case Falloff::linear: return 1 - held;
    case Falloff::sphere: return std::sqrt(largerOf(0, 1 - held * held));
    case Falloff::root: return std::sqrt(largerOf(0, 1 - held));
    case Falloff::sharp: return (1 - held) * (1 - held);
    case Falloff::constant: return 1;
    }
    return 1;
}

} // namespace editor
