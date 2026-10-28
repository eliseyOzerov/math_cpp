// Booleans of filled regions: union, intersection, difference, exclusion and cut, on outlines
// made of lines and cubic curves. Ported from graphics_cpp's boolean.hpp and bool.hpp (Curves2D).
// The operations are declared by the slice that ports them (Boolean2D.cpp).
#pragma once

#include "Shape2D.h"

namespace editor {

/// Which boolean to take of two regions A and B.
///
///     unite     A ∪ B       everything either covers
///     intersect A ∩ B       only where both cover
///     subtract  A − B       A with B cut away
///     exclude   A ⊕ B       where exactly one covers
///     cut       A split along B's outline, every piece kept
///
/// Use when: combining two outlines into one; cutting a window out of a wall profile
/// Tags: boolean, union, intersection, difference, xor, exclusion, path ops, CSG 2D, pathfinder
/// Node: planned (booleanShapes)
enum class BooleanOperation { unite, intersect, subtract, exclude, cut };

} // namespace editor
