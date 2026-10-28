// The exact share of each pixel a filled shape covers: the reference rasterizer (stage R0 of
// Marble's Docs/Plans/Text-Rasterizer.md). Slow and obviously right, so every faster glyph
// rasterizer (the CPU accumulator, the GPU per-pixel coverage) is checked against it pixel by
// pixel.
#pragma once

#include <vector>

#include "Shape2D.h"

namespace editor {

/// A grid of pixel coverages: for each unit pixel square, the share of it a shape fills, from 0
/// (empty) to 1 (full).
///
///     row 1  ┌─────┬─────┐       pixel (column, row) is the square
///            │ 0,1 │ 1,1 │       [column, column + 1] × [row, row + 1]
///     row 0  ├─────┼─────┤       of the shape's own plane
///            │ 0,0 │ 1,0 │
///            └─────┴─────┘
///          (0,0)
///
/// Conventions: y-up, like the shape: row 0 is the bottom row. `coverages` holds the rows one
/// after another, bottom first, each from left to right.
///
/// Use when: holding a rendered glyph's alpha; comparing two rasterizers pixel by pixel
/// Tags: coverage image, alpha mask, rendered glyph, antialiased mask, raster, bitmap
/// Node: none (text rendering reference, not a modelling step)
struct CoverageImage {
    int columns = 0;
    int rows = 0;
    /// The coverage of every pixel, rows bottom first, each row left to right.
    std::vector<float> coverages;

    /// The coverage of pixel (`column`, `row`), 0…1. Throws a `MathProblem` outside the grid.
    ///
    /// Use when: reading one pixel's alpha; comparing one pixel of two renders
    /// Tags: pixel value, alpha at pixel, coverage at pixel
    /// Node: none (text rendering reference)
    float coverage(int column, int row) const;

    /// The sum of every pixel's coverage: the filled area inside the grid, in pixels.
    ///
    /// Use when: checking a render against the shape's area; measuring how much ink a glyph uses
    /// Tags: total coverage, ink area, sum of alpha, filled area
    /// Node: none (text rendering reference)
    float totalCoverage() const;
};

CoverageImage exactCoverage(const Shape2D& shape, int columns, int rows, float flatteningTolerance);

} // namespace editor
