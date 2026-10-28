// The arithmetic of the shader library's image effects, ported from graphics_metal's Effects.metal:
// the blur weights and sample paths, sharpen, the Sobel edge, emboss, chromatic aberration,
// vignette, film grain, the CRT warp and scanlines, bloom, pixelate, mosaic, displacement, the
// normal from a height, the haze and lens warps and Kuwahara's choice of quadrant.
//
// What stays in Metal and what is here. An effect reads a texture around a point (`imageTap`), which
// has no meaning on the CPU. Each effect is split in two: the "taps" (the texture reads, with the
// loops that make them) stay in the Metal backend, and everything that is arithmetic on a uv or on
// values already read is a function here. Each function's comment names the Metal function it came
// from and the taps its caller makes. `vignetteColor`, `filmGrain`, `hazeUv` and `lensUv` read
// nothing and are here whole.
//
// Name map from graphics_metal (each function's Tags line repeats its old name):
//   blurImage → blurTapOffset, blurTapWeight, blurTapFraction, radialBlurPoint, zoomBlurPoint,
//   sharpenImage → sharpenedColor, sobelOf → sobelGradient, edgeStrength → edgeStrengthOf,
//   embossImage → embossOffset, embossColor, chromaticAberration → chromaticOffset, chromaticColor,
//   vignetteColor → vignetteColor, filmGrain → filmGrain,
//   scanlinesCrt → crtCentredWarp, crtWarpedUv, isOutsideUnitSquare, crtScanlineFactor, crtMask,
//     crtCornerDarkening, crtShadedColor,
//   bloomImage → bloomRingOffset, bloomRingWeight, bloomBrightPart, bloomCombined,
//   pixelateImage → pixelateTapUv, mosaicImage → mosaicTileId, mosaicTileCentreUv, mosaicTileColor,
//   displaceImage → displacementFromMap, luminanceSlope, displacedUv,
//   normalFromHeight → luminanceSlope, normalFromHeightColor,
//   hazeUv → hazeUv, lensUv → lensUv, lensImage → lensUv + isOutsideUnitSquare,
//   kuwahara → kuwaharaRadius, kuwaharaQuadrantSign, kuwaharaQuadrantStatistics,
//     kuwaharaBestQuadrant, kuwaharaSelectedMean.
//
// Stays in the Metal backend: `imageTap` and `texelOf` (texture reads), every `blurImage` / `bloomImage`
// / `kuwahara` loop over taps, and the `luminance` of each tap (the caller reads a tap, takes its
// `luminance` and passes the numbers here).
//
// Differences from graphics_metal, all on purpose: `mix`, `smoothstep`, `clamp`, `fract`, `floor`
// and the others are the prelude's names (`linearBlend`, `safeSmoothStep`, `clamped`,
// `fractionalPart`, `roundedDown`), `rotation2` is `rotatedCounterClockwise`, `toRadians` is
// `degreesToRadians`, the constants are `tau<float>()`, and a smooth step with equal edges is a
// hard step, not undefined. graphics_metal's bloom has a cutoff and no soft knee; so does this.
#pragma once

#include "Hashes.h"
#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

/// The offset, in units of the blur radius, of one tap of the 7 by 7 box or Gaussian blur.
///
/// From: `offset` in `blurImage` (modes 0 and 1). Taps: the caller reads uv + offset × radius × texel
/// for x and y each in −3…3, adds weight × colour and divides by the sum of the weights.
///
/// How: (x, y) / 3, so the grid reaches ±1 radius.
///
/// Use when: laying out the taps of a box or Gaussian blur
/// Tags: blurImage, blur offset, kernel grid, 7x7 taps
/// Node: inside blur
inline Vector2Of<float> blurTapOffset(int x, int y) {
    return Vector2Of<float>(static_cast<float>(x), static_cast<float>(y)) / 3.0f;
}

/// The weight of one tap of a box (mode 0) or Gaussian (any other mode) blur.
///
/// From: `weight` in `blurImage`. Taps: none.
///
/// How: box 1; Gaussian exp(−2 |offset|²), a bell with σ = 0.5 in units of the radius (so the edge
/// tap at |offset| = 1 has weight e⁻² ≈ 0.135).
///
/// Conventions: `offset` from `blurTapOffset`, so within ±1 on each axis. The caller divides by the
/// sum of the weights it used.
///
/// Use when: weighting the taps of a blur
/// Tags: blurImage, gaussian weight, box blur, bell curve
/// Node: inside blur
inline float blurTapWeight(Vector2Of<float> offset, int mode) {
    return mode == 0 ? 1.0f : exponential(-dotProduct(offset, offset) * 2.0f);
}

/// How far along a radial or zoom blur's path the tap `tap` of 16 is: −0.5 to +0.5.
///
/// From: `t` in `blurImage` (modes 2 and 3). Taps: 16, averaged with equal weights.
///
/// How: tap / 15 − 0.5.
///
/// Use when: spreading a radial or zoom blur's 16 taps symmetrically about the pixel
/// Tags: blurImage, radial blur, zoom blur, sample fraction
/// Node: inside blur
inline float blurTapFraction(int tap) { return static_cast<float>(tap) / 15.0f - 0.5f; }

/// Where a radial (spin) blur reads for the tap at `fraction`: the pixel turned about `middle`.
///
/// From: `point` for mode 2 of `blurImage`. Taps: the caller reads the image there.
///
/// How: middle + rotate(uv − middle, fraction × radiusDegrees in radians); counter-clockwise for a
/// positive angle.
///
/// Conventions: `radiusDegrees` is the whole sweep in degrees; the taps spread ±half of it.
///
/// Use when: a spin or radial blur around a point
/// Tags: blurImage, radial blur, spin blur, rotational blur
/// Node: inside blur
inline Vector2Of<float> radialBlurPoint(Vector2Of<float> uv, Vector2Of<float> middle, float fraction, float radiusDegrees) {
    return middle + rotatedCounterClockwise(uv - middle, degreesToRadians(fraction * radiusDegrees));
}

/// Where a zoom blur reads for the tap at `fraction`: the pixel pulled toward `middle`.
///
/// From: `point` for mode 3 of `blurImage`. Taps: the caller reads the image there.
///
/// How: middle + (uv − middle) × (1 − (fraction + 0.5) × radiusPercent / 100). Fraction −0.5 reads
/// at the pixel itself, +0.5 at `radiusPercent` percent of the way to `middle`.
///
/// Use when: a speed-line or zoom blur from a point
/// Tags: blurImage, zoom blur, radial zoom, speed lines
/// Node: inside blur
inline Vector2Of<float> zoomBlurPoint(Vector2Of<float> uv, Vector2Of<float> middle, float fraction, float radiusPercent) {
    return middle + (uv - middle) * (1.0f - (fraction + 0.5f) * radiusPercent * 0.01f);
}

/// A sharpened pixel: the pixel pushed away from the average of its four neighbours.
///
/// From: `sharpenImage`. Taps: the centre, and uv ± (texel.x, 0) and uv ± (0, texel.y), summed into
/// `aroundSum`.
///
/// How: unsharp mask: rgb = centre + (centre − ¼ aroundSum) × amount; alpha is the centre's.
///
/// Conventions: `amount` 0 leaves the image; larger is crisper and can overshoot 0…1.
///
/// Use when: sharpening an image
/// Tags: sharpenImage, sharpen, unsharp mask, crisp
/// Node: inside sharpen
inline Vector4Of<float> sharpenedColor(Vector4Of<float> centre, Vector4Of<float> aroundSum, float amount) {
    Vector3Of<float> color(centre.x, centre.y, centre.z);
    Vector3Of<float> around = Vector3Of<float>(aroundSum.x, aroundSum.y, aroundSum.z) * 0.25f;
    Vector3Of<float> result = color + (color - around) * amount;
    return Vector4Of<float>(result.x, result.y, result.z, centre.w);
}

/// The slope of brightness across a pixel from the luminance of the pixels right, left, above and
/// below it: x the change to the right, y the change upward.
///
/// From: the `dx`, `dy` of `normalFromHeight` and the `offset` of mode 1 of `displaceImage`. Taps:
/// the caller reads uv ± (texel.x, 0) and uv ± (0, texel.y) and takes each `luminance`.
///
/// How: central difference: (right − left, up − down). Not divided by the spacing.
///
/// Conventions: y grows upward (uv from the bottom left).
///
/// Use when: the gradient of a height map or of a brightness map
/// Tags: normalFromHeight, displaceImage, gradient, slope, central difference
/// Node: inside normal-from-height, displace
inline Vector2Of<float> luminanceSlope(float right, float left, float up, float down) {
    return Vector2Of<float>(right - left, up - down);
}

/// The Sobel gradient of the luminance of a pixel from its eight neighbours.
///
/// From: `sobelOf`. Taps: the eight neighbours at uv + texel × (−1…1, −1…1) without the centre, each
/// through `luminance`; named top-left (−1, +1), top, top-right, left, right, bottom-left, bottom,
/// bottom-right.
///
/// How: x = (topRight + 2 right + bottomRight) − (topLeft + 2 left + bottomLeft);
/// y = (topLeft + 2 top + topRight) − (bottomLeft + 2 bottom + bottomRight) (Sobel and Feldman, 1968).
///
/// Conventions: y grows upward; a vertical edge (dark left, light right) gives a positive x.
///
/// Use when: edge detection; the direction and size of an edge
/// Tags: sobelOf, Sobel operator, edge detection, gradient, image derivative
/// Node: inside edge-detect
inline Vector2Of<float> sobelGradient(float topLeft, float top, float topRight, float left, float right, float bottomLeft,
                                      float bottom, float bottomRight) {
    return Vector2Of<float>(topRight + 2.0f * right + bottomRight - topLeft - 2.0f * left - bottomLeft,
                            topLeft + 2.0f * top + topRight - bottomLeft - 2.0f * bottom - bottomRight);
}

/// How strong an edge is, 0 to 1, from its Sobel gradient.
///
/// From: `edgeStrength`. Taps: the eight of `sobelGradient`.
///
/// How: clamp(|gradient| × strength, 0, 1).
///
/// Use when: turning a gradient into an edge mask
/// Tags: edgeStrength, edge mask, edge magnitude
/// Node: inside edge-detect
inline float edgeStrengthOf(Vector2Of<float> gradient, float strength) {
    return clamped(lengthOf(gradient) * strength, 0.0f, 1.0f);
}

/// The offset, in uv, from a pixel to the pixel read ahead of it for emboss: one texel along the
/// light direction.
///
/// From: `direction` in `embossImage`. Taps: the caller reads uv + offset and uv − offset.
///
/// How: (cos a, sin a) × texel with a the angle in radians.
///
/// Conventions: `angleDegrees` counter-clockwise from +x.
///
/// Use when: the sampling direction of an emboss
/// Tags: embossImage, emboss direction, light angle
/// Node: inside emboss
inline Vector2Of<float> embossOffset(float angleDegrees, Vector2Of<float> texel) {
    float radians = degreesToRadians(angleDegrees);
    return Vector2Of<float>(cosine(radians), sine(radians)) * texel;
}

/// The embossed grey: mid grey lifted where the image brightens along the light direction.
///
/// From: `embossImage`. Taps: the luminance at uv + `embossOffset` (`ahead`) and at uv − it (`behind`).
///
/// How: 0.5 + (ahead − behind) × amount in all three channels; alpha 1.
///
/// Use when: an embossed relief look
/// Tags: embossImage, emboss, relief, bump look
/// Node: inside emboss
inline Vector4Of<float> embossColor(float ahead, float behind, float amount) {
    float grey = 0.5f + (ahead - behind) * amount;
    return Vector4Of<float>(grey, grey, grey, 1.0f);
}

/// The offset, in uv, by which red and blue are shifted apart: growing away from `middle`.
///
/// From: `offset` in `chromaticAberration`. Taps: the caller reads red at uv + offset and blue at
/// uv − offset.
///
/// How: (uv − middle) × amount × 0.02.
///
/// Use when: a lens colour fringe, glitch looks
/// Tags: chromaticAberration, colour fringe, RGB split, lens fringe
/// Node: inside chromatic-aberration
inline Vector2Of<float> chromaticOffset(Vector2Of<float> uv, float amount, Vector2Of<float> middle) {
    return (uv - middle) * amount * 0.02f;
}

/// The colour of a chromatic-aberration pixel: red from one tap, green and alpha from the centre,
/// blue from the opposite tap.
///
/// From: the return of `chromaticAberration`. Taps: `redTap` at uv + offset, `centre` at uv,
/// `blueTap` at uv − offset.
///
/// How: (redTap.r, centre.g, blueTap.b, centre.a).
///
/// Use when: finishing a chromatic aberration
/// Tags: chromaticAberration, RGB split
/// Node: inside chromatic-aberration
inline Vector4Of<float> chromaticColor(Vector4Of<float> redTap, Vector4Of<float> centre, Vector4Of<float> blueTap) {
    return Vector4Of<float>(redTap.x, centre.y, blueTap.z, centre.w);
}

/// A vignette: the colour darkened toward the edges of the picture.
///
/// From: `vignetteColor`. Taps: none (the caller passes the pixel's colour).
///
/// How: fade = smoothstep(radius, radius − max(softness, 0.0001), |position|), a reversed smooth step:
/// 1 where |position| ≤ radius − softness, 0 where |position| ≥ radius, an S-curve between; rgb ×
/// linearBlend(1 − intensity, 1, fade); alpha unchanged.
///
/// Conventions: `position` is centred (0 at the middle, ±1 at the edges, as the caller chooses);
/// `intensity` 0 does nothing, 1 goes black at the edge. 1 at the centre for any radius above 0.
///
/// Use when: drawing the eye to the middle; a cinematic look
/// Tags: vignetteColor, vignette, darken edges, corner shade
/// Node: inside vignette
inline Vector4Of<float> vignetteColor(Vector4Of<float> color, Vector2Of<float> position, float radius, float softness,
                                      float intensity) {
    float fade = safeSmoothStep(radius, radius - largerOf(softness, 0.0001f), lengthOf(position));
    float scale = linearBlend(1.0f - intensity, 1.0f, fade);
    return Vector4Of<float>(color.x * scale, color.y * scale, color.z * scale, color.w);
}

/// Film grain: the colour with a flickering random value added, less on bright pixels.
///
/// From: `filmGrain`. Taps: none.
///
/// How: noise = hash31((⌊pixel / max(size, 1)⌋, ⌊time × 24⌋)) − 0.5, so the grain is square blocks
/// of `size` pixels that change 24 times a second; rgb + noise × amount × (1 − 0.5 luminance); alpha
/// unchanged.
///
/// Conventions: `pixel` in pixels; `time` in seconds; noise is −0.5…0.5 so `amount` is the full swing.
///
/// Use when: a filmic texture; hiding banding
/// Tags: filmGrain, film grain, noise, grain
/// Node: inside film-grain
inline Vector4Of<float> filmGrain(Vector4Of<float> color, Vector2Of<float> pixel, float time, float amount, float size) {
    Vector2Of<float> block = roundedDown(pixel / largerOf(size, 1.0f));
    float noise = hash31(Vector3Of<float>(block.x, block.y, roundedDown(time * 24.0f))) - 0.5f;
    float grain = noise * amount * (1.0f - 0.5f * luminance(color));
    return Vector4Of<float>(color.x + grain, color.y + grain, color.z + grain, color.w);
}

/// The CRT screen's point in −1…1 after the tube's barrel curve.
///
/// From: the first lines of `scanlinesCrt`. Taps: none.
///
/// How: c = uv × 2 − 1; c × (1 + |c|² × curvature × 0.15): points far from the middle move outward.
///
/// Conventions: `curvature` 0 is flat; the middle is a fixed point.
///
/// Use when: curving a flat image like a CRT
/// Tags: scanlinesCrt, CRT curvature, barrel warp, tube
/// Node: inside crt
inline Vector2Of<float> crtCentredWarp(Vector2Of<float> uv, float curvature) {
    Vector2Of<float> centred = uv * 2.0f - 1.0f;
    return centred * (1.0f + dotProduct(centred, centred) * curvature * 0.15f);
}

/// The uv a warped CRT point reads from: (centred + 1) / 2.
///
/// From: `warped` in `scanlinesCrt`. Taps: the caller reads the image there unless
/// `isOutsideUnitSquare`.
///
/// Use when: converting the warped centred point back to uv
/// Tags: scanlinesCrt, centred to uv, remap
/// Node: inside crt
inline Vector2Of<float> crtWarpedUv(Vector2Of<float> centred) { return centred * 0.5f + 0.5f; }

/// Whether a uv lies outside the picture (a coordinate below 0 or above 1): the caller draws black.
///
/// From: the `any(warped < 0.0) || any(warped > 1.0)` test of `scanlinesCrt` and `lensImage`.
///
/// Use when: black borders where a warp pulls in nothing
/// Tags: scanlinesCrt, lensImage, outside, out of bounds, border
/// Node: inside crt, lens
inline bool isOutsideUnitSquare(Vector2Of<float> uv) { return uv.x < 0.0f || uv.y < 0.0f || uv.x > 1.0f || uv.y > 1.0f; }

/// The brightness factor of the scanlines at a pixel row.
///
/// From: `line` and its mix in `scanlinesCrt`. Taps: none.
///
/// How: line = ½ + ½ cos(2π pixelY / max(spacing, 1)), a wave with one dark line per `spacing`
/// rows; the factor is linearBlend(1, line, intensity): 1 at intensity 0, `line` at intensity 1.
///
/// Use when: scanlines, interlaced displays
/// Tags: scanlinesCrt, scanlines, raster lines, interlace
/// Node: inside crt
inline float crtScanlineFactor(float pixelY, float spacing, float intensity) {
    float line = 0.5f + 0.5f * cosine(pixelY / largerOf(spacing, 1.0f) * tau<float>());
    return linearBlend(1.0f, line, intensity);
}

/// The RGB phosphor mask at a pixel column: every third column keeps one channel at full and the
/// other two at 0.8.
///
/// From: `mask` and `phase` in `scanlinesCrt`. Taps: none.
///
/// How: phase = ⌊pixelX⌋ mod 3; channel k of the mask is 1 when phase = k, else 0.8; the result is
/// linearBlend((1, 1, 1), mask, intensity × 0.5), so a strong effect shows the stripes more.
///
/// Conventions: with `intensity` 0 the mask is white.
///
/// Use when: the red-green-blue stripes of a shadow mask
/// Tags: scanlinesCrt, RGB mask, shadow mask, aperture grille, phosphor
/// Node: inside crt
inline Vector3Of<float> crtMask(float pixelX, float intensity) {
    float phase = flooredModulo(roundedDown(pixelX), 3.0f);
    Vector3Of<float> mask(phase == 0.0f ? 1.0f : 0.8f, phase == 1.0f ? 1.0f : 0.8f, phase == 2.0f ? 1.0f : 0.8f);
    return linearBlend(Vector3Of<float>(1.0f), mask, intensity * 0.5f);
}

/// How much the CRT's corners are darkened at a centred point: 1 at the middle.
///
/// From: the last line of `scanlinesCrt`. Taps: none.
///
/// How: 1 − 0.3 × curvature × |centred|² × 0.25.
///
/// Use when: the darker corners of a curved screen
/// Tags: scanlinesCrt, corner darkening, tube shading
/// Node: inside crt
inline float crtCornerDarkening(Vector2Of<float> centred, float curvature) {
    return 1.0f - 0.3f * curvature * dotProduct(centred, centred) * 0.25f;
}

/// The colour of a CRT pixel: the image colour with scanlines, mask and darker corners.
///
/// From: the three `color.rgb *=` lines of `scanlinesCrt`. Taps: `color` is the tap at the warped
/// uv; `centred` is `crtCentredWarp`'s result; `pixel` is the pixel position in the output.
///
/// How: rgb × crtScanlineFactor(pixel.y) × crtMask(pixel.x) × crtCornerDarkening; alpha unchanged.
///
/// Use when: finishing a CRT effect
/// Tags: scanlinesCrt, CRT, scanlines and mask
/// Node: inside crt
inline Vector4Of<float> crtShadedColor(Vector4Of<float> color, Vector2Of<float> centred, Vector2Of<float> pixel,
                                       float spacing, float intensity, float curvature) {
    Vector3Of<float> rgb(color.x, color.y, color.z);
    rgb = rgb * crtScanlineFactor(pixel.y, spacing, intensity) * crtMask(pixel.x, intensity);
    rgb = rgb * crtCornerDarkening(centred, curvature);
    return Vector4Of<float>(rgb.x, rgb.y, rgb.z, color.w);
}

/// The offset, in uv, of tap `tapIndex` (0…7) on ring `ring` (1…3) of the bloom's sampling pattern.
///
/// From: `offset` in `bloomImage`. Taps: the caller reads uv + offset for 3 rings × 8 taps.
///
/// How: angle = tapIndex τ/8 + ring × 0.4 (each ring turned 0.4 rad against the last so the taps do
/// not line up); offset = (cos, sin) × ring × radius × texel / 3, so the rings are at ⅓, ⅔ and 1
/// radius.
///
/// Use when: laying out a bloom's taps
/// Tags: bloomImage, glow taps, ring sampling, bloom pattern
/// Node: inside bloom
inline Vector2Of<float> bloomRingOffset(int ring, int tapIndex, float radius, Vector2Of<float> texel) {
    float angle = static_cast<float>(tapIndex) * tau<float>() / 8.0f + static_cast<float>(ring) * 0.4f;
    return Vector2Of<float>(cosine(angle), sine(angle)) * static_cast<float>(ring) * radius * texel / 3.0f;
}

/// The weight of the taps on bloom ring `ring`: 1 / ring, so the nearer rings count more.
///
/// From: `weight` in `bloomImage`. Taps: none.
///
/// Use when: weighting a bloom's rings
/// Tags: bloomImage, ring weight, falloff
/// Node: inside bloom
inline float bloomRingWeight(int ring) { return 1.0f / static_cast<float>(ring); }

/// What of a colour is brighter than the bloom cutoff, per channel.
///
/// From: `max(color − cutoff, 0.0)` in `bloomImage`. Taps: none.
///
/// How: max(channel − cutoff, 0) for red, green and blue. There is no soft knee: the cut is sharp
/// (as in graphics_metal).
///
/// Use when: picking out the highlights that will glow
/// Tags: bloomImage, bloom threshold, bright pass, cutoff
/// Node: inside bloom
inline Vector3Of<float> bloomBrightPart(Vector3Of<float> color, float cutoff) {
    return largerOf(color - cutoff, Vector3Of<float>(0.0f));
}

/// The bloomed pixel: the centre plus the average glow of its surroundings.
///
/// From: the end of `bloomImage`. Taps: the caller sums weight × `bloomBrightPart` over the 24 ring
/// taps, adds the centre's own `bloomBrightPart` (weight 1) to `glowSum`, and sums the 24 weights
/// into `weightSum` (without the centre's 1).
///
/// How: rgb = centre + glowSum / (weightSum + 1) × intensity × 4; alpha the centre's.
///
/// Use when: finishing a bloom
/// Tags: bloomImage, glow, bloom
/// Node: inside bloom
inline Vector4Of<float> bloomCombined(Vector4Of<float> centre, Vector3Of<float> glowSum, float weightSum, float intensity) {
    Vector3Of<float> glow = glowSum / (weightSum + 1.0f) * intensity * 4.0f;
    return Vector4Of<float>(centre.x + glow.x, centre.y + glow.y, centre.z + glow.z, centre.w);
}

/// The uv at the centre of the `size`-pixel block that holds `uv`: the one place the pixelate effect
/// reads.
///
/// From: `pixelateImage`. Taps: the caller reads the image there.
///
/// How: cell = max(size, 1) / (width, height); (⌊uv / cell⌋ + 0.5) × cell.
///
/// Conventions: `pixels` is the texture's (width, height). ShaderCoordinates.h's `pixelatedUv` takes a
/// number of cells instead.
///
/// Use when: a blocky pixelated look
/// Tags: pixelateImage, pixelate, mosaic blocks, low resolution
/// Node: inside pixelate
inline Vector2Of<float> pixelateTapUv(Vector2Of<float> uv, float size, Vector2Of<float> pixels) {
    Vector2Of<float> cell = Vector2Of<float>(largerOf(size, 1.0f)) / pixels;
    return (roundedDown(uv / cell) + 0.5f) * cell;
}

/// The whole-number tile a uv is in on the mosaic grid: `tiles` along the short side.
///
/// From: `id` in `mosaicImage`. Taps: none.
///
/// How: ⌊uv × aspectScale × tiles⌋.
///
/// Conventions: `aspectScale` stretches the grid so tiles are square on a wide picture.
///
/// Use when: finding which mosaic tile a pixel belongs to
/// Tags: mosaicImage, tile id, mosaic grid
/// Node: inside mosaic
inline Vector2Of<float> mosaicTileId(Vector2Of<float> uv, Vector2Of<float> aspectScale, float tiles) {
    return roundedDown(uv * aspectScale * tiles);
}

/// The uv at the centre of mosaic tile `id`: the one place the tile's colour is read.
///
/// From: the tap point in `mosaicImage`. Taps: the caller reads the image there.
///
/// How: (id + 0.5) / (aspectScale × tiles).
///
/// Use when: sampling one colour per mosaic tile
/// Tags: mosaicImage, tile centre, mosaic tap
/// Node: inside mosaic
inline Vector2Of<float> mosaicTileCentreUv(Vector2Of<float> id, Vector2Of<float> aspectScale, float tiles) {
    return (id + 0.5f) / (aspectScale * tiles);
}

/// A mosaic tile's final colour: its sampled colour varied a little and cut off at the grout.
///
/// From: the lines after the tap in `mosaicImage`. Taps: `color` is the tap at `mosaicTileCentreUv`.
///
/// How: id and local = fract(grid) − 0.5 from grid = uv × aspectScale × tiles; rgb × (1 + (hash21(id)
/// − 0.5) × variation); the tile mask is 1 − smoothstep(0.5 − gap − 0.02, 0.5 − gap, max(|local.x|,
/// |local.y|)): 1 inside the square tile, 0 in the grout, over 0.02 of a tile; rgb × mask.
///
/// Conventions: `gap` is the grout half-width in tiles; `variation` 0 keeps all tiles the same
/// brightness.
///
/// Use when: stained glass, tiled-mosaic looks
/// Tags: mosaicImage, mosaic, tiles, grout, stained glass
/// Node: inside mosaic
inline Vector4Of<float> mosaicTileColor(Vector4Of<float> color, Vector2Of<float> uv, Vector2Of<float> aspectScale,
                                        float tiles, float gap, float variation) {
    Vector2Of<float> grid = uv * aspectScale * tiles;
    Vector2Of<float> id = roundedDown(grid);
    Vector2Of<float> local = fractionalPart(grid) - 0.5f;
    float varied = 1.0f + (hash21(id) - 0.5f) * variation;
    float inside = 1.0f - safeSmoothStep(0.5f - gap - 0.02f, 0.5f - gap, largerOf(absolute(local.x), absolute(local.y)));
    float scale = varied * inside;
    return Vector4Of<float>(color.x * scale, color.y * scale, color.z * scale, color.w);
}

/// The displacement a map's red and green ask for: −0.5…0.5 on each axis, 0 where the map is 0.5.
///
/// From: `offset` for mode 0 of `displaceImage`. Taps: the caller reads the map at uv.
///
/// How: (red, green) − 0.5.
///
/// Use when: warping an image by a displacement map
/// Tags: displaceImage, displacement map, warp by map
/// Node: inside displace
inline Vector2Of<float> displacementFromMap(Vector2Of<float> mapRedGreen) { return mapRedGreen - 0.5f; }

/// The uv a displaced pixel reads from: shifted by the offset times `amount`.
///
/// From: the last line of `displaceImage`. Taps: the caller reads the image there.
///
/// How: uv + offset × amount.
///
/// Use when: finishing a displacement
/// Tags: displaceImage, displaced uv, warp
/// Node: inside displace
inline Vector2Of<float> displacedUv(Vector2Of<float> uv, Vector2Of<float> offset, float amount) { return uv + offset * amount; }

/// The colour of a surface from its height's slope: its normal as a colour (mode 0), or the image lit
/// by a light from `lightAngleDegrees` (any other mode).
///
/// From: `normalFromHeight`. Taps: `slope` from `luminanceSlope` of the four height taps; `heightColor`
/// is the tap at uv itself (used only when lit).
///
/// How: normal = normalize(−slope.x × strength, −slope.y × strength, 1). Mode 0: normal × ½ + ½, the
/// usual blue-ish normal map, alpha 1. Otherwise light = normalize(cos a, sin a, 0.8) with a the angle in
/// radians, lit = clamp(normal · light, 0, 1) and rgb = heightColor × (0.35 + 0.9 lit), alpha 1.
///
/// Conventions: flat ground gives the normal (0, 0, 1), colour (0.5, 0.5, 1).
///
/// Use when: turning a height map into a normal map or a lit relief
/// Tags: normalFromHeight, normal map, bump map, relief lighting
/// Node: inside normal-from-height
inline Vector4Of<float> normalFromHeightColor(Vector2Of<float> slope, float strength, int mode, float lightAngleDegrees,
                                              Vector4Of<float> heightColor) {
    Vector3Of<float> normal = normalized(Vector3Of<float>(-slope.x * strength, -slope.y * strength, 1.0f));
    if (mode == 0) {
        Vector3Of<float> packedNormal = normal * 0.5f + 0.5f;
        return Vector4Of<float>(packedNormal.x, packedNormal.y, packedNormal.z, 1.0f);
    }
    float radians = degreesToRadians(lightAngleDegrees);
    Vector3Of<float> light = normalized(Vector3Of<float>(cosine(radians), sine(radians), 0.8f));
    float lit = clamped(dotProduct(normal, light), 0.0f, 1.0f);
    float scale = 0.35f + 0.9f * lit;
    return Vector4Of<float>(heightColor.x * scale, heightColor.y * scale, heightColor.z * scale, 1.0f);
}

/// Where a wobbling medium (heat haze, water) makes `uv` look from.
///
/// From: `hazeUv`. Taps: the caller reads the image at the result.
///
/// How: p = uv × scale; wobble = (sin(1.7 p.y + 2.1 time) + sin(3.3 p.y − 1.3 time + p.x),
/// cos(1.9 p.x + 1.7 time) + cos(3.1 p.x − 1.1 time + p.y)); uv + wobble × strength × 0.01.
///
/// Conventions: `time` in seconds; `strength` 0 gives `uv`; the wobble is at most 2 × strength × 0.01 on each axis.
///
/// Use when: heat shimmer, underwater wobble
/// Tags: hazeUv, heat haze, wobble, water, shimmer
/// Node: inside haze
inline Vector2Of<float> hazeUv(Vector2Of<float> uv, float strength, float scale, float time) {
    Vector2Of<float> p = uv * scale;
    Vector2Of<float> wobble(sine(p.y * 1.7f + time * 2.1f) + sine(p.y * 3.3f - time * 1.3f + p.x),
                            cosine(p.x * 1.9f + time * 1.7f) + cosine(p.x * 3.1f - time * 1.1f + p.y));
    return uv + wobble * strength * 0.01f;
}

/// Barrel (positive `strength`) or pincushion (negative) lens distortion about the middle.
///
/// From: `lensUv` (the uv under `lensImage`). Taps: the caller reads the image at the result unless
/// `isOutsideUnitSquare` (then black).
///
/// How: p = (uv − ½) × 2; p × (1 + strength |p|²); the result is p / max(zoom, 0.01) × ½ + ½. A
/// quadratic radial distortion model (Brown–Conrady, first term).
///
/// Conventions: the middle maps to itself; `zoom` 1 keeps the scale, above 1 magnifies.
///
/// Use when: fisheye, lens bulge or pinch
/// Tags: lensUv, lensImage, lens distortion, barrel, pincushion, fisheye
/// Node: inside lens
inline Vector2Of<float> lensUv(Vector2Of<float> uv, float strength, float zoom) {
    Vector2Of<float> p = (uv - 0.5f) * 2.0f;
    p = p * (1.0f + strength * dotProduct(p, p));
    return p / largerOf(zoom, 0.01f) * 0.5f + 0.5f;
}

/// Kuwahara's radius as a whole number of taps: the input rounded, kept within 1…4.
///
/// From: `radius` in `kuwahara`. Taps: the caller reads (radius + 1)² per quadrant.
///
/// How: clamp(⌊radiusInput + 0.5⌋, 1, 4).
///
/// Use when: bounding the cost of the painterly filter
/// Tags: kuwahara, radius, painterly smoothing
/// Node: inside kuwahara
inline int kuwaharaRadius(float radiusInput) {
    return static_cast<int>(clamped(roundedDown(radiusInput + 0.5f), 1.0f, 4.0f));
}

/// The direction of quadrant number `quadrant` (0…3) from the pixel: the signs of x and y.
///
/// From: `sign` in `kuwahara`. Taps: the caller reads uv + (x, y) × sign × texel for x, y in
/// 0…radius.
///
/// How: quadrant 0 (+, +), 1 (−, +), 2 (+, −), 3 (−, −): x is + for even quadrants, y is + for the
/// first two.
///
/// Use when: walking the four quadrants of a Kuwahara filter
/// Tags: kuwahara, quadrant, window sign
/// Node: inside kuwahara
inline Vector2Of<float> kuwaharaQuadrantSign(int quadrant) {
    return Vector2Of<float>(quadrant % 2 == 0 ? 1.0f : -1.0f, quadrant < 2 ? 1.0f : -1.0f);
}

/// The mean colour and brightness variance of one Kuwahara quadrant from its sums.
///
/// From: the end of the quadrant loop in `kuwahara`. Taps: the caller sums the colours of the
/// (radius + 1)² taps of the quadrant into `sum` and their squared lengths (dot(colour, colour)) into
/// `squares`.
///
/// How: count = (radius + 1)²; mean = sum / count; variance = squares / count − |mean|², the mean of
/// squares minus the square of the mean (Kuwahara et al., 1976). Returns (mean.r, mean.g, mean.b,
/// variance).
///
/// Conventions: variance is over all three channels together and can be −1e-7 by rounding.
///
/// Use when: scoring how flat a window of an image is
/// Tags: kuwahara, quadrant mean, variance, painterly
/// Node: inside kuwahara
inline Vector4Of<float> kuwaharaQuadrantStatistics(Vector3Of<float> sum, float squares, int radius) {
    float count = static_cast<float>((radius + 1) * (radius + 1));
    Vector3Of<float> mean = sum / count;
    return Vector4Of<float>(mean.x, mean.y, mean.z, squares / count - dotProduct(mean, mean));
}

/// The number (0…3) of the quadrant whose variance (the `w` of its statistics) is smallest.
///
/// From: the `best` loop in `kuwahara`. Taps: none.
///
/// How: start at 0 and take a later quadrant only if its variance is strictly smaller, so a tie keeps the
/// lowest number.
///
/// Use when: choosing the smoothest of four windows
/// Tags: kuwahara, least variance, best quadrant
/// Node: inside kuwahara
inline int kuwaharaBestQuadrant(Vector4Of<float> first, Vector4Of<float> second, Vector4Of<float> third, Vector4Of<float> fourth) {
    int best = 0;
    float bestVariance = first.w;
    if (second.w < bestVariance) { best = 1; bestVariance = second.w; }
    if (third.w < bestVariance) { best = 2; bestVariance = third.w; }
    if (fourth.w < bestVariance) { best = 3; }
    return best;
}

/// The mean colour of the least varied quadrant: the painterly pixel.
///
/// From: `means[best]` in `kuwahara`. Taps: the caller adds the alpha of the tap at uv.
///
/// How: the mean (x, y, z) of the quadrant that `kuwaharaBestQuadrant` picks.
///
/// Use when: finishing a Kuwahara filter
/// Tags: kuwahara, painterly, oil paint
/// Node: inside kuwahara
inline Vector3Of<float> kuwaharaSelectedMean(Vector4Of<float> first, Vector4Of<float> second, Vector4Of<float> third,
                                             Vector4Of<float> fourth) {
    int best = kuwaharaBestQuadrant(first, second, third, fourth);
    Vector4Of<float> picked = best == 0 ? first : (best == 1 ? second : (best == 2 ? third : fourth));
    return Vector3Of<float>(picked.x, picked.y, picked.z);
}

} // namespace kernels
} // namespace editor
