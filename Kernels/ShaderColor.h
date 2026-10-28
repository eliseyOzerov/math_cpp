// The shader library's colour functions, ported from graphics_metal's Color.metal: conversions
// between sRGB, HSV, HSL, OKLab, OKLCH and OKHSV, and colour adjustments (hue, saturation,
// vibrance, levels, tone curve, posterize, colorize, white balance, split tone), a four-stop
// ramp and a cosine palette. Every function is written once for a number type `Real` (`float`) and
// the kernel vectors of it (KernelVectors.h), and runs on the CPU and in a Metal shader alike.
//
// Differences from graphics_metal, all on purpose:
// - Names are textbook names in "X from Y" form (`hsvFromRGB`, `rgbFromHSV`); the graphics_metal
//   name is in each Tags line.
// - Swizzles (`c.bg`) are written as components, and `mix(a, b, step(...))` as `linearBlend` with
//   a `stepFunction` amount, which is the same arithmetic (a + (b − a) × amount).
// - `fract` is the prelude's `fractionalPart` (value − ⌊value⌋), which can give exactly 1 for a
//   tiny negative value where Metal's own `fract` stops just below 1. For hues that is the same
//   colour (hue 1 is hue 0).
// - `linearToSrgb` is `SRGBTransfer::encodedFromLinear` per channel (see `encodedSRGBFromLinear`).
//   `srgbToLinear` is NOT `SRGBTransfer::linearFromEncoded`: graphics_metal's does not clamp, so it
//   is kept as `linearFromEncodedColor`.
// - `lookupColor` was not ported: it samples a texture, which kernels cannot do.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "SRGBTransfer.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

// ---------------------------------------------------------------------------------------------
// OKLab and OKHSV to sRGB

/// Linear sRGB for an OKLab colour (lightness, green–red a, blue–yellow b).
///
/// How: Björn Ottosson's OKLab (2020). Three cube-root values l′, m′, s′ are made from the lab
/// colour, cubed to l, m, s, and mixed to linear red, green, blue by the inverse matrix:
///
///     l′ = L + 0.3963377774 a + 0.2158037573 b      l = l′³
///     m′ = L − 0.1055613458 a − 0.0638541728 b      m = m′³
///     s′ = L − 0.0894841775 a − 1.2914855480 b      s = s′³
///     red   =  4.0767416621 l − 3.3077115913 m + 0.2309699292 s
///     green = −1.2684380046 l + 2.6097574011 m − 0.3413193965 s
///     blue  = −0.0041960863 l − 0.7034186147 m + 1.7076147010 s
///
/// Conventions: linear light, not gamma-encoded. Colours outside the sRGB gamut give channels
/// below 0 or above 1; nothing is clamped. Lab (1, 0, 0) is white (1, 1, 1).
///
/// Use when: showing an OKLab or OKLCH colour from a gradient or picker on screen
/// Tags: oklabToLinearSrgb, OKLab to linear sRGB, perceptual color, Ottosson
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> linearSRGBFromOklab(Vector3Of<Real> lab) {
    Real longRoot = lab.x + Real(0.3963377774f) * lab.y + Real(0.2158037573f) * lab.z;
    Real mediumRoot = lab.x - Real(0.1055613458f) * lab.y - Real(0.0638541728f) * lab.z;
    Real shortRoot = lab.x - Real(0.0894841775f) * lab.y - Real(1.2914855480f) * lab.z;
    Real longCone = longRoot * longRoot * longRoot;
    Real mediumCone = mediumRoot * mediumRoot * mediumRoot;
    Real shortCone = shortRoot * shortRoot * shortRoot;
    return Vector3Of<Real>(
        Real(4.0767416621f) * longCone - Real(3.3077115913f) * mediumCone + Real(0.2309699292f) * shortCone,
        Real(-1.2684380046f) * longCone + Real(2.6097574011f) * mediumCone - Real(0.3413193965f) * shortCone,
        Real(-0.0041960863f) * longCone - Real(0.7034186147f) * mediumCone + Real(1.7076147010f) * shortCone);
}

/// The sRGB-encoded value of each channel of a linear colour, for showing on screen.
///
/// How: `SRGBTransfer::encodedFromLinear` per channel (12.92 x below 0.0031308, else
/// 1.055 x^(1/2.4) − 0.055), result 0…1. graphics_metal's `linearToSrgb` is the same curve
/// clamped to 0…1; the two differ only at a channel exactly equal to 0.0031308, where they
/// pick different pieces that agree to 2.9e-8.
///
/// Conventions: input linear light, output sRGB-encoded 0…1; inputs outside 0…1 are clamped.
///
/// Use when: the last step before writing a colour to the screen
/// Tags: linearToSrgb, linear to sRGB, gamma encode, display encoding
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> encodedSRGBFromLinear(Vector3Of<Real> color) {
    return Vector3Of<Real>(SRGBTransfer::encodedFromLinear(color.x), SRGBTransfer::encodedFromLinear(color.y),
        SRGBTransfer::encodedFromLinear(color.z));
}

/// The sRGB-encoded colour for an OKLab colour.
///
/// How: `encodedSRGBFromLinear(linearSRGBFromOklab(lab))`.
///
/// Conventions: out-of-gamut colours are clamped channel by channel to 0…1.
///
/// Use when: drawing an OKLab colour
/// Tags: oklabToSrgb, OKLab to sRGB, perceptual color
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> encodedSRGBFromOklab(Vector3Of<Real> lab) {
    return encodedSRGBFromLinear(linearSRGBFromOklab(lab));
}

/// The inverse of Ottosson's OKHSV lightness "toe", mapping a perceived lightness back to OKLab L.
///
/// How: (x² + k₁ x) / (k₃ (x + k₂)) with k₁ = 0.206, k₂ = 0.03, k₃ = (1 + k₁)/(1 + k₂)
/// (Ottosson, "Okhsv and Okhsl", 2020). 0 → 0 and 1 → 1.
///
/// Use when: converting OKHSV value to OKLab lightness
/// Tags: okhsvToeInverse, toe, lightness correction, OKHSV
/// Node: none (shader library)
template <typename Real>
inline Real okhsvToeInverse(Real value) {
    const Real k1 = Real(0.206f);
    const Real k2 = Real(0.03f);
    const Real k3 = (Real(1.0f) + k1) / (Real(1.0f) + k2);
    return (value * value + k1 * value) / (k3 * (value + k2));
}

/// The largest saturation a hue can have inside the sRGB gamut, as a slope of chroma over
/// lightness, for the hue direction (a, b) in OKLab.
///
/// How: Ottosson's polynomial fit for the saturation S = C/L at which one of red, green or blue
/// first reaches 0 (the branch is chosen by which channel that is), followed by one step of
/// Halley's method on that channel's cubic to tighten it.
///
/// Conventions: `ab` is a unit direction (cos hue, sin hue).
///
/// Use when: the gamut edge of a hue in OKHSV
/// Tags: okhsvMaximumSaturation, compute max saturation, gamut edge, OKHSV
/// Node: none (shader library)
template <typename Real>
inline Real okhsvMaximumSaturation(Vector2Of<Real> ab) {
    Vector4Of<Real> coefficients;
    Real k4;
    Vector3Of<Real> weights;
    if (Real(-1.88170328f) * ab.x - Real(0.80936493f) * ab.y > Real(1.0f)) {
        coefficients = Vector4Of<Real>(Real(1.19086277f), Real(1.76576728f), Real(0.59662641f), Real(0.75515197f));
        k4 = Real(0.56771245f);
        weights = Vector3Of<Real>(Real(4.0767416621f), Real(-3.3077115913f), Real(0.2309699292f));
    } else if (Real(1.81444104f) * ab.x - Real(1.19445276f) * ab.y > Real(1.0f)) {
        coefficients = Vector4Of<Real>(Real(0.73956515f), Real(-0.45954404f), Real(0.08285427f), Real(0.12541070f));
        k4 = Real(0.14503204f);
        weights = Vector3Of<Real>(Real(-1.2684380046f), Real(2.6097574011f), Real(-0.3413193965f));
    } else {
        coefficients = Vector4Of<Real>(Real(1.35733652f), Real(-0.00915799f), Real(-1.15130210f), Real(-0.50559606f));
        k4 = Real(0.00692167f);
        weights = Vector3Of<Real>(Real(-0.0041960863f), Real(-0.7034186147f), Real(1.7076147010f));
    }
    Real saturation = coefficients.x + coefficients.y * ab.x + coefficients.z * ab.y
        + coefficients.w * ab.x * ab.x + k4 * ab.x * ab.y;
    Real kl = Real(0.3963377774f) * ab.x + Real(0.2158037573f) * ab.y;
    Real km = Real(-0.1055613458f) * ab.x - Real(0.0638541728f) * ab.y;
    Real ks = Real(-0.0894841775f) * ab.x - Real(1.2914855480f) * ab.y;
    Vector3Of<Real> roots(Real(1.0f) + saturation * kl, Real(1.0f) + saturation * km, Real(1.0f) + saturation * ks);
    Vector3Of<Real> first = Real(3.0f) * Vector3Of<Real>(kl, km, ks) * roots * roots;
    Vector3Of<Real> second = Real(6.0f) * Vector3Of<Real>(kl * kl, km * km, ks * ks) * roots;
    Real value = dotProduct(weights, roots * roots * roots);
    Real derivative = dotProduct(weights, first);
    Real secondDerivative = dotProduct(weights, second);
    saturation -= (value * derivative) / (derivative * derivative - Real(0.5f) * value * secondDerivative);
    return saturation;
}

/// The slopes (S, T) of the line from black to the sRGB gamut's cusp for a hue: chroma per unit
/// of lightness (x) and chroma per unit of distance from white (y).
///
/// How: find the cusp with `okhsvMaximumSaturation`, take its largest linear channel to scale
/// lightness L_cusp = ∛(1 / max channel), chroma C_cusp = L_cusp × saturation, then
/// S = C_cusp / L_cusp and T = C_cusp / (1 − L_cusp), each divisor held at least 0.000001.
///
/// Use when: scaling OKHSV saturation to the gamut of a hue
/// Tags: okhsvMaximumST, get ST max, cusp, gamut triangle, OKHSV
/// Node: none (shader library)
template <typename Real>
inline Vector2Of<Real> okhsvCuspSlopes(Vector2Of<Real> ab) {
    Real saturation = okhsvMaximumSaturation(ab);
    Vector3Of<Real> rgb = linearSRGBFromOklab(Vector3Of<Real>(Real(1.0f), saturation * ab.x, saturation * ab.y));
    Real lightness = power(Real(1.0f) / largerOf(largerOf(rgb.x, rgb.y), largerOf(rgb.z, smallestSafeDivisor<Real>())),
        Real(1.0f) / Real(3.0f));
    Real chroma = lightness * saturation;
    return Vector2Of<Real>(chroma / largerOf(lightness, smallestSafeDivisor<Real>()),
        chroma / largerOf(Real(1.0f) - lightness, smallestSafeDivisor<Real>()));
}

/// The sRGB-encoded colour for an OKHSV colour: a hue, a saturation and a value that stay inside
/// the sRGB gamut.
///
/// How: Björn Ottosson's "Okhsv and Okhsl" (2020), as in the web editor's colour picker. The
/// hue picks a direction (a, b) = (cos, sin) in OKLab; `okhsvCuspSlopes` gives the gamut
/// triangle for it; saturation and value are mapped through the triangle to a lightness and
/// chroma, the lightness is corrected by `okhsvToeInverse`, both are scaled so that value 1 is the
/// brightest in-gamut colour, and the OKLab result is encoded with `encodedSRGBFromOklab`.
///
/// Conventions: `hueDegrees` in degrees, any value (wrapped into 0…360); saturation and value
/// 0…1. A value of 0.000001 or less gives black. Saturation 0 gives greys.
///
/// Use when: a colour picker with perceptually even hue, saturation and value
/// Tags: okhsvToSrgb, OKHSV, Okhsv to sRGB, perceptual HSV, color picker
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> encodedSRGBFromOkhsv(Real hueDegrees, Real saturation, Real value) {
    if (value <= smallestSafeDivisor<Real>()) return Vector3Of<Real>(Real(0.0f));
    Real hue = degreesToRadians(flooredModulo(flooredModulo(hueDegrees, Real(360.0f)) + Real(360.0f), Real(360.0f)));
    Vector2Of<Real> ab(cosine(hue), sine(hue));
    Vector2Of<Real> cuspSlopes = okhsvCuspSlopes(ab);
    const Real baseSaturation = Real(0.5f);
    Real bend = Real(1.0f) - baseSaturation / cuspSlopes.x;
    Real denominator = baseSaturation + cuspSlopes.y - cuspSlopes.y * bend * saturation;
    Real valueLightness = Real(1.0f) - saturation * baseSaturation / denominator;
    Real valueChroma = saturation * cuspSlopes.y * baseSaturation / denominator;
    Real lightness = value * valueLightness;
    Real chroma = value * valueChroma;
    Real correctedValueLightness = okhsvToeInverse(valueLightness);
    Real correctedValueChroma = valueChroma * (correctedValueLightness / largerOf(valueLightness, smallestSafeDivisor<Real>()));
    Real correctedLightness = okhsvToeInverse(lightness);
    chroma *= correctedLightness / largerOf(lightness, smallestSafeDivisor<Real>());
    lightness = correctedLightness;
    Vector3Of<Real> scaleRGB = linearSRGBFromOklab(
        Vector3Of<Real>(correctedValueLightness, ab.x * correctedValueChroma, ab.y * correctedValueChroma));
    Real scale = power(Real(1.0f) / largerOf(largerOf(scaleRGB.x, scaleRGB.y), largerOf(scaleRGB.z, smallestSafeDivisor<Real>())),
        Real(1.0f) / Real(3.0f));
    lightness *= scale;
    chroma *= scale;
    return encodedSRGBFromOklab(Vector3Of<Real>(lightness, ab.x * chroma, ab.y * chroma));
}

// ---------------------------------------------------------------------------------------------
// Conversions between spaces. Hue is in turns (0 to 1) throughout.

/// The linear light value of each channel of an sRGB-encoded colour, without clamping.
///
/// How: encoded ≤ 0.04045: encoded / 12.92; otherwise ((max(encoded, 0) + 0.055) / 1.055)^2.4.
/// This is graphics_metal's `srgbToLinear`, not `SRGBTransfer::linearFromEncoded`: that one
/// clamps its input to 0…1, this one lets a negative channel through the straight piece
/// (−0.1292 → −0.01) and a channel above 1 grow past 1.
///
/// Conventions: input sRGB-encoded, output linear light.
///
/// Use when: turning a picked or texture colour into linear values before lighting or blending
/// Tags: srgbToLinear, sRGB to linear, decode gamma, remove gamma
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> linearFromEncodedColor(Vector3Of<Real> color) {
    Vector3Of<Real> straight = color / SRGBTransfer::linearSegmentSlope<Real>();
    Vector3Of<Real> curved = power((largerOf(color, Vector3Of<Real>(Real(0.0f))) + SRGBTransfer::powerSegmentOffset<Real>())
            / SRGBTransfer::powerSegmentScale<Real>(),
        Vector3Of<Real>(SRGBTransfer::gamma<Real>()));
    return chosen(color > Vector3Of<Real>(SRGBTransfer::encodedThreshold<Real>()), curved, straight);
}

/// A colour as hue, saturation and value.
///
/// How: the branch-free form (Sam Hocevar's, from the GLSL "rgb2hsv" shared widely) that sorts
/// the channels with two blends. With max and min the largest and smallest channel and
/// d = max − min: value = max, saturation = d / max, hue = |K + (g − b)/(6d)| in turns, with a
/// tiny 0.0000001 added to each divisor so that grey (d = 0) gives hue 0 and black gives
/// saturation 0 instead of NaN.
///
/// Conventions: red, green, blue 0…1 (sRGB-encoded, as picked); hue in turns 0…1: red 0, yellow
/// 1/6, green 1/3, cyan 1/2, blue 2/3. Pure red is (0, 1, 1).
///
/// Use when: changing hue or saturation of a colour; picking colours
/// Tags: rgbToHsv, RGB to HSV, hue saturation value, color space
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> hsvFromRGB(Vector3Of<Real> color) {
    Real red = color.x;
    Real green = color.y;
    Real blue = color.z;
    Vector4Of<Real> blueFirst(blue, green, Real(-1.0f), Real(2.0f) / Real(3.0f));
    Vector4Of<Real> greenFirst(green, blue, Real(0.0f), Real(-1.0f) / Real(3.0f));
    Vector4Of<Real> p = linearBlend(blueFirst, greenFirst, stepFunction(blue, green));
    Vector4Of<Real> redLast(p.x, p.y, p.w, red);
    Vector4Of<Real> redFirst(red, p.y, p.z, p.x);
    Vector4Of<Real> q = linearBlend(redLast, redFirst, stepFunction(p.x, red));
    Real spread = q.x - smallerOf(q.w, q.y);
    Real tiny = Real(0.0000001f);
    return Vector3Of<Real>(absolute(q.z + (q.w - q.y) / (Real(6.0f) * spread + tiny)), spread / (q.x + tiny), q.x);
}

/// A colour from hue, saturation and value.
///
/// How: three triangle waves of the hue, offset by a third of a turn each, give the pure hue:
/// p = |fract(h + (0, 2/3, 1/3)) × 6 − 3|, hue colour = clamp(p − 1, 0, 1); the result is
/// value × lerp(white, hue colour, saturation).
///
/// Conventions: hue in turns (wraps: 1 is 0), saturation and value 0…1; red, green, blue 0…1.
/// (0, 1, 1) is pure red.
///
/// Use when: building a colour from a hue slider; cycling hue over time
/// Tags: hsvToRgb, HSV to RGB, hue wheel, color space
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> rgbFromHSV(Vector3Of<Real> hsv) {
    Vector3Of<Real> offsets(Real(0.0f), Real(2.0f) / Real(3.0f), Real(1.0f) / Real(3.0f));
    Vector3Of<Real> p = absolute(fractionalPart(Vector3Of<Real>(hsv.x) + offsets) * Real(6.0f) - Real(3.0f));
    Vector3Of<Real> hueColor = clamped(p - Real(1.0f), Real(0.0f), Real(1.0f));
    return hsv.z * linearBlend(Vector3Of<Real>(Real(1.0f)), hueColor, hsv.y);
}

/// A colour as hue, saturation and lightness.
///
/// How: from HSV: lightness l = v (1 − s/2); saturation = (v − l) / min(l, 1 − l), or 0 when
/// that divisor is 0.000001 or less (black and white). The hue is HSV's.
///
/// Conventions: hue in turns; saturation and lightness 0…1. Pure red is (0, 1, 0.5).
///
/// Use when: lightening or darkening a colour while keeping its look
/// Tags: rgbToHsl, RGB to HSL, hue saturation lightness
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> hslFromRGB(Vector3Of<Real> color) {
    Vector3Of<Real> hsv = hsvFromRGB(color);
    Real lightness = hsv.z * (Real(1.0f) - hsv.y * Real(0.5f));
    Real denominator = smallerOf(lightness, Real(1.0f) - lightness);
    Real saturation = denominator > smallestSafeDivisor<Real>() ? (hsv.z - lightness) / denominator : Real(0.0f);
    return Vector3Of<Real>(hsv.x, saturation, lightness);
}

/// A colour from hue, saturation and lightness: the inverse of `hslFromRGB`.
///
/// How: HSV value v = l + s × min(l, 1 − l); HSV saturation = 2 (1 − l / v), or 0 when v is
/// 0.000001 or less; then `rgbFromHSV`.
///
/// Use when: building a colour from an HSL slider
/// Tags: hslToRgb, HSL to RGB, hue saturation lightness
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> rgbFromHSL(Vector3Of<Real> hsl) {
    Real value = hsl.z + hsl.y * smallerOf(hsl.z, Real(1.0f) - hsl.z);
    Real saturation = value > smallestSafeDivisor<Real>() ? Real(2.0f) * (Real(1.0f) - hsl.z / value) : Real(0.0f);
    return rgbFromHSV(Vector3Of<Real>(hsl.x, saturation, value));
}

/// An OKLab colour (lightness, a, b) for a linear sRGB colour.
///
/// How: Ottosson's OKLab (2020): the linear colour is mixed into three cone responses l, m, s by
/// a matrix (0.4122214708 r + 0.5363325363 g + 0.0514459929 b, …), each is cube-rooted (a
/// negative response is held at 0), and mixed into lightness and the two opponent axes by a
/// second matrix. White (1, 1, 1) gives (1, 0, 0).
///
/// Conventions: input linear light (use `linearFromEncodedColor` on picked colours first).
///
/// Use when: blending or comparing colours the way the eye does
/// Tags: linearSrgbToOklab, linear sRGB to OKLab, perceptual color, Ottosson
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> oklabFromLinearSRGB(Vector3Of<Real> color) {
    Real third = Real(1.0f) / Real(3.0f);
    Real longRoot = power(largerOf(Real(0.4122214708f) * color.x + Real(0.5363325363f) * color.y + Real(0.0514459929f) * color.z, Real(0.0f)), third);
    Real mediumRoot = power(largerOf(Real(0.2119034982f) * color.x + Real(0.6806995451f) * color.y + Real(0.1073969566f) * color.z, Real(0.0f)), third);
    Real shortRoot = power(largerOf(Real(0.0883024619f) * color.x + Real(0.2817188376f) * color.y + Real(0.6299787005f) * color.z, Real(0.0f)), third);
    return Vector3Of<Real>(
        Real(0.2104542553f) * longRoot + Real(0.7936177850f) * mediumRoot - Real(0.0040720468f) * shortRoot,
        Real(1.9779984951f) * longRoot - Real(2.4285922050f) * mediumRoot + Real(0.4505937099f) * shortRoot,
        Real(0.0259040371f) * longRoot + Real(0.7827717662f) * mediumRoot - Real(0.8086757660f) * shortRoot);
}

/// OKLab as lightness, chroma and hue (OKLCH).
///
/// How: chroma = √(a² + b²); hue = fract(atan2(b, a) / τ), in turns 0…1.
///
/// Conventions: lightness unchanged; hue 0 is the +a axis (pinkish red), counter-clockwise.
/// A grey (a = b = 0) has chroma 0 and hue 0.
///
/// Use when: changing the hue or chroma of a colour perceptually
/// Tags: oklabToOklch, OKLab to OKLCH, polar lab, LCH
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> oklchFromOklab(Vector3Of<Real> lab) {
    return Vector3Of<Real>(lab.x, lengthOf(Vector2Of<Real>(lab.y, lab.z)),
        fractionalPart(arcTangent(lab.z, lab.y) / tau<Real>()));
}

/// OKLab from lightness, chroma and hue: the inverse of `oklchFromOklab`.
///
/// How: a = chroma cos(τ hue), b = chroma sin(τ hue).
///
/// Conventions: hue in turns.
///
/// Use when: building a colour from an OKLCH slider or a gradient in OKLCH
/// Tags: oklchToOklab, OKLCH to OKLab, polar lab
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> oklabFromOklch(Vector3Of<Real> lch) {
    Real angle = lch.z * tau<Real>();
    return Vector3Of<Real>(lch.x, lch.y * cosine(angle), lch.y * sine(angle));
}

/// Converts between colour spaces by a mode number: 0 sRGB to linear, 1 linear to sRGB, then RGB
/// to and from HSV (2, 3), HSL (4, 5), OKLab (6, 7) and OKLCH (8, 9; any other mode is 9).
///
/// How: the matching function above. RGB here is sRGB-encoded; OKLab and OKLCH go through linear
/// light (6 is `oklabFromLinearSRGB(linearFromEncodedColor(c))`, 7 is the inverse).
///
/// Use when: a Convert Color node with a mode setting
/// Tags: convertColorSpace, convert color, color space node, colour conversion
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> convertedColorSpace(Vector3Of<Real> color, int mode) {
    switch (mode) {
        case 0: return linearFromEncodedColor(color);
        case 1: return encodedSRGBFromLinear(color);
        case 2: return hsvFromRGB(color);
        case 3: return rgbFromHSV(color);
        case 4: return hslFromRGB(color);
        case 5: return rgbFromHSL(color);
        case 6: return oklabFromLinearSRGB(linearFromEncodedColor(color));
        case 7: return encodedSRGBFromLinear(linearSRGBFromOklab(color));
        case 8: return oklchFromOklab(oklabFromLinearSRGB(linearFromEncodedColor(color)));
        default: return encodedSRGBFromLinear(linearSRGBFromOklab(oklabFromOklch(color)));
    }
}

// ---------------------------------------------------------------------------------------------
// Colour adjustments

/// A colour with its hue turned.
///
/// How: HSV, then hue = fract(hue + turns), then back to RGB.
///
/// Conventions: `turns` of 1 is a full circle (no change); greys do not change.
///
/// Use when: recolouring an object; animating a rainbow
/// Tags: shiftHue, hue shift, hue rotate, rotate hue
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> hueShifted(Vector3Of<Real> color, Real turns) {
    Vector3Of<Real> hsv = hsvFromRGB(color);
    return rgbFromHSV(Vector3Of<Real>(fractionalPart(hsv.x + turns), hsv.y, hsv.z));
}

/// A colour with its saturation scaled about its luminance grey.
///
/// How: lerp(grey, color, amount) with grey = luminance(color) in every channel. 0 gives grey, 1
/// leaves the colour, above 1 oversaturates (channels may leave 0…1).
///
/// Use when: a saturation slider; desaturating a layer
/// Tags: adjustSaturation, saturation, desaturate, saturate
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> saturationAdjusted(Vector3Of<Real> color, Real amount) {
    return linearBlend(Vector3Of<Real>(luminance(color)), color, amount);
}

/// Raises saturation most where a colour has little of it.
///
/// How: spread = max channel − min channel; lerp(grey, color, 1 + amount × (1 − spread)) with
/// grey = luminance(color). A vivid colour (spread 1) is unchanged; a dull one is scaled by up to
/// 1 + amount. Amount 0 changes nothing.
///
/// Use when: a vibrance slider that does not blow out colours that are already vivid
/// Tags: adjustVibrance, vibrance, smart saturation
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> vibranceAdjusted(Vector3Of<Real> color, Real amount) {
    Real spread = largerOf(color.x, largerOf(color.y, color.z)) - smallerOf(color.x, smallerOf(color.y, color.z));
    return linearBlend(Vector3Of<Real>(luminance(color)), color, Real(1.0f) + amount * (Real(1.0f) - spread));
}

/// A colour turned to grey: mode 0 luminance, 1 average, 2 brightest channel, 3 darkest channel.
///
/// How: luminance = 0.2126 r + 0.7152 g + 0.0722 b; average = (r + g + b)/3; maximum and minimum
/// of the channels. Any other mode is 0. The result has the grey in all three channels.
///
/// Use when: a black-and-white filter; a height from a colour
/// Tags: grayscale, grayscale mode, greyscale, desaturate
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> grayscaled(Vector3Of<Real> color, int mode) {
    if (mode == 1) return Vector3Of<Real>((color.x + color.y + color.z) / Real(3.0f));
    if (mode == 2) return Vector3Of<Real>(largerOf(color.x, largerOf(color.y, color.z)));
    if (mode == 3) return Vector3Of<Real>(smallerOf(color.x, smallerOf(color.y, color.z)));
    return Vector3Of<Real>(luminance(color));
}

/// Photoshop-style levels: new black and white points, a gamma, and an output range.
///
/// How: t = clamp((c − inBlack) / max(inWhite − inBlack, 0.000001), 0, 1); result =
/// lerp(outBlack, outWhite, t^(1 / max(gamma, 0.001))).
///
/// Conventions: gamma above 1 brightens the midtones, below 1 darkens them; 1 is linear.
///
/// Use when: stretching contrast; clipping blacks and whites
/// Tags: adjustLevels, levels, contrast stretch, gamma, black point, white point
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> levelsAdjusted(Vector3Of<Real> color, Real inBlack, Real inWhite, Real gammaValue, Real outBlack, Real outWhite) {
    Vector3Of<Real> t = clamped((color - inBlack) / largerOf(inWhite - inBlack, smallestSafeDivisor<Real>()), Real(0.0f), Real(1.0f));
    Vector3Of<Real> curved = power(t, Vector3Of<Real>(Real(1.0f) / largerOf(gammaValue, Real(0.001f))));
    return linearBlend(Vector3Of<Real>(outBlack), Vector3Of<Real>(outWhite), curved);
}

/// A smooth curve through (0, 0), (0.25, low), (0.5, mid), (0.75, high), (1, 1).
///
/// How: Catmull-Rom spline (Catmull and Rom, 1974) over the five points with the end points
/// repeated: with x scaled to t = 4x, segment i = ⌊t⌋ (at most 3), f = t − i and neighbouring
/// ordinates p₀…p₃, y = ½ [2p₁ + (p₂ − p₀) f + (2p₀ − 5p₁ + 4p₂ − p₃) f² + (3p₁ − p₀ − 3p₂ + p₃) f³],
/// clamped to 0…1.
///
/// Conventions: x is clamped to 0…1. The first and last segments repeat their end point as the outer
/// neighbour, so low, mid, high = 0.25, 0.5, 0.75 gives x only between 0.25 and 0.75.
///
/// Use when: a three-handle tone curve
/// Tags: toneCurve, curves, tone curve, Catmull-Rom, contrast curve
/// Node: none (shader library)
template <typename Real>
inline Real toneCurveValue(Real x, Real low, Real mid, Real high) {
    Real ordinates[5] = {Real(0.0f), low, mid, high, Real(1.0f)};
    Real scaled = clamped(x, Real(0.0f), Real(1.0f)) * Real(4.0f);
    int segment = smallerOf(static_cast<int>(scaled), 3);
    Real f = scaled - static_cast<Real>(segment);
    Real p0 = ordinates[largerOf(segment - 1, 0)];
    Real p1 = ordinates[segment];
    Real p2 = ordinates[segment + 1];
    Real p3 = ordinates[smallerOf(segment + 2, 4)];
    Real value = Real(0.5f) * ((Real(2.0f) * p1) + (-p0 + p2) * f + (Real(2.0f) * p0 - Real(5.0f) * p1 + Real(4.0f) * p2 - p3) * f * f
        + (-p0 + Real(3.0f) * p1 - Real(3.0f) * p2 + p3) * f * f * f);
    return clamped(value, Real(0.0f), Real(1.0f));
}

/// `toneCurveValue` on each channel of a colour.
///
/// Use when: a Curves node on a colour
/// Tags: toneCurve3, color curves, per channel curve
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> toneCurveColor(Vector3Of<Real> color, Real low, Real mid, Real high) {
    return Vector3Of<Real>(toneCurveValue(color.x, low, mid, high), toneCurveValue(color.y, low, mid, high),
        toneCurveValue(color.z, low, mid, high));
}

/// A colour reduced to a few evenly spaced levels per channel.
///
/// How: n = max(steps, 2) − 1; result = ⌊c n + 0.5⌋ / n. With 2 steps every channel becomes 0 or
/// 1; with 5 steps one of 0, ¼, ½, ¾, 1.
///
/// Use when: a poster or cartoon look; banding on purpose
/// Tags: posterize, quantize colors, color levels, banding
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> posterized(Vector3Of<Real> color, Real steps) {
    Real levels = largerOf(steps, Real(2.0f)) - Real(1.0f);
    return roundedDown(color * levels + Real(0.5f)) / levels;
}

/// A soft 0…1 mask of where a colour is brighter than a cutoff.
///
/// How: smoothstep from (cutoff − softness/2 − 0.0001) to (cutoff + softness/2 + 0.0001) of the
/// colour's luminance. The 0.0001 keeps the edges apart when softness is 0. A negative softness
/// that brings the edges within 0.000001 of each other makes a hard step (`safeSmoothStep`'s rule;
/// graphics_metal's smoothstep is undefined there).
///
/// Use when: selecting highlights; a bloom threshold
/// Tags: thresholdMask, threshold, luma key, highlight mask
/// Node: none (shader library)
template <typename Real>
inline Real luminanceThresholdMask(Vector3Of<Real> color, Real cutoff, Real softness) {
    return safeSmoothStep(cutoff - softness * Real(0.5f) - Real(0.0001f), cutoff + softness * Real(0.5f) + Real(0.0001f),
        luminance(color));
}

/// Dark to the tint to white by the colour's brightness, then faded in by `amount`.
///
/// How: l = luminance(color); toned = l < ½ ? lerp(black, tint, 2l) : lerp(tint, white, 2(l − ½));
/// result lerp(color, toned, amount).
///
/// Use when: a duotone or sepia look
/// Tags: colorize, tint, duotone, sepia, monochrome tint
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> colorized(Vector3Of<Real> color, Vector3Of<Real> tint, Real amount) {
    Real brightness = luminance(color);
    Vector3Of<Real> toned = brightness < Real(0.5f)
        ? linearBlend(Vector3Of<Real>(Real(0.0f)), tint, brightness * Real(2.0f))
        : linearBlend(tint, Vector3Of<Real>(Real(1.0f)), (brightness - Real(0.5f)) * Real(2.0f));
    return linearBlend(color, toned, amount);
}

/// A colour with a warmer or cooler cast, and a green or magenta tint.
///
/// How: multiply by (1 + 0.4 temperature, 1 − 0.4 tint, 1 − 0.4 temperature). Positive
/// temperature adds red and removes blue; positive tint removes green.
///
/// Use when: correcting or stylising the colour cast of a scene
/// Tags: whiteBalance, white balance, temperature, tint, warm, cool
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> whiteBalanced(Vector3Of<Real> color, Real temperature, Real tint) {
    return color * Vector3Of<Real>(Real(1.0f) + Real(0.4f) * temperature, Real(1.0f) - Real(0.4f) * tint,
        Real(1.0f) - Real(0.4f) * temperature);
}

/// Tints shadows and highlights differently.
///
/// How: t = smoothstep(0, 1, clamp((luminance − balance) × 2 + ½, 0, 1)); the colour is
/// multiplied by 2 × lerp(shadows, highlights, t), then faded in by `amount`. A tint of
/// (½, ½, ½) changes nothing.
///
/// Use when: a teal-and-orange grade
/// Tags: splitTone, split toning, shadows highlights tint, color grading
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> splitToned(Vector3Of<Real> color, Vector3Of<Real> shadows, Vector3Of<Real> highlights, Real balance, Real amount) {
    Real t = safeSmoothStep(Real(0.0f), Real(1.0f),
        clamped((luminance(color) - balance) * Real(2.0f) + Real(0.5f), Real(0.0f), Real(1.0f)));
    return linearBlend(color, color * linearBlend(shadows, highlights, t) * Real(2.0f), amount);
}

/// Four colour stops at positions p0 to p3 (in order), blended linearly between and held at the ends.
///
/// How: start with c0; blend toward c1 by clamp((t − p0)/(p1 − p0), 0, 1), then toward c2 by the
/// same over p1…p2, then c3 over p2…p3. Each span is held at least 0.00001 wide.
///
/// Conventions: colours are red, green, blue, alpha; before p0 the result is c0, after p3 it is c3.
///
/// Use when: a gradient ramp node
/// Tags: rampColor, color ramp, gradient, color stops
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> rampColor(Real t, Vector4Of<Real> c0, Vector4Of<Real> c1, Vector4Of<Real> c2, Vector4Of<Real> c3,
    Real p0, Real p1, Real p2, Real p3) {
    Real narrowest = Real(0.00001f);
    Vector4Of<Real> result = c0;
    result = linearBlend(result, c1, clamped((t - p0) / largerOf(p1 - p0, narrowest), Real(0.0f), Real(1.0f)));
    result = linearBlend(result, c2, clamped((t - p1) / largerOf(p2 - p1, narrowest), Real(0.0f), Real(1.0f)));
    result = linearBlend(result, c3, clamped((t - p2) / largerOf(p3 - p2, narrowest), Real(0.0f), Real(1.0f)));
    return result;
}

/// Inigo Quilez's cosine palette: a smooth looping colour gradient from four colours.
///
/// How: a + b × cos(τ (c t + d)) per channel (Quilez, "Palettes", 2015). `a` is the offset, `b` the
/// amplitude, `c` the frequency and `d` the phase. The result repeats every 1/c in t.
///
/// Conventions: channels are not clamped; choose a, b so a ± b stays in 0…1.
///
/// Use when: a procedural gradient or an iteration colouring
/// Tags: cosinePalette, cosine gradient, Inigo Quilez palette, procedural palette
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> cosinePalette(Real t, Vector3Of<Real> a, Vector3Of<Real> b, Vector3Of<Real> c, Vector3Of<Real> d) {
    return Vector3Of<Real>(a.x + b.x * cosine(tau<Real>() * (c.x * t + d.x)), a.y + b.y * cosine(tau<Real>() * (c.y * t + d.y)),
        a.z + b.z * cosine(tau<Real>() * (c.z * t + d.z)));
}

} // namespace kernels
} // namespace editor
