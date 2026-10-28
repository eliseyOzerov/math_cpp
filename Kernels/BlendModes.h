// The shader library's blend modes, ported from graphics_metal's Blend.metal: the W3C
// "Compositing and Blending Level 1" modes for one colour channel and for a whole colour, and
// compositing a colour over another with alpha. Written once for a number type `Real` (`float`) and
// the kernel vectors of it, so the CPU and a Metal shader run the same steps.
//
// Differences from graphics_metal, all on purpose:
// - Each separable mode is its own function (`multiplyBlend`, `screenBlend`, ...); the mode
//   numbers 0…13 still select one in `blendedChannel`, with the same numbering.
// - Names are textbook names; the graphics_metal name is in each Tags line.
// - `mix(...)` is `linearBlend` (the same arithmetic); `switch` falls to the last case for an
//   unknown mode, as graphics_metal's `default` does.
//
// Conventions for every function: colour channels 0…1, in whatever space the caller blends in
// (the W3C modes are defined on encoded or linear values alike); `base` is the colour underneath
// and `top` the colour laid over it.
#pragma once

#include "KernelPrelude.h"
#include "KernelVectors.h"
#include "ShaderMath.h"

namespace editor {
namespace kernels {

// ---------------------------------------------------------------------------------------------
// Separable modes: one channel at a time

/// Normal: the top channel.
///
/// Use when: laying a colour over another with no blending
/// Tags: blendChannel 0, normal blend, replace
/// Node: none (shader library)
template <typename Real>
inline Real normalBlend(Real base, Real top) {
    (void)base;
    return top;
}

/// Multiply: darkens, as two filters over each other.
///
/// How: base × top. White (1) leaves the base, black (0) gives black.
///
/// Use when: shadows, tinting, ambient occlusion over a colour
/// Tags: blendChannel 1, multiply blend, darken, product
/// Node: none (shader library)
template <typename Real>
inline Real multiplyBlend(Real base, Real top) {
    return base * top;
}

/// Screen: lightens, the inverse of multiply on the inverted colours.
///
/// How: base + top − base × top = 1 − (1 − base)(1 − top). Black leaves the base, white gives white.
///
/// Use when: glows, light leaks
/// Tags: blendChannel 2, screen blend, lighten, dodge-like
/// Node: none (shader library)
template <typename Real>
inline Real screenBlend(Real base, Real top) {
    return base + top - base * top;
}

/// Overlay: multiplies the darks of the base and screens the lights.
///
/// How: base ≤ ½: 2 base × top; otherwise 1 − 2 (1 − base)(1 − top). Overlay is hard light with
/// the two colours swapped; a base of ½ gives the top back.
///
/// Use when: adding contrast and texture over a base colour
/// Tags: blendChannel 3, overlay blend, contrast blend
/// Node: none (shader library)
template <typename Real>
inline Real overlayBlend(Real base, Real top) {
    return base <= Real(0.5f) ? Real(2.0f) * base * top : Real(1.0f) - Real(2.0f) * (Real(1.0f) - base) * (Real(1.0f) - top);
}

/// Soft light: a gentle darken or lighten of the base by the top, W3C's formula.
///
/// How: with shade D = ((16 base − 12) base + 4) base for base ≤ ¼ and √base above,
/// top ≤ ½: base − (1 − 2 top) base (1 − base); otherwise base + (2 top − 1)(D − base). A top of ½
/// leaves the base.
///
/// Use when: subtle shading and dodge-and-burn
/// Tags: blendChannel 4, soft light blend, gentle contrast
/// Node: none (shader library)
template <typename Real>
inline Real softLightBlend(Real base, Real top) {
    Real shade = base <= Real(0.25f) ? ((Real(16.0f) * base - Real(12.0f)) * base + Real(4.0f)) * base : squareRoot(base);
    return top <= Real(0.5f) ? base - (Real(1.0f) - Real(2.0f) * top) * base * (Real(1.0f) - base)
                             : base + (Real(2.0f) * top - Real(1.0f)) * (shade - base);
}

/// Hard light: multiplies or screens by the top, like a harsh spotlight.
///
/// How: top ≤ ½: 2 base × top; otherwise 1 − 2 (1 − base)(1 − top). A top of ½ leaves the base.
///
/// Use when: strong contrast from the top layer
/// Tags: blendChannel 5, hard light blend
/// Node: none (shader library)
template <typename Real>
inline Real hardLightBlend(Real base, Real top) {
    return top <= Real(0.5f) ? Real(2.0f) * base * top : Real(1.0f) - Real(2.0f) * (Real(1.0f) - base) * (Real(1.0f) - top);
}

/// Add (linear dodge): the sum, held at 1.
///
/// How: min(base + top, 1).
///
/// Use when: adding light, such as a glow or a particle
/// Tags: blendChannel 6, add blend, linear dodge, plus
/// Node: none (shader library)
template <typename Real>
inline Real addBlend(Real base, Real top) {
    return smallerOf(base + top, Real(1.0f));
}

/// Subtract: the base less the top, held at 0.
///
/// How: max(base − top, 0).
///
/// Use when: removing light
/// Tags: blendChannel 7, subtract blend, minus
/// Node: none (shader library)
template <typename Real>
inline Real subtractBlend(Real base, Real top) {
    return largerOf(base - top, Real(0.0f));
}

/// Difference: how far apart the two are.
///
/// How: |base − top|. Equal colours give black.
///
/// Use when: comparing two images; inverting where the top is white
/// Tags: blendChannel 8, difference blend, absolute difference
/// Node: none (shader library)
template <typename Real>
inline Real differenceBlend(Real base, Real top) {
    return absolute(base - top);
}

/// Exclusion: like difference with lower contrast.
///
/// How: base + top − 2 base × top. A top of ½ gives ½ whatever the base.
///
/// Use when: a softer difference
/// Tags: blendChannel 9, exclusion blend
/// Node: none (shader library)
template <typename Real>
inline Real exclusionBlend(Real base, Real top) {
    return base + top - Real(2.0f) * base * top;
}

/// Colour dodge: brightens the base by dividing by the inverse of the top.
///
/// How: base ≤ 0: 0; otherwise top ≥ 1: 1; otherwise min(1, base / (1 − top)).
///
/// Use when: bright glows and sparks
/// Tags: blendChannel 10, color dodge blend, dodge
/// Node: none (shader library)
template <typename Real>
inline Real colorDodgeBlend(Real base, Real top) {
    if (base <= Real(0.0f)) return Real(0.0f);
    if (top >= Real(1.0f)) return Real(1.0f);
    return smallerOf(Real(1.0f), base / (Real(1.0f) - top));
}

/// Colour burn: darkens the base by dividing the inverse of the base by the top.
///
/// How: base ≥ 1: 1; otherwise top ≤ 0: 0; otherwise 1 − min(1, (1 − base) / top).
///
/// Use when: deep, saturated shadows
/// Tags: blendChannel 11, color burn blend, burn
/// Node: none (shader library)
template <typename Real>
inline Real colorBurnBlend(Real base, Real top) {
    if (base >= Real(1.0f)) return Real(1.0f);
    if (top <= Real(0.0f)) return Real(0.0f);
    return Real(1.0f) - smallerOf(Real(1.0f), (Real(1.0f) - base) / top);
}

/// Darken: the darker of the two.
///
/// How: min(base, top).
///
/// Use when: keeping only what is darker than the top
/// Tags: blendChannel 12, darken blend, minimum
/// Node: none (shader library)
template <typename Real>
inline Real darkenBlend(Real base, Real top) {
    return smallerOf(base, top);
}

/// Lighten: the lighter of the two.
///
/// How: max(base, top).
///
/// Use when: keeping only what is lighter than the top
/// Tags: blendChannel 13, lighten blend, maximum
/// Node: none (shader library)
template <typename Real>
inline Real lightenBlend(Real base, Real top) {
    return largerOf(base, top);
}

/// One channel through a separable blend mode.
///
/// How: mode 0 Normal, 1 Multiply, 2 Screen, 3 Overlay, 4 Soft Light, 5 Hard Light, 6 Add,
/// 7 Subtract, 8 Difference, 9 Exclusion, 10 Color Dodge, 11 Color Burn, 12 Darken; any other
/// mode is 13 Lighten. Each is the function of that name above.
///
/// Use when: a Blend node working one channel at a time
/// Tags: blendChannel, separable blend, blend mode switch
/// Node: none (shader library)
template <typename Real>
inline Real blendedChannel(Real base, Real top, int mode) {
    switch (mode) {
        case 0: return normalBlend(base, top);
        case 1: return multiplyBlend(base, top);
        case 2: return screenBlend(base, top);
        case 3: return overlayBlend(base, top);
        case 4: return softLightBlend(base, top);
        case 5: return hardLightBlend(base, top);
        case 6: return addBlend(base, top);
        case 7: return subtractBlend(base, top);
        case 8: return differenceBlend(base, top);
        case 9: return exclusionBlend(base, top);
        case 10: return colorDodgeBlend(base, top);
        case 11: return colorBurnBlend(base, top);
        case 12: return darkenBlend(base, top);
        default: return lightenBlend(base, top);
    }
}

// ---------------------------------------------------------------------------------------------
// Non-separable modes: hue, saturation, colour, luminosity (W3C Compositing and Blending, section
// "Non-separable blend modes")

/// The W3C blend luminance of a colour: how bright it is for the non-separable modes.
///
/// How: 0.3 r + 0.59 g + 0.11 b. These are the W3C weights, not `luminance`'s Rec. 709 ones.
///
/// Use when: the Hue, Saturation, Color and Luminosity blend modes
/// Tags: blendLuminance, Lum, W3C luminosity
/// Node: none (shader library)
template <typename Real>
inline Real blendLuminance(Vector3Of<Real> color) {
    return dotProduct(color, Vector3Of<Real>(Real(0.3f), Real(0.59f), Real(0.11f)));
}

/// A colour pulled back into 0…1 without changing its luminance or hue.
///
/// How: the W3C ClipColor. With l = blendLuminance, n the smallest and x the largest channel:
/// if n < 0, c = l + (c − l) l / (l − n); if x > 1, c = l + (c − l)(1 − l) / (x − l), each divisor
/// held at least 0.000001.
///
/// Use when: after changing a colour's luminance, which can push channels out of range
/// Tags: blendClip, ClipColor, gamut clip
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> clippedIntoRange(Vector3Of<Real> color) {
    Real luminanceOfColor = blendLuminance(color);
    Real smallest = smallerOf(color.x, smallerOf(color.y, color.z));
    Real largest = largerOf(color.x, largerOf(color.y, color.z));
    if (smallest < Real(0.0f)) {
        color = luminanceOfColor + (color - luminanceOfColor) * luminanceOfColor
            / largerOf(luminanceOfColor - smallest, smallestSafeDivisor<Real>());
    }
    if (largest > Real(1.0f)) {
        color = luminanceOfColor + (color - luminanceOfColor) * (Real(1.0f) - luminanceOfColor)
            / largerOf(largest - luminanceOfColor, smallestSafeDivisor<Real>());
    }
    return color;
}

/// A colour moved to a given luminance, keeping its hue and saturation, then clipped.
///
/// How: the W3C SetLum: `clippedIntoRange(color + (luminance − blendLuminance(color)))`.
///
/// Use when: the Color and Luminosity blend modes
/// Tags: blendSetLuminance, SetLum, set luminosity
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> withBlendLuminance(Vector3Of<Real> color, Real luminanceValue) {
    return clippedIntoRange(color + (luminanceValue - blendLuminance(color)));
}

/// The W3C saturation of a colour: its largest channel less its smallest.
///
/// How: max(r, g, b) − min(r, g, b).
///
/// Use when: the Hue and Saturation blend modes
/// Tags: blendSaturation, Sat, spread
/// Node: none (shader library)
template <typename Real>
inline Real blendSaturation(Vector3Of<Real> color) {
    return largerOf(color.x, largerOf(color.y, color.z)) - smallerOf(color.x, smallerOf(color.y, color.z));
}

/// A colour rescaled to a given saturation, keeping its hue.
///
/// How: the W3C SetSat. With n and x the smallest and largest channel: (c − n) × saturation /
/// (x − n), or black when x = n (a grey has no hue to scale).
///
/// Use when: the Hue and Saturation blend modes
/// Tags: blendSetSaturation, SetSat, set saturation
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> withBlendSaturation(Vector3Of<Real> color, Real saturation) {
    Real smallest = smallerOf(color.x, smallerOf(color.y, color.z));
    Real largest = largerOf(color.x, largerOf(color.y, color.z));
    return largest > smallest ? (color - smallest) * saturation / (largest - smallest) : Vector3Of<Real>(Real(0.0f));
}

/// The colour a blend mode gives, with no alpha.
///
/// How: modes 0…13 blend each channel with `blendedChannel`. Mode 14 Hue is the top's hue with
/// the base's saturation and luminance, 15 Saturation the base's hue and luminance with the top's
/// saturation, 16 Color the top's hue and saturation with the base's luminance, 17 Luminosity the
/// base's hue and saturation with the top's luminance (W3C formulas, SetLum and SetSat above).
/// Any other mode is 13 Lighten.
///
/// Use when: a Blend node
/// Tags: blendMode, blend modes, W3C compositing, hue saturation color luminosity
/// Node: none (shader library)
template <typename Real>
inline Vector3Of<Real> blendedColor(Vector3Of<Real> base, Vector3Of<Real> top, int mode) {
    if (mode == 14) return withBlendLuminance(withBlendSaturation(top, blendSaturation(base)), blendLuminance(base));
    if (mode == 15) return withBlendLuminance(withBlendSaturation(base, blendSaturation(top)), blendLuminance(base));
    if (mode == 16) return withBlendLuminance(top, blendLuminance(base));
    if (mode == 17) return withBlendLuminance(base, blendLuminance(top));
    return Vector3Of<Real>(blendedChannel(base.x, top.x, mode), blendedChannel(base.y, top.y, mode),
        blendedChannel(base.z, top.z, mode));
}

// ---------------------------------------------------------------------------------------------
// Compositing with alpha

/// `top` laid over `base` through a blend mode, by `top`'s alpha times `opacity`.
///
/// How: W3C general compositing. source = top.alpha × clamp(opacity, 0, 1); alpha = source +
/// base.alpha (1 − source); colour = [(1 − source) base.alpha base.rgb + source ((1 − base.alpha)
/// top.rgb + base.alpha blendedColor(base.rgb, top.rgb, mode))] / alpha.
///
/// Conventions: straight (not premultiplied) alpha in and out. When the result alpha is 0.000001
/// or less the colour is black. Opacity 0 gives the base back; an opaque base shows the blended colour.
///
/// Use when: a Layer node with a blend mode and opacity
/// Tags: blendColors, composite, layer blend, blend with opacity
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> compositedWithBlend(Vector4Of<Real> base, Vector4Of<Real> top, int mode, Real opacity) {
    Vector3Of<Real> baseColor(base.x, base.y, base.z);
    Vector3Of<Real> topColor(top.x, top.y, top.z);
    Real source = top.w * clamped(opacity, Real(0.0f), Real(1.0f));
    Real alpha = source + base.w * (Real(1.0f) - source);
    Vector3Of<Real> mixed = (Real(1.0f) - source) * base.w * baseColor
        + source * ((Real(1.0f) - base.w) * topColor + base.w * blendedColor(baseColor, topColor, mode));
    Vector3Of<Real> color = alpha > smallestSafeDivisor<Real>() ? mixed / alpha : Vector3Of<Real>(Real(0.0f));
    return Vector4Of<Real>(color.x, color.y, color.z, alpha);
}

/// `top` over `base` with straight alpha (Porter and Duff's "over", 1984).
///
/// How: alpha = top.alpha + base.alpha (1 − top.alpha); colour = (top.rgb top.alpha + base.rgb
/// base.alpha (1 − top.alpha)) / alpha, or black when alpha is 0.000001 or less.
///
/// Use when: stacking a translucent colour over another
/// Tags: alphaOver, over operator, Porter-Duff, composite
/// Node: none (shader library)
template <typename Real>
inline Vector4Of<Real> straightAlphaOver(Vector4Of<Real> base, Vector4Of<Real> top) {
    Vector3Of<Real> baseColor(base.x, base.y, base.z);
    Vector3Of<Real> topColor(top.x, top.y, top.z);
    Real alpha = top.w + base.w * (Real(1.0f) - top.w);
    Vector3Of<Real> mixed = topColor * top.w + baseColor * base.w * (Real(1.0f) - top.w);
    Vector3Of<Real> color = alpha > smallestSafeDivisor<Real>() ? mixed / alpha : Vector3Of<Real>(Real(0.0f));
    return Vector4Of<Real>(color.x, color.y, color.z, alpha);
}

} // namespace kernels
} // namespace editor
