// The sRGB transfer functions: the curve between the linear light values the engine computes
// with (see `Color`) and the encoded values stored in image files and shown by screens
// (Engine/Math/Optics/SRGB.swift). The formulas live in the shared kernel
// Kernels/SRGBTransfer.h; this struct is the CPU-side name for them.
#pragma once

#include "../Kernels/SRGBTransfer.h"

namespace editor {

/// **sRGB** encoding and decoding of a single color channel.
///
/// Source: IEC 61966-2-1:1999, the sRGB standard (Stokes, Anderson, Chandrasekar & Motta,
/// 1996). Constants below are the standard's.
///
/// Why a curve: eyes tell dark shades apart better than bright ones, so image files spend
/// more of their 256 steps on the darks. Linear value 0.5 (half the light) is stored as
/// 0.7354.
///
/// Conventions: channel values are 0…1 and inputs outside it are clamped (this is not the
/// extended-range variant). Alpha is never encoded; it is already linear coverage.
///
/// Use when: showing linear light values on screen correctly; reading colors from images or
///   color pickers
/// Tags: sRGB, gamma, gamma correction, linear to sRGB, color space, transfer function
/// Node: none (renderer shading)
struct SRGB {
    /// Linear values at or below this use the straight-line piece of the encoding curve.
    static constexpr float linearThreshold = kernels::SRGBTransfer::linearThreshold<float>();
    /// Encoded values at or below this use the straight-line piece of the decoding curve.
    static constexpr float encodedThreshold = kernels::SRGBTransfer::encodedThreshold<float>();
    /// The slope of the straight-line piece: encoded = 12.92 · linear.
    static constexpr float linearSegmentSlope = kernels::SRGBTransfer::linearSegmentSlope<float>();
    /// The scale of the power piece of the encoding curve.
    static constexpr float powerSegmentScale = kernels::SRGBTransfer::powerSegmentScale<float>();
    /// The offset of the power piece, chosen so the curve passes through 1 at 1 and joins
    /// the line smoothly: scale − offset = 1, so 1.055 − 0.055.
    static constexpr float powerSegmentOffset = kernels::SRGBTransfer::powerSegmentOffset<float>();
    /// The exponent of the decoding power piece. Encoding uses its reciprocal, 1/2.4.
    static constexpr float gamma = kernels::SRGBTransfer::gamma<float>();

    /// The encoded sRGB value for a linear light value (Swift `encoded(fromLinear:)`).
    ///
    /// Formula:
    ///
    ///     linear ≤ 0.0031308 :  12.92 · linear
    ///     otherwise          :  1.055 · linear^(1/2.4) − 0.055
    ///
    /// The straight piece near black avoids the power curve's infinite slope at 0. The
    /// curve is increasing, maps 0 to 0 and 1 to 1, and is continuous at the threshold to
    /// within the standard's rounding: the two pieces differ by about 2.9e-8 there (2.3e-9
    /// for the decoding curve), far below one step of an 8-bit image (0.0039).
    ///
    /// Use when: preparing a lit pixel value for display on a screen
    /// Tags: linear to sRGB, gamma encode, gamma 2.2, display encoding, OETF, encode color
    /// Node: none (renderer shading)
    static float encodedFromLinear(float linear) { return kernels::SRGBTransfer::encodedFromLinear(linear); }

    /// The linear light value for an encoded sRGB value: the inverse of `encodedFromLinear`
    /// (Swift `linear(fromEncoded:)`).
    ///
    /// Formula:
    ///
    ///     encoded ≤ 0.04045 :  encoded / 12.92
    ///     otherwise         :  ((encoded + 0.055) / 1.055)^2.4
    ///
    /// 0.04045 is about `12.92 × 0.0031308`, the encoded value of the other curve's threshold.
    ///
    /// Use when: turning a picked or texture color into linear values before lighting or blending
    /// Tags: sRGB to linear, gamma decode, remove gamma, EOTF, decode color, texture color to linear
    /// Node: none (renderer shading)
    static float linearFromEncoded(float encoded) { return kernels::SRGBTransfer::linearFromEncoded(encoded); }
};

} // namespace editor
