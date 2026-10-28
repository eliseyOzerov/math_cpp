// The sRGB transfer curves for one colour channel (Engine/Math/Optics/SRGB.swift, `SRGB`): the
// shared kernel behind `editor::SRGB`, so the formulas exist once and run on the CPU and in a
// Metal shader alike.
#pragma once

#include "KernelPrelude.h"

namespace editor {
namespace kernels {

/// **sRGB** encoding and decoding of a single colour channel.
///
/// Source: IEC 61966-2-1:1999, the sRGB standard (Stokes, Anderson, Chandrasekar & Motta,
/// 1996). Constants below are the standard's.
///
/// Why a curve: eyes tell dark shades apart better than bright ones, so image files spend more
/// of their 256 steps on the darks. Linear value 0.5 (half the light) is stored as 0.7354.
///
/// Conventions: channel values are 0…1 and inputs outside it are clamped (this is not the
/// extended-range variant). Alpha is never encoded; it is already linear coverage. The constants
/// are functions of the number type because a kernel cannot write a `double` literal.
///
/// Use when: showing linear light values on screen correctly; reading colors from images or
///   color pickers, on the GPU or the CPU
/// Tags: sRGB, gamma, gamma correction, linear to sRGB, color space, transfer function
/// Node: none (renderer shading)
struct SRGBTransfer {
    /// Linear values at or below this use the straight-line piece of the encoding curve.
    template <typename Real>
    static constexpr Real linearThreshold() { return Real(0.0031308f); }

    /// Encoded values at or below this use the straight-line piece of the decoding curve.
    template <typename Real>
    static constexpr Real encodedThreshold() { return Real(0.04045f); }

    /// The slope of the straight-line piece: encoded = 12.92 · linear.
    template <typename Real>
    static constexpr Real linearSegmentSlope() { return Real(12.92f); }

    /// The scale of the power piece of the encoding curve.
    template <typename Real>
    static constexpr Real powerSegmentScale() { return Real(1.055f); }

    /// The offset of the power piece, chosen so the curve passes through 1 at 1 and joins the
    /// line smoothly: scale − offset = 1, so 1.055 − 0.055.
    template <typename Real>
    static constexpr Real powerSegmentOffset() { return Real(0.055f); }

    /// The exponent of the decoding power piece. Encoding uses its reciprocal, 1/2.4.
    template <typename Real>
    static constexpr Real gamma() { return Real(2.4f); }

    /// The encoded sRGB value for a linear light value.
    ///
    /// Formula:
    ///
    ///     linear ≤ 0.0031308 :  12.92 · linear
    ///     otherwise          :  1.055 · linear^(1/2.4) − 0.055
    ///
    /// The straight piece near black avoids the power curve's infinite slope at 0. The curve is
    /// increasing, maps 0 to 0 and 1 to 1, and is continuous at the threshold to within the
    /// standard's rounding: the two pieces differ by about 2.9e-8 there (2.3e-9 for the decoding
    /// curve), far below one step of an 8-bit image (0.0039).
    ///
    /// Use when: preparing a lit pixel value for display on a screen
    /// Tags: linear to sRGB, gamma encode, gamma 2.2, display encoding, OETF, encode color
    /// Node: none (renderer shading)
    template <typename Real>
    static Real encodedFromLinear(Real linear) {
        Real clampedLinear = clamped(linear, Real(0), Real(1));
        if (clampedLinear <= linearThreshold<Real>()) {
            return linearSegmentSlope<Real>() * clampedLinear;
        }
        return powerSegmentScale<Real>() * power(clampedLinear, Real(1) / gamma<Real>()) - powerSegmentOffset<Real>();
    }

    /// The linear light value for an encoded sRGB value: the inverse of `encodedFromLinear`.
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
    template <typename Real>
    static Real linearFromEncoded(Real encoded) {
        Real clampedEncoded = clamped(encoded, Real(0), Real(1));
        if (clampedEncoded <= encodedThreshold<Real>()) {
            return clampedEncoded / linearSegmentSlope<Real>();
        }
        return power((clampedEncoded + powerSegmentOffset<Real>()) / powerSegmentScale<Real>(), gamma<Real>());
    }
};

} // namespace kernels
} // namespace editor
