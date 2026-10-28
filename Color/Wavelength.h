// The wavelength of light: what we call its colour when it is a single pure colour
// (Engine/Math/Optics/Wavelength.swift).
#pragma once

namespace editor {

/// A closed span of wavelengths in nanometres (Swift `ClosedRange<Float>`).
///
/// Use when: naming the span of wavelengths a formula is meaningful over
/// Tags: wavelength range, nanometre span, spectrum range
/// Node: none (value type)
struct WavelengthRange {
    float lower;
    float upper;
};

/// The length of one wave of light, which sets its colour: violet is about 400 nm, green
/// about 550 nm, red about 700 nm.
///
/// Stored in **nanometres** (1 nm = 10⁻⁹ m = 10⁻³ µm), the unit people quote colours in.
/// The dispersion formulas of glass catalogs want **micrometres** (µm), so both are
/// available by name and the unit is never a bare number at a call site.
///
/// C++ naming: Swift's `Wavelength.nanometres(_:)` is `fromNanometres` here, because the
/// stored field is already called `nanometres`.
///
/// Use when: describing the color of pure light, such as red at 700 nm, for dispersion or a
///   rainbow
/// Tags: wavelength, nanometres, color of light, spectrum, lambda, wave length
/// Node: none (value type)
struct Wavelength {
    /// The wavelength in nanometres.
    float nanometres = 0;

    constexpr bool operator==(const Wavelength&) const = default;

    /// A wavelength given in nanometres.
    ///
    /// Use when: writing wavelengths the way tables list them, such as 589 nm
    /// Tags: nanometres, nm, wavelength from nanometres, spectral line
    /// Node: none (renderer shading)
    static constexpr Wavelength fromNanometres(float nanometres) { return Wavelength{nanometres}; }

    /// A wavelength given in micrometres: `nanometres = micrometres × 1000`.
    ///
    /// Use when: entering a wavelength in the unit used by Cauchy or Sellmeier formulas
    /// Tags: micrometres, microns, um, wavelength from micrometres
    /// Node: none (renderer shading)
    static constexpr Wavelength micrometres(float micrometres) {
        return Wavelength{micrometres * nanometresPerMicrometre};
    }

    /// The wavelength in micrometres: `nanometres / 1000`.
    ///
    /// Use when: plugging a wavelength into Cauchy or Sellmeier formulas
    /// Tags: micrometres, microns, um, lambda in micrometres
    /// Node: none (renderer shading)
    constexpr float micrometres() const { return nanometres / nanometresPerMicrometre; }

    /// The shortest and longest wavelengths the standard observer sees, 380–780 nm, the span
    /// the CIE 1931 tables are used over (CIE 15:2004).
    static constexpr WavelengthRange visibleRange{380, 780};

    // The three spectral lines that define the **Abbe number** of a glass (ISO 7944).

    /// The **d** line, helium (not one of Fraunhofer's own lines; his D is sodium), 587.5618 nm
    /// (yellow): the reference for "the" index.
    static constexpr Wavelength dLine() { return Wavelength{587.5618f}; }
    /// The Fraunhofer **F** line, hydrogen, 486.1327 nm (blue-green).
    static constexpr Wavelength fLine() { return Wavelength{486.1327f}; }
    /// The Fraunhofer **C** line, hydrogen, 656.2725 nm (red).
    static constexpr Wavelength cLine() { return Wavelength{656.2725f}; }

private:
    static constexpr float nanometresPerMicrometre = 1000;
};

} // namespace editor
