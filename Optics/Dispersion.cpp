// Dispersion: glass bends blue light more than red (Engine/Math/Optics/Dispersion.swift).
#include "Dispersion.h"

#include <cassert>
#include <cmath>

namespace editor {

float DispersiveMaterial::abbeNumber() const {
    return (refractiveIndex(Wavelength::dLine()) - 1)
        / (refractiveIndex(Wavelength::fLine()) - refractiveIndex(Wavelength::cLine()));
}

float CauchyDispersion::refractiveIndex(Wavelength wavelength) const {
    assert(wavelength.nanometres > 0 && "A wavelength must be positive.");
    float micrometres = wavelength.micrometres();
    return coefficientA + coefficientB / (micrometres * micrometres);
}

SellmeierDispersion SellmeierDispersion::schottNBK7() {
    return SellmeierDispersion(
        Vector3(1.03961212f, 0.231792344f, 1.01046945f),
        Vector3(0.00600069867f, 0.0200179144f, 103.560653f));
}

SellmeierDispersion SellmeierDispersion::schottNSF11() {
    return SellmeierDispersion(
        Vector3(1.73759695f, 0.313747346f, 1.89878101f),
        Vector3(0.013188707f, 0.0623068142f, 155.23629f));
}

SellmeierDispersion SellmeierDispersion::fusedSilica() {
    return SellmeierDispersion(
        Vector3(0.6961663f, 0.4079426f, 0.8974794f),
        Vector3(0.0684043f * 0.0684043f, 0.1162414f * 0.1162414f, 9.896161f * 9.896161f));
}

float SellmeierDispersion::refractiveIndex(Wavelength wavelength) const {
    assert(wavelength.nanometres > 0 && "A wavelength must be positive.");
    float micrometres = wavelength.micrometres();
    float wavelengthSquared = micrometres * micrometres;
    float indexSquared = 1;
    for (int resonance = 0; resonance < 3; resonance += 1) {
        indexSquared += strengths[resonance] * wavelengthSquared
            / (wavelengthSquared - squaredResonanceWavelengths[resonance]);
    }
    return std::sqrt(indexSquared);
}

} // namespace editor
