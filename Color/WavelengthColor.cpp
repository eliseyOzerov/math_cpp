// The colour we see for light of a single wavelength (Engine/Math/Optics/WavelengthColor.swift).
#include "WavelengthColor.h"

#include <cmath>

#include "../Linear/Scalar.h"

namespace editor {

namespace {

/// One lobe of the Wyman fit: a Gaussian bump centred at `centre` whose width is
/// `leftWidth` on the short-wavelength side and `rightWidth` on the other.
float lobe(float lambda, float centre, float leftWidth, float rightWidth) {
    float width = lambda < centre ? leftWidth : rightWidth;
    float distanceInWidths = (lambda - centre) / width;
    return std::exp(-0.5f * distanceInWidths * distanceInWidths);
}

} // namespace

Vector3 WavelengthColor::colorMatching(Wavelength wavelength) {
    float lambda = wavelength.nanometres;
    float x = 1.056f * lobe(lambda, 599.8f, 37.9f, 31.0f)
        + 0.362f * lobe(lambda, 442.0f, 16.0f, 26.7f)
        - 0.065f * lobe(lambda, 501.1f, 20.4f, 26.2f);
    float y = 0.821f * lobe(lambda, 568.8f, 46.9f, 40.5f)
        + 0.286f * lobe(lambda, 530.9f, 16.3f, 31.1f);
    float z = 1.217f * lobe(lambda, 437.0f, 11.8f, 36.0f)
        + 0.681f * lobe(lambda, 459.0f, 26.0f, 13.8f);
    return Vector3(x, y, z);
}

Vector3 WavelengthColor::linearRGB(Wavelength wavelength) {
    Vector3 xyz = colorMatching(wavelength);
    float red = 3.2406f * xyz.x - 1.5372f * xyz.y - 0.4986f * xyz.z;
    float green = -0.9689f * xyz.x + 1.8758f * xyz.y + 0.0415f * xyz.z;
    float blue = 0.0557f * xyz.x - 0.2040f * xyz.y + 1.0570f * xyz.z;
    return Vector3(largerOf(red, 0), largerOf(green, 0), largerOf(blue, 0));
}

Vector3 WavelengthColor::displayColor(Wavelength wavelength) {
    Wavelength hueWavelength = Wavelength::fromNanometres(
        clamped(wavelength.nanometres, reliableHueRange.lower, reliableHueRange.upper));
    Vector3 color = linearRGB(hueWavelength);
    return color / largerOf(largerOf(color.x, color.y), color.z);
}

} // namespace editor
