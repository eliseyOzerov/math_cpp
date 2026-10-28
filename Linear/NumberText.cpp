#include "NumberText.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

#if defined(__APPLE__)
#include <xlocale.h>
#endif

namespace editor {

namespace {

/// Swift writes numbers from 2^24 up in exponential notation: above it, not every whole number
/// is a float, so fixed notation would suggest digits that are not there.
constexpr float largestFixedNumber = 16777216.0f;

/// Swift writes numbers whose first digit is below 10^−4 in exponential notation.
constexpr int smallestFixedExponent = -4;

/// The shortest digits that read back as exactly `value`, in C's "%e" form ("-d.ddde±XX").
///
/// How: `std::to_chars` in scientific format, except on Apple systems, where libc++ offers it
/// for floating point only from macOS 13.3 and hosts such as Unreal build for older systems.
/// There it prints with one, two, three... significant digits (`snprintf_l` in the C locale,
/// which rounds correctly) and keeps the first that reads back exactly. Both give the nearest
/// decimal of the shortest length that round-trips.
///
/// Use when: printing a float or double so it reads back unchanged
/// Tags: shortest round trip, to_chars, ryu, printf %e, float to string
/// Node: none (file format)
template <typename Number>
std::string shortestScientificText(Number value) {
    char buffer[64];
#if defined(__APPLE__)
    for (int precision = 0;; ++precision) {
        snprintf_l(buffer, sizeof buffer, LC_C_LOCALE, "%.*e", precision, static_cast<double>(value));
        bool readsBack = std::is_same_v<Number, float>
            ? strtof_l(buffer, nullptr, LC_C_LOCALE) == static_cast<float>(value)
            : strtod_l(buffer, nullptr, LC_C_LOCALE) == static_cast<double>(value);
        if (readsBack) return buffer;
    }
#else
    std::to_chars_result written = std::to_chars(buffer, buffer + sizeof buffer, value, std::chars_format::scientific);
    return std::string(buffer, written.ptr);
#endif
}

} // namespace

std::string numberText(float value) {
    if (std::isnan(value)) return "nan";
    if (std::isinf(value)) return value < 0 ? "-inf" : "inf";
    if (value == 0) return std::signbit(value) ? "-0.0" : "0.0";

    // The shortest round-trip digits, as "-d.ddde±XX".
    std::string scientific = shortestScientificText(value);
    size_t exponentMark = scientific.find('e');
    int exponent = std::atoi(scientific.c_str() + exponentMark + 1);
    std::string digits;
    for (size_t index = 0; index < exponentMark; ++index) {
        if (scientific[index] >= '0' && scientific[index] <= '9') digits += scientific[index];
    }
    std::string sign = value < 0 ? "-" : "";

    if (exponent >= smallestFixedExponent && std::fabs(value) <= largestFixedNumber) {
        if (exponent < 0) return sign + "0." + std::string(-exponent - 1, '0') + digits;
        size_t wholeDigits = static_cast<size_t>(exponent) + 1;
        if (digits.size() <= wholeDigits) return sign + digits + std::string(wholeDigits - digits.size(), '0') + ".0";
        return sign + digits.substr(0, wholeDigits) + "." + digits.substr(wholeDigits);
    }
    std::string mantissa = digits.size() == 1 ? digits : digits.substr(0, 1) + "." + digits.substr(1);
    std::string exponentDigits = std::to_string(std::abs(exponent));
    if (exponentDigits.size() < 2) exponentDigits = "0" + exponentDigits;
    return sign + mantissa + "e" + (exponent < 0 ? "-" : "+") + exponentDigits;
}

std::string vectorText(Vector3 vector) {
    return "[" + numberText(vector.x) + ", " + numberText(vector.y) + ", " + numberText(vector.z) + "]";
}

float floatFromText(const std::string& text) {
#if defined(__APPLE__)
    return strtof_l(text.c_str(), nullptr, LC_C_LOCALE);
#else
    float number = 0;
    std::from_chars(text.data(), text.data() + text.size(), number);
    return number;
#endif
}

std::string doubleText(double value) {
    return shortestScientificText(value);
}

double doubleFromText(const std::string& text) {
#if defined(__APPLE__)
    return strtod_l(text.c_str(), nullptr, LC_C_LOCALE);
#else
    double number = 0;
    std::from_chars(text.data(), text.data() + text.size(), number);
    return number;
#endif
}

} // namespace editor
