// Numbers as text, printed exactly the way Swift prints them, so a problem reads the same from
// either engine, and read back from the digits of a graph file.
#pragma once

#include <string>

#include "Vector3.h"

namespace editor {

/// The text Swift's `Float.description` gives for `value`: the fewest digits that read back as
/// the same float, such as "3.0", "0.1", "16777216.0", "1e-05" or "3.3554432e+07".
///
/// How: take the shortest round-trip digits and their decimal exponent (the power of ten of the
/// first digit). If the exponent is −4 or more and |value| is at most 2^24, write the digits in
/// fixed notation, adding ".0" to a whole number. Otherwise write one digit, the rest after a
/// point (none if there is no rest), then "e", the exponent's sign and at least two digits.
///
/// Conventions: infinity prints "inf" or "-inf", not-a-number "nan", negative zero "-0.0".
///
/// Use when: putting a number into a problem's sentence; printing a value for a person
/// Tags: float to string, number formatting, description, shortest round trip
/// Node: none (messages)
std::string numberText(float value);

/// A vector as text: its three components in brackets, "[1.0, 2.0, 0.5]".
///
/// Conventions: Swift prints `SIMD3<Float>(1.0, 2.0, 0.5)` here; the bracket form is the one
/// deliberate difference between the engines' messages, because it is how a graph writes a
/// vector.
///
/// Use when: putting a vector into a problem's sentence
/// Tags: vector to string, vector formatting
/// Node: none (messages)
std::string vectorText(Vector3 vector);

/// The float nearest to the decimal number written in `text` (JSON number syntax), rounded once.
///
/// How: `std::from_chars`, except on Apple systems, where libc++ offers it for floating point
/// only from macOS 26 and hosts such as Unreal build for older systems; there the C library's
/// `strtof_l` in the C locale, which also rounds correctly (David Gay's algorithm), so a "." is
/// the decimal point whatever the user's language.
///
/// Conventions: `text` must be a whole, valid number; graph files are checked by the JSON reader
/// first.
///
/// Use when: reading a number from a graph file
/// Tags: parse float, string to float, from_chars, strtof, decimal to binary
/// Node: none (file format)
float floatFromText(const std::string& text);

/// The shortest decimal that reads back as exactly `value`, in scientific notation
/// ("1.0000000000000002e+00"), with a "." whatever the user's language.
///
/// Conventions: `value` must be finite.
///
/// Use when: writing a double to a graph file
/// Tags: double to string, shortest round trip, to_chars, ryu
/// Node: none (file format)
std::string doubleText(double value);

/// The double nearest to the decimal number written in `text`, as `floatFromText` for doubles.
///
/// Use when: the JSON reader keeps a number at full precision
/// Tags: parse double, string to double, from_chars, strtod
/// Node: none (file format)
double doubleFromText(const std::string& text);

} // namespace editor
