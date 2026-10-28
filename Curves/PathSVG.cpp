// See PathSVG.h. Ported from graphics_cpp's svg.cpp.
#include "PathSVG.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>

#include "../Linear/Angle.h"
#include "../Linear/MathProblem.h"
#include "CubicBezier.h"

namespace editor {

namespace {

using Cubic = CubicBezier<Vector2>;

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r'; }
bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isLetter(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

/// A quadratic Bézier (start, control, end) as the cubic that traces it exactly (degree raising):
/// each handle sits two thirds of the way from its end to the control point.
Cubic raisedQuadratic(Vector2 start, Vector2 control, Vector2 end) {
    return Cubic(start, Vector2::linearBlend(start, control, 2.0f / 3), Vector2::linearBlend(end, control, 2.0f / 3), end);
}

/// The angle in radians from vector u to vector v, positive counter-clockwise (in SVG's y-down
/// plane that reads clockwise, as the SVG notes define it).
float angleBetween(float ux, float uy, float vx, float vy) { return std::atan2(ux * vy - uy * vx, ux * vx + uy * vy); }

/// An elliptical arc from `from` to `to` as SVG gives it (radii, rotation, flags), as cubics: the
/// end-point form turned to the centre form (SVG 1.1 implementation notes F.6.5), radii grown when
/// too small to reach. Equal end points give none; a zero radius means a straight line, which is
/// reported by setting `straight` (and returning no cubics) for the caller to draw.
std::vector<Cubic> arcCubics(Vector2 from, float radiusX, float radiusY, float rotationDegrees, bool large, bool sweep,
                             Vector2 to, bool& straight) {
    straight = false;
    if (from.distance(to) < 1e-6f) return {};
    radiusX = std::fabs(radiusX);
    radiusY = std::fabs(radiusY);
    if (radiusX == 0 || radiusY == 0) {
        straight = true;
        return {};
    }
    float phi = Angle::degrees(rotationDegrees).radians, cosine = std::cos(phi), sine = std::sin(phi);
    float halfX = (from.x - to.x) / 2, halfY = (from.y - to.y) / 2;
    float rotatedX = cosine * halfX + sine * halfY, rotatedY = -sine * halfX + cosine * halfY;
    float grow = rotatedX * rotatedX / (radiusX * radiusX) + rotatedY * rotatedY / (radiusY * radiusY);
    // Radii too small to reach are scaled until the arc is exactly half an ellipse centred midway
    // (the root below is then 0 in theory, so it is set to 0 rather than left to rounding).
    bool scaledUp = grow > 1;
    if (scaledUp) {
        radiusX *= std::sqrt(grow);
        radiusY *= std::sqrt(grow);
    }
    float numerator = radiusX * radiusX * radiusY * radiusY - radiusX * radiusX * rotatedY * rotatedY -
                      radiusY * radiusY * rotatedX * rotatedX;
    float denominator = radiusX * radiusX * rotatedY * rotatedY + radiusY * radiusY * rotatedX * rotatedX;
    float factor = scaledUp ? 0.0f : (large == sweep ? -1.0f : 1.0f) * std::sqrt(std::max(0.0f, numerator / denominator));
    float centreRotatedX = factor * radiusX * rotatedY / radiusY, centreRotatedY = -factor * radiusY * rotatedX / radiusX;
    float centreX = cosine * centreRotatedX - sine * centreRotatedY + (from.x + to.x) / 2;
    float centreY = sine * centreRotatedX + cosine * centreRotatedY + (from.y + to.y) / 2;
    float startAngle = angleBetween(1, 0, (rotatedX - centreRotatedX) / radiusX, (rotatedY - centreRotatedY) / radiusY);
    float sweepAngle = angleBetween((rotatedX - centreRotatedX) / radiusX, (rotatedY - centreRotatedY) / radiusY,
                                    (-rotatedX - centreRotatedX) / radiusX, (-rotatedY - centreRotatedY) / radiusY);
    if (!sweep && sweepAngle > 0) sweepAngle -= 2 * Angle::pi;
    if (sweep && sweepAngle < 0) sweepAngle += 2 * Angle::pi;
    float pieceCount = std::ceil(std::fabs(sweepAngle) / (Angle::pi / 2) - 1e-5f);
    float step = sweepAngle / pieceCount, handle = 4.0f / 3 * std::tan(step / 4);
    auto onEllipse = [&](float angle) {
        return Vector2(centreX + radiusX * std::cos(angle) * cosine - radiusY * std::sin(angle) * sine,
                       centreY + radiusX * std::cos(angle) * sine + radiusY * std::sin(angle) * cosine);
    };
    auto tangent = [&](float angle) {
        return Vector2(-radiusX * std::sin(angle) * cosine - radiusY * std::cos(angle) * sine,
                       -radiusX * std::sin(angle) * sine + radiusY * std::cos(angle) * cosine);
    };
    std::vector<Cubic> result;
    int count = pieceCount > 0 ? int(pieceCount) : 0;
    for (int index = 0; index < count; index++) {
        float pieceStart = startAngle + float(index) * step, pieceEnd = pieceStart + step;
        Vector2 start = index == 0 ? from : onEllipse(pieceStart);
        Vector2 end = index == count - 1 ? to : onEllipse(pieceEnd);
        result.push_back(Cubic(start, start + tangent(pieceStart) * handle, end - tangent(pieceEnd) * handle, end));
    }
    return result;
}

/// A reader over the characters of a path's `d`.
class Reader {
public:
    explicit Reader(std::string_view data) : data(data) {}

    /// Skips whitespace and commas; whether anything is left.
    bool hasMore() {
        skipSeparators();
        return position < data.size();
    }

    bool atLetter() const { return position < data.size() && isLetter(data[position]); }
    char takeLetter() { return data[position++]; }
    size_t where() const { return position; }

    [[noreturn]] void fail(const std::string& what) const {
        throw MathProblem("svg path: " + what + " at " + std::to_string(position) + ": \"" +
                          std::string(data.substr(position, 12)) + "\"");
    }

    /// The number at the reader: [+-]?(digits[.digits]|.digits)([eE][+-]?digits)?.
    float number() {
        skipSeparators();
        size_t end = position;
        if (end < data.size() && (data[end] == '+' || data[end] == '-')) end++;
        if (end < data.size() && isDigit(data[end])) {
            while (end < data.size() && isDigit(data[end])) end++;
            if (end < data.size() && data[end] == '.') end++;
            while (end < data.size() && isDigit(data[end])) end++;
        } else if (end + 1 < data.size() && data[end] == '.' && isDigit(data[end + 1])) {
            end++;
            while (end < data.size() && isDigit(data[end])) end++;
        } else {
            fail("a number expected");
        }
        if (end < data.size() && (data[end] == 'e' || data[end] == 'E')) {
            size_t exponent = end + 1;
            if (exponent < data.size() && (data[exponent] == '+' || data[exponent] == '-')) exponent++;
            if (exponent < data.size() && isDigit(data[exponent])) {
                while (exponent < data.size() && isDigit(data[exponent])) exponent++;
                end = exponent;
            }
        }
        std::string text(data.substr(position, end - position));
        position = end;
        return std::strtof(text.c_str(), nullptr);
    }

    /// An arc flag: a single 0 or 1 (flags may touch the next number).
    bool flag() {
        skipSeparators();
        if (position >= data.size() || (data[position] != '0' && data[position] != '1')) fail("a flag expected");
        return data[position++] == '1';
    }

private:
    std::string_view data;
    size_t position = 0;

    void skipSeparators() {
        while (position < data.size() && (isSpace(data[position]) || data[position] == ',')) position++;
    }
};

/// Digits of a coordinate: rounded to `digits` decimals (halves away from zero, as JS's toFixed does on
/// the exact value), written fixed with trailing zeros and a bare point dropped, and a negative zero
/// written as 0.
std::string coordinateText(float value, int digits) {
    double scale = std::pow(10.0, digits);
    double rounded = std::round(static_cast<double>(value) * scale) / scale;
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%.*f", digits, rounded);
    std::string text = buffer;
    if (text.find('.') != std::string::npos) {
        while (text.back() == '0') text.pop_back();
        if (text.back() == '.') text.pop_back();
    }
    if (text == "-0") text = "0";
    return text;
}

} // namespace

std::vector<Path2D> PathSVG::parsed(std::string_view data) {
    std::vector<Path2D> result;
    Reader reader(data);
    char command = 0;
    Vector2 current, subpathStart;
    std::optional<Path2D> path;
    // The control point the next S or T reflects, and which command left it.
    std::optional<Vector2> lastControl;
    char lastCommand = 0;

    auto finish = [&]() {
        if (path && !path->segments().empty()) {
            path->close();
            result.push_back(*path);
        }
        path.reset();
        current = subpathStart;
    };
    auto begin = [&]() {
        if (!path) path = Path2D(current);
    };
    auto point = [&](bool relative) {
        float x = reader.number();
        float y = reader.number();
        return relative ? Vector2(current.x + x, current.y + y) : Vector2(x, y);
    };
    auto lineTo = [&](Vector2 to) {
        begin();
        path->addLine(to);
        current = to;
    };

    while (reader.hasMore()) {
        if (reader.atLetter()) command = reader.takeLetter();
        else if (command == 0) reader.fail("a command expected");
        char upper = char(std::toupper(static_cast<unsigned char>(command)));
        bool relative = command != upper;
        std::optional<Vector2> control;
        if (upper == 'M') {
            finish();
            current = subpathStart = point(relative);
            command = relative ? 'l' : 'L'; // further pairs after a moveto are lines
        } else if (upper == 'Z') {
            finish();
            command = 0;
        } else if (upper == 'L') {
            lineTo(point(relative));
        } else if (upper == 'H') {
            float x = reader.number();
            lineTo(Vector2(relative ? current.x + x : x, current.y));
        } else if (upper == 'V') {
            float y = reader.number();
            lineTo(Vector2(current.x, relative ? current.y + y : y));
        } else if (upper == 'C' || upper == 'S') {
            // S reflects the last cubic's second handle about the current point.
            Vector2 startHandle = upper == 'C' ? point(relative)
                                  : lastControl && (lastCommand == 'C' || lastCommand == 'S') ? current * 2 - *lastControl
                                                                                               : current;
            Vector2 endHandle = point(relative);
            Vector2 end = point(relative);
            begin();
            path->addCubic(startHandle, endHandle, end);
            control = endHandle;
            current = end;
        } else if (upper == 'Q' || upper == 'T') {
            Vector2 quadraticControl = upper == 'Q' ? point(relative)
                                       : lastControl && (lastCommand == 'Q' || lastCommand == 'T') ? current * 2 - *lastControl
                                                                                                    : current;
            Vector2 end = point(relative);
            Cubic raised = raisedQuadratic(current, quadraticControl, end);
            begin();
            path->addCubic(raised.startHandle, raised.endHandle, end);
            control = quadraticControl;
            current = end;
        } else if (upper == 'A') {
            float radiusX = reader.number();
            float radiusY = reader.number();
            float rotation = reader.number();
            bool large = reader.flag();
            bool sweep = reader.flag();
            Vector2 end = point(relative);
            bool straight = false;
            std::vector<Cubic> pieces = arcCubics(current, radiusX, radiusY, rotation, large, sweep, end, straight);
            begin();
            if (straight) path->addLine(end);
            for (const Cubic& piece : pieces) path->addCubic(piece.startHandle, piece.endHandle, piece.end);
            current = end;
        } else {
            reader.fail(std::string("unknown command ") + command);
        }
        lastControl = control;
        lastCommand = upper;
    }
    finish();
    return result;
}

std::string PathSVG::text(const std::vector<Path2D>& paths, int digits) {
    if (digits < 0 || digits > 9) throw MathProblem("svg path: digits must be between 0 and 9.");
    auto number = [&](float value) {
        if (!std::isfinite(value)) throw MathProblem("svg path: a coordinate is not finite.");
        return coordinateText(value, digits);
    };
    auto pair = [&](Vector2 point) { return number(point.x) + " " + number(point.y); };
    std::string result;
    for (const Path2D& path : paths) {
        if (path.segments().empty()) continue;
        result += "M" + pair(path.start);
        for (const Path2D::Segment& segment : path.segments()) {
            if (segment.isLine()) result += "L" + pair(segment.end());
            else result += "C" + pair(segment.startHandle) + " " + pair(segment.endHandle) + " " + pair(segment.end());
        }
        if (path.isClosed()) result += "Z";
    }
    return result;
}

} // namespace editor
