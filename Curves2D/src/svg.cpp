// See include/curves/svg.hpp. Ported from fountain-ts/src/curves/svg.ts line for line.
#include "curves/svg.hpp"

#include "curves/bezn.hpp"
#include "js.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

// No fused multiply-adds: JS rounds every operation.
#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r'; }
bool isDigit(char c) { return c >= '0' && c <= '9'; }
bool isAlpha(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

/** An elliptical arc from `a` to `b` as SVG gives it (radii, rotation in degrees, flags), as cubics: the endpoint form
 *  turned to the centre form (SVG 1.1 implementation notes, F.6.5), radii grown when too small to reach. */
std::vector<Piece> arc(Pt a, double rx, double ry, double rot, bool large, bool sweep, Pt b) {
  if (curves::hypot(b[0] - a[0], b[1] - a[1]) < 1e-12) return {};
  rx = std::abs(rx);
  ry = std::abs(ry);
  if (!js::truthy(rx) || !js::truthy(ry)) return {Piece::of(a, b)};
  double phi = rot * PI / 180, cos = std::cos(phi), sin = std::sin(phi);
  double dx = (a[0] - b[0]) / 2, dy = (a[1] - b[1]) / 2, x1 = cos * dx + sin * dy, y1 = -sin * dx + cos * dy;
  double grow = x1 * x1 / (rx * rx) + y1 * y1 / (ry * ry);
  if (grow > 1) {
    rx *= std::sqrt(grow);
    ry *= std::sqrt(grow);
  }
  double num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1, den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
  double k = (large == sweep ? -1 : 1) * std::sqrt(js::max(0, num / den)), cx1 = k * rx * y1 / ry,
         cy1 = -k * ry * x1 / rx;
  double cx = cos * cx1 - sin * cy1 + (a[0] + b[0]) / 2, cy = sin * cx1 + cos * cy1 + (a[1] + b[1]) / 2;
  auto ang = [](double ux, double uy, double vx, double vy) { return std::atan2(ux * vy - uy * vx, ux * vx + uy * vy); };
  double t0 = ang(1, 0, (x1 - cx1) / rx, (y1 - cy1) / ry);
  double dt = ang((x1 - cx1) / rx, (y1 - cy1) / ry, (-x1 - cx1) / rx, (-y1 - cy1) / ry);
  if (!sweep && dt > 0) dt -= 2 * PI;
  if (sweep && dt < 0) dt += 2 * PI;
  double n = std::ceil(std::abs(dt) / (PI / 2) - 1e-9), step = dt / n, h = 4.0 / 3 * std::tan(step / 4);
  auto on = [&](double t) -> Pt {
    return {cx + rx * std::cos(t) * cos - ry * std::sin(t) * sin, cy + rx * std::cos(t) * sin + ry * std::sin(t) * cos};
  };
  auto tan = [&](double t) -> Pt {
    return {-rx * std::sin(t) * cos - ry * std::cos(t) * sin, -rx * std::sin(t) * sin + ry * std::cos(t) * cos};
  };
  std::vector<Piece> out;
  // Array.from({ length: n }): a NaN or non-positive n makes none.
  int count = n > 0 ? int(n) : 0;
  for (int j = 0; j < count; j++) {
    double s = t0 + j * step, e = s + step;
    Pt p = j ? on(s) : a, q = j == count - 1 ? b : on(e), ts = tan(s), te = tan(e);
    out.push_back(Piece::of(Cubic{p, {p[0] + h * ts[0], p[1] + h * ts[1]}, {q[0] - h * te[0], q[1] - h * te[1]}, q}));
  }
  return out;
}

}  // namespace

std::vector<Contour> parseSvgPath(const std::string& d) {
  std::vector<Contour> out;
  size_t i = 0;
  char cmd = 0, lastCmd = 0;
  Pt cur{0, 0}, start{0, 0};
  bool hasLast = false;
  Pt last{};
  Contour pieces;
  auto skip = [&] { while (i < d.size() && (isSpace(d[i]) || d[i] == ',')) i++; };
  auto num = [&]() -> double {
    skip();
    // The sticky regex [+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?, at i.
    size_t j = i;
    if (j < d.size() && (d[j] == '+' || d[j] == '-')) j++;
    if (j < d.size() && isDigit(d[j])) {
      while (j < d.size() && isDigit(d[j])) j++;
      if (j < d.size() && d[j] == '.') j++;
      while (j < d.size() && isDigit(d[j])) j++;
    } else if (j + 1 < d.size() && d[j] == '.' && isDigit(d[j + 1])) {
      j++;
      while (j < d.size() && isDigit(d[j])) j++;
    } else {
      throw std::invalid_argument("svg path: a number expected at " + std::to_string(i) + ": \"" + d.substr(i, 12) + "\"");
    }
    if (j < d.size() && (d[j] == 'e' || d[j] == 'E')) {
      size_t m = j + 1;
      if (m < d.size() && (d[m] == '+' || d[m] == '-')) m++;
      if (m < d.size() && isDigit(d[m])) {
        while (m < d.size() && isDigit(d[m])) m++;
        j = m;
      }
    }
    std::string text = d.substr(i, j - i);
    i = j;
    return std::strtod(text.c_str(), nullptr);
  };
  auto flag = [&]() -> bool {
    skip();
    if (i >= d.size()) { i++; throw std::invalid_argument("svg path: a flag expected at " + std::to_string(i - 1)); }
    char c = d[i++];
    if (c != '0' && c != '1') throw std::invalid_argument("svg path: a flag expected at " + std::to_string(i - 1));
    return c == '1';
  };
  auto close = [&] {
    if (pieces.empty()) return;
    if (curves::hypot(cur[0] - start[0], cur[1] - start[1]) > 1e-9) pieces.push_back(Piece::of(cur, start));
    out.push_back(std::move(pieces));
    pieces = {};
    cur = start;
  };
  auto pt = [&](bool rel) -> Pt {
    double x = num();
    double y = num();
    return rel ? Pt{cur[0] + x, cur[1] + y} : Pt{x, y};
  };
  while (true) {
    skip();
    if (i >= d.size()) break;
    if (isAlpha(d[i])) cmd = d[i++];
    else if (!cmd) throw std::invalid_argument("svg path: a command expected at " + std::to_string(i));
    char C = char(std::toupper(static_cast<unsigned char>(cmd)));
    bool rel = cmd != C;
    bool hasCtl = false;
    Pt ctl{};
    if (C == 'M') {
      close();
      Pt p = pt(rel);
      cur = start = p;
      cmd = rel ? 'l' : 'L';
    } else if (C == 'Z') {
      close();
      cmd = 0;
    } else if (C == 'L') {
      Pt p = pt(rel);
      pieces.push_back(Piece::of(cur, p));
      cur = p;
    } else if (C == 'H') {
      double x = num();
      Pt p{rel ? cur[0] + x : x, cur[1]};
      pieces.push_back(Piece::of(cur, p));
      cur = p;
    } else if (C == 'V') {
      double y = num();
      Pt p{cur[0], rel ? cur[1] + y : y};
      pieces.push_back(Piece::of(cur, p));
      cur = p;
    } else if (C == 'C' || C == 'S') {
      // S reflects the last cubic's second handle.
      Pt a = C == 'C' ? pt(rel) : hasLast && (lastCmd == 'C' || lastCmd == 'S') ? Pt{2 * cur[0] - last[0], 2 * cur[1] - last[1]} : cur;
      Pt b = pt(rel);
      Pt p = pt(rel);
      pieces.push_back(Piece::of(Cubic{cur, a, b, p}));
      ctl = b;
      hasCtl = true;
      cur = p;
    } else if (C == 'Q' || C == 'T') {
      Pt a = C == 'Q' ? pt(rel) : hasLast && (lastCmd == 'Q' || lastCmd == 'T') ? Pt{2 * cur[0] - last[0], 2 * cur[1] - last[1]} : cur;
      Pt p = pt(rel);
      std::vector<Pt> r = raise({cur, a, p});
      pieces.push_back(Piece::of(Cubic{r[0], r[1], r[2], r[3]}));
      ctl = a;
      hasCtl = true;
      cur = p;
    } else if (C == 'A') {
      double rx = num();
      double ry = num();
      double rot = num();
      bool large = flag();
      bool sweep = flag();
      Pt p = pt(rel);
      for (auto& q : arc(cur, rx, ry, rot, large, sweep, p)) pieces.push_back(q);
      cur = p;
    } else {
      throw std::invalid_argument(std::string("svg path: unknown command ") + cmd);
    }
    hasLast = hasCtl;
    last = ctl;
    lastCmd = C;
  }
  close();
  return out;
}

std::string svgPath(const std::vector<Contour>& cs, int digits) {
  auto n = [&](double v) {
    if (!std::isfinite(v) || std::abs(v) >= 1e21) return js::string(v);
    return js::string(std::strtod(js::toFixed(v, digits).c_str(), nullptr));
  };
  auto f = [&](Pt p) { return n(p[0]) + " " + n(p[1]); };
  std::string out;
  for (auto& c : cs) {
    if (c.empty()) continue;
    out += "M" + f(c[0].p[0]);
    for (auto& q : c) out += q.line ? "L" + f(q.p[1]) : "C" + f(q.p[1]) + " " + f(q.p[2]) + " " + f(q.p[3]);
    out += "Z";
  }
  return out;
}

}  // namespace curves
