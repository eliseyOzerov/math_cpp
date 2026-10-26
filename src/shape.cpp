#include "curves/shape.hpp"

#include "js.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#pragma STDC FP_CONTRACT OFF

namespace curves {

Cubic bowed(Pt a, Pt b, double bulge) {
  Pt d = b - a, n{-d[1], d[0]};
  double h = bulge / 0.75;  // both handles moved h·n put the middle 0.75h·n out
  Pt p1 = lerp(a, b, 1.0 / 3), p2 = lerp(a, b, 2.0 / 3);
  return {a, Pt{p1[0] + h * n[0], p1[1] + h * n[1]}, Pt{p2[0] + h * n[0], p2[1] + h * n[1]}, b};
}

namespace {

/** The most a corner may turn once its sides are bowed: short of turning straight back, where the sides would cross. */
const double MAX_TURN = PI * 11 / 12;

}  // namespace

std::vector<double> fitBulges(const std::vector<Pt>& pts, const std::vector<double>& bulges) {
  std::vector<double> scale(bulges.size(), 1);
  for (size_t i = 1; i + 1 < pts.size(); i++) {
    Pt a = pts[i] - pts[i - 1], b = pts[i + 1] - pts[i];
    double phi = turn(a, b);
    auto turned = [&](double s) { return std::abs(phi + std::atan(4 * s * bulges[i - 1]) + std::atan(4 * s * bulges[i])); };
    if (turned(1) <= MAX_TURN) continue;
    double lo = 0, hi = 1;
    for (int k = 0; k < 40; k++) {
      double m = (lo + hi) / 2;
      if (turned(m) <= MAX_TURN) lo = m;
      else hi = m;
    }
    scale[i - 1] = js::min(scale[i - 1], lo);
    scale[i] = js::min(scale[i], lo);
  }
  std::vector<double> out;
  for (size_t i = 0; i < bulges.size(); i++) out.push_back(bulges[i] * scale[i]);
  return out;
}

std::vector<Cubic> softPolyline(const std::vector<Pt>& pts, const Softness& softness) {
  const int n = int(pts.size());
  if (n < 2) return {};
  const double radius = softness.radius;
  std::vector<double> given;
  for (int i = 0; i < n - 1; i++) {
    if (auto* b = std::get_if<double>(&softness.bulge)) given.push_back(*b);
    else {
      auto& bs = std::get<std::vector<double>>(softness.bulge);
      given.push_back(size_t(i) < bs.size() ? bs[i] : 0);
    }
  }
  std::vector<double> bulges = fitBulges(pts, given);
  std::vector<Cubic> sides;
  std::vector<double> lens;
  for (int i = 0; i < n - 1; i++) {
    sides.push_back(bowed(pts[i], pts[i + 1], bulges[i]));
    lens.push_back(dist(pts[i], pts[i + 1]));
  }
  // The room each inner corner i (1 … n−2) takes from its sides.
  auto room = [&](int s) { return s == 0 || s == n - 2 ? lens[s] : lens[s] / 2; };
  std::vector<double> cut;
  for (int i = 0; i < n; i++)
    cut.push_back(i == 0 || i == n - 1 ? 0 : js::max(0, js::min(1, radius)) * js::min(room(i - 1), room(i)));
  std::vector<Cubic> out;
  for (int i = 0; i < n - 1; i++) {
    const Cubic& s = sides[i];
    double L = js::orElse(lens[i], 1), t0 = cut[i] / L, t1 = 1 - cut[i + 1] / L;
    if (i > 0 && cut[i] > 1e-9) {
      // The corner at pts[i], from where the last side was cut to where this one starts.
      const Cubic& prev = sides[i - 1];
      double tp = 1 - cut[i] / js::orElse(lens[i - 1], 1);
      out.push_back(arcCubic(bez(prev, tp), bezD(prev, tp), bez(s, t0), bezD(s, t0)));
    }
    if (t1 - t0 > 1e-9) out.push_back(trim(s, t0, t1));
  }
  return out;
}

Path soft(const std::vector<Pt>& pts, const Softness& softness, Pt metric) {
  std::vector<Pt> P;
  for (Pt p : pts) P.push_back(toMetric(p, metric));
  std::vector<Cubic> segs = softPolyline(P, softness);
  for (Cubic& c : segs)
    for (Pt& p : c) p = fromMetric(p, metric);
  return Path::of(segs, false, metric);
}

Path oval(double x0, double y0, double x1, double y1, const OvalOptions& o) {
  double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const std::array<Pt, 4> at{Pt{cx, y0}, Pt{x1, cy}, Pt{cx, y1}, Pt{x0, cy}};  // bottom, right, top, left
  const int i = int(o.start);
  const Heading heading = o.start == Side::bottom || o.start == Side::top ? Heading::h : Heading::v;
  Path p = from(at[i]);
  for (int k = 0; k < 4; k++) {
    QuarterOptions q;
    q.tension = o.tension;
    if (!k) q.from = heading;
    p = p.quarter(at[(i + k + 1) % 4], q);
  }
  return p.close();
}

namespace {

std::string jsPt(Pt p) {
  auto num = [](double x) {
    char b[32];
    std::snprintf(b, sizeof b, "%.17g", x);
    return std::string(b);
  };
  return num(p[0]) + "," + num(p[1]);
}

}  // namespace

Path lobe(Pt a, Pt b, const LobeOptions& o, Pt metric) {
  // Worked in the metric's units, in a frame of the axis (u) and outward from it (n), and back.
  Pt A = toMetric(a, metric), B = toMetric(b, metric);
  double th = o.axis ? rad(*o.axis) : std::atan2(A[1] - B[1], A[0] - B[0]);
  Pt u{std::cos(th), std::sin(th)}, n{u[1], -u[0]};
  auto local = [&](Pt p) -> Pt { return {p[0] * n[0] + p[1] * n[1], p[0] * u[0] + p[1] * u[1]}; };
  auto world = [&](double on, double s) -> Pt { return {on * n[0] + s * u[0], on * n[1] + s * u[1]}; };
  Pt la = local(A), lb = local(B);
  double oa = la[0], sa = la[1], ob = lb[0], sb = lb[1], base = js::min(oa, ob);
  double far = (o.reachFrom == ReachFrom::start ? oa : base) + o.reach * metric[0];
  if (!(far > base) || !(sa > sb))
    throw std::invalid_argument("a lobe needs room: reach " + std::to_string(o.reach) + ", from " + jsPt(a) + " round to " + jsPt(b));
  int k = int(js::max(1, js::round(o.nodes)));
  double mid = (sa + sb) / 2, r = (sa - sb) / 2;
  std::vector<Pt> pts{A};
  for (int i = 0; i < k; i++) {
    double t = PI / 2 - PI * (i + 0.5) / k, c = std::cos(t), s = std::sin(t), m = js::max(std::abs(c), std::abs(s));
    pts.push_back(world(base + (far - base) * c / m, mid + r * s / m));
  }
  pts.push_back(B);
  // Running clockwise (out along the top, round, back along the bottom), outward is the left of travel. Sides lying
  // across the axis at either end stay straight, so the ink keeps touching its line.
  std::vector<double> ss, bulges;
  for (Pt p : pts) ss.push_back(local(p)[1]);
  for (size_t i = 1; i < ss.size(); i++) bulges.push_back(std::abs(ss[i] - ss[i - 1]) < 1e-9 ? 0 : o.bulge);
  std::vector<Cubic> segs = softPolyline(pts, Softness{o.radius, bulges});
  for (Cubic& c : segs)
    for (Pt& p : c) p = fromMetric(p, metric);
  return Path::of(segs, false, metric);
}

}  // namespace curves
