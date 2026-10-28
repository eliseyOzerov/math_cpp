// See include/curves/corners.hpp. Ported from fountain-ts/src/curves/corners.ts line for line.
#include "curves/corners.hpp"

#include "curves/boolean.hpp"
#include "js.hpp"

#include <algorithm>
#include <map>
#include <set>

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** Math.sign. */
double sign(double x) { return std::isnan(x) ? x : x > 0 ? 1 : x < 0 ? -1 : x; }

/** The points a piece uses. */
int npts(const Piece& q) { return q.line ? 2 : 4; }

}  // namespace

std::vector<Corner> corners(std::span<const Contour> cs, double minDeg) {
  std::vector<Corner> out;
  // Pieces that are all one point have no direction: they are passed over, the corner read between their neighbours.
  auto dot0 = [](const Piece& q) {
    for (int j = 0; j < npts(q); j++)
      if (!(dist(q.p[j], q.p[0]) < 1e-9)) return false;
    return true;
  };
  for (size_t ci = 0; ci < cs.size(); ci++) {
    const Contour& c = cs[ci];
    int n = int(c.size());
    for (int i = 0; i < n; i++) {
      const Piece& q = c[i];
      if (dot0(q)) continue;
      int h = (i + n - 1) % n;
      while (h != i && dot0(c[h])) h = (h + n - 1) % n;
      const Piece& prev = c[h];
      Pt a = endTangents(prev)[1], b = endTangents(q)[0];
      double t = turn(a, b);
      if (deg(std::abs(t)) < minDeg) continue;
      // Outside the outline is on the left, and its wedge is π − turn: reflex at a convex corner, under π at a concave one.
      double outside = deg(PI - t);
      out.push_back({int(ci), i, q.first(), a, b, t, outside});
    }
  }
  return out;
}

std::vector<Corner> pits(std::span<const Contour> cs, double maxDeg) {
  std::vector<Corner> out;
  for (auto& k : corners(cs))
    if (k.turn > 0 && k.outside < maxDeg) out.push_back(k);
  return out;
}

std::vector<Corner> peaks(std::span<const Contour> cs, double minDeg) {
  std::vector<Corner> out;
  for (auto& k : corners(cs))
    if (k.turn < 0 && deg(-k.turn) > minDeg) out.push_back(k);
  return out;
}

namespace {

/** The piece with `s0` of its length cut off its start and `s1` off its end. */
Piece cutPiece(const Piece& q, double s0, double s1) {
  double L = pieceLength(q);
  if (q.line) return Piece::of(lerp(q.p[0], q.p[1], s0 / L), lerp(q.p[1], q.p[0], s1 / L));
  return Piece::of(trim(q.p, tAtLength(q.p, s0, L), tAtLength(q.p, L - s1, L)));
}

}  // namespace

std::vector<Contour> replaceCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& reach,
                                    const MakeCorner& make, CornerOpts opts) {
  double minDeg = opts.minDeg, share = opts.share;
  auto all = corners(cs, minDeg);
  std::vector<Contour> result;
  for (size_t ci = 0; ci < cs.size(); ci++) {
    const Contour& c = cs[ci];
    int n = int(c.size());
    std::vector<Corner> ks;
    for (auto& k : all)
      if (k.contour == int(ci)) ks.push_back(k);
    std::vector<double> lens;
    for (auto& q : c) lens.push_back(pieceLength(q));
    if (ks.empty()) { result.push_back(c); continue; }
    // A corner's sides run along the contour to the corners either side of it, over the pieces between that run on
    // smoothly: a merge splits the outline wherever it likes, and capping the cut at the piece next to the corner
    // made a well as big as one join's and a sliver at the next, by where the split fell (A's bar into its legs).
    std::set<int> uniq;
    for (auto& k : ks) uniq.insert(k.at);
    std::vector<int> stops(uniq.begin(), uniq.end());
    auto run = [&](int from, int to) {
      double s = 0;
      int j = from;
      do { s += lens[j]; j = (j + 1) % n; } while (j != to);
      return s;
    };
    // What each piece loses at its start and end, the pieces cut away whole, and what goes in at each corner.
    std::vector<double> s0(n, 0), s1(n, 0);
    std::vector<char> gone(n, 0);
    std::map<int, std::vector<Piece>> fills;
    for (auto& k : ks) {
      double r = reach(k);
      if (!(r > 0)) continue;
      int i = k.at;
      int si = int(std::find(stops.begin(), stops.end(), i) - stops.begin());
      int next = stops[(si + 1) % stops.size()], prev = stops[(si + stops.size() - 1) % stops.size()];
      double d = js::min(js::min(r, share * run(i, next)), share * run(prev, i));
      if (!(d > 1e-9)) continue;
      // Forward over the pieces from the corner, then back over those before it.
      int f = i;
      double rem = d;
      for (int step = 0; step < n && rem > lens[f] - 1e-9 && rem > 1e-9; step++) { gone[f] = 1; rem -= lens[f]; f = (f + 1) % n; }
      s0[f] = js::max(0, rem);
      int b = (i + n - 1) % n;
      double back = d;
      for (int step = 0; step < n && back > lens[b] - 1e-9 && back > 1e-9; step++) { gone[b] = 1; back -= lens[b]; b = (b + n - 1) % n; }
      s1[b] = js::max(0, back);
      // The tangents come from each side cut back at this corner only: cut at both ends, a side as long as the two
      // cuts (a square end at softness 1) is left a point, with no direction.
      Piece pq = cutPiece(c[b], 0, s1[b]), here = cutPiece(c[f], s0[f], 0);
      fills[i] = make(pq.last(), endTangents(pq)[1], here.first(), endTangents(here)[0], k);
    }
    if (fills.empty()) { result.push_back(c); continue; }
    Contour out;
    for (int i = 0; i < n; i++) {
      auto it = fills.find(i);
      if (it != fills.end()) out.insert(out.end(), it->second.begin(), it->second.end());
      if (!gone[i] && lens[i] - s0[i] - s1[i] > 1e-6) out.push_back(cutPiece(c[i], s0[i], s1[i]));
    }
    result.push_back(stitch(out));
  }
  return result;
}

namespace {

/** How far a fillet of radius `r` reaches along the sides of a corner turning by `turn`. */
double filletReach(double r, double turn) { return r * std::tan(js::min(std::abs(turn), PI * 0.999) / 2); }

}  // namespace

std::vector<Contour> roundCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& radius,
                                  CornerOpts opts) {
  return replaceCorners(
    cs, [&](const Corner& k) { return filletReach(radius(k), k.turn); },
    [](Pt a, Pt ta, Pt b, Pt tb, const Corner&) { return std::vector<Piece>{Piece::of(arcCubic(a, ta, b, tb))}; }, opts);
}

std::vector<Contour> roundCorners(std::span<const Contour> cs, double radius, CornerOpts opts) {
  return roundCorners(cs, [radius](const Corner&) { return radius; }, opts);
}

CornerFrame cornerFrame(Pt a, Pt b, Pt p) {
  Pt m = lerp(a, b, 0.5), ex = (b - a) * 0.5;
  // A corner on its chord (no turn) has no height: a unit square to the chord stands in.
  Pt ey = p - m;
  if (std::abs(ex[0] * ey[1] - ex[1] * ey[0]) < 1e-9) ey = {-ex[1], ex[0]};
  return {m, ex, ey, ex[0] * ey[1] - ex[1] * ey[0]};
}
Pt CornerFrame::to(Pt q) const { return {m[0] + ex[0] * q[0] + ey[0] * q[1], m[1] + ex[1] * q[0] + ey[1] * q[1]}; }
Pt CornerFrame::from(Pt w) const {
  Pt d = w - m;
  return {(d[0] * ey[1] - d[1] * ey[0]) / det, (ex[0] * d[1] - ex[1] * d[0]) / det};
}

std::vector<Contour> shapeCorners(std::span<const Contour> cs, const std::function<double(const Corner&)>& size,
                                  const std::function<CornerShape(const Corner&)>& shape,
                                  const std::function<void(const Corner&, Pt, Pt)>& seen) {
  return replaceCorners(
    cs, [&](const Corner& k) { return filletReach(size(k), k.turn); },
    [&](Pt a, Pt ta, Pt b, Pt tb, const Corner& k) -> std::vector<Piece> {
      CornerShape sh = shape(k);
      double s = sh.angle;
      const auto& profile = sh.profile;
      if (seen) seen(k, a, b);
      if (sh.curve && !sh.curve->empty()) {
        CornerFrame fr = cornerFrame(a, b, k.p);
        std::vector<Piece> out;
        for (auto& q : *sh.curve) {
          Piece m = q;
          for (int j = 0; j < npts(q); j++) m.p[j] = fr.to(q.p[j]);
          out.push_back(m);
        }
        out[0].p[0] = a;
        Piece& last = out.back();
        last.p[last.line ? 1 : 3] = b;
        return out;
      }
      if (!js::truthy(s) && !profile) return {Piece::of(arcCubic(a, ta, b, tb))};
      Pt u = norm(b - a);
      double chord = dist(a, b);
      // The fillet leaves at `f` off its chord, bulging towards the corner; a larger angle bulges further that way,
      // which is away from the ink at a peak and into it at a pit.
      double f = turn(u, ta), towards = k.turn < 0 ? s : -s;
      double al = clamp(f + sign(f) * towards * PI / 2, -PI * 0.98, PI * 0.98);
      if (std::abs(al) < 1e-6) return {Piece::of(a, b)};
      if (profile && !profile->empty()) {
        // The arc leaving at α bulges (chord/2)·tan(α/2) off its chord, to the left of it for α > 0.
        double bulge = chord / 2 * std::tan(al / 2);
        Pt left{-u[1], u[0]};
        double top = box(*profile)[1][1];
        auto to = [&](Pt p) -> Pt {
          double x = (p[0] + 1) / 2 * chord, y = top > 1e-9 ? p[1] / top * bulge : 0;
          return {a[0] + u[0] * x + left[0] * y, a[1] + u[1] * x + left[1] * y};
        };
        std::vector<Piece> out;
        for (auto& q : *profile) {
          Piece m = q;
          for (int j = 0; j < npts(q); j++) m.p[j] = to(q.p[j]);
          out.push_back(m);
        }
        out[0].p[0] = a;
        Piece& last = out.back();
        last.p[last.line ? 1 : 3] = b;
        return out;
      }
      std::vector<Piece> out;
      for (auto& c : arcCubics(a, b, al)) out.push_back(Piece::of(c));
      return out;
    });
}

namespace caps {

std::vector<Piece> triangle(double h) {
  return {Piece::of({-1, 0}, {0, h}), Piece::of({0, h}, {1, 0})};
}
std::vector<Piece> rect(double h) {
  return {Piece::of({-1, 0}, {-1, h}), Piece::of({-1, h}, {1, h}), Piece::of({1, h}, {1, 0})};
}
std::vector<Piece> arc(double h) {
  double k = KAPPA;
  return {Piece::of(Cubic{Pt{-1, 0}, Pt{-1, k * h}, Pt{-k, h}, Pt{0, h}}),
          Piece::of(Cubic{Pt{0, h}, Pt{k, h}, Pt{1, k * h}, Pt{1, 0}})};
}

}  // namespace caps

namespace {

/** Where a ray from `o` along `d` first meets the contours, within `far`. */
std::optional<double> reachOf(std::span<const Contour> cs, Pt o, Pt d, double far) {
  Cubic ray = lineCubic(o, o + d * far);
  std::optional<double> best;
  for (auto& c : cs)
    for (auto& q : c) {
      auto xs = crossings(ray, asCubic(q));
      if (!xs) continue;
      for (auto& x : *xs) {
        double t = x[0];
        if (!best || t * far < *best) best = t * far;
      }
    }
  return best;
}

Piece line(Pt a, Pt b) { return Piece::of(a, b); }

}  // namespace

std::vector<Contour> capped(std::span<const Contour> cs, const EndFrame& end, const Cap& cap) {
  const auto& segs = cap.shape;
  if (segs.empty()) return std::vector<Contour>(cs.begin(), cs.end());
  Pt t = norm(end.t), right{t[1], -t[0]};
  Box bx = box(segs);
  double x0 = bx[0][0], y0 = bx[0][1], x1 = bx[1][0], y1 = bx[1][1];
  double k = 2 / js::orElse(x1 - x0, 1), a = clamp(cap.align.value_or(1), -1, 1);
  double lift = (1 - a) / 2 * (y1 - y0) * k;
  // The stroke's middle and half-width across it at a depth behind the end, measured on the outline.
  struct Across { Pt c; double r; };
  auto across = [&](double depth) {
    Pt o = end.p + t * -depth;
    double rr = reachOf(cs, o, right, 4 * end.r).value_or(end.r), rl = reachOf(cs, o, right * -1, 4 * end.r).value_or(end.r);
    return Across{o + right * ((rr - rl) / 2), (rl + rr) / 2};
  };
  // Local units: u across (−1…1 side to side), v out along the stroke (0 at its end), both in half-widths. The cap
  // takes over the stroke a little behind where it starts (the end, or as deep as it carves), at `bottom`: far
  // enough that the stroke's own end, cut square to its end tangent, lies clear past that line. There the stroke is
  // measured, and the way it runs taken from its middle a little deeper still, so the cap's sides meet the stroke's
  // on that line and go on along it (a curved stroke's end tangent would aim them off).
  double W = 1.5, tip = (y1 - y0) * k - lift, low = -lift, bottom = low - 0.25, floor = bottom - 0.1;
  double top = js::max(tip, 0) + 0.5, depth = -bottom * end.r;
  Across m = across(depth), back = across(depth + end.r / 2);
  double r = m.r;
  Pt dir = norm(m.c - back.c);
  Pt tt = std::isfinite(dir[0]) && (js::truthy(dir[0]) || js::truthy(dir[1])) ? dir : t;
  Pt rt{tt[1], -tt[0]};
  auto local = [&](Pt p) -> Pt { return {(p[0] - (x0 + x1) / 2) * k, (p[1] - y0) * k - lift}; };
  auto world = [&](Pt p) -> Pt { return m.c + (rt * (p[0] * r) + tt * ((p[1] - bottom) * r)); };
  Contour curve;
  for (auto& q : segs) {
    Piece w = q;
    for (int j = 0; j < npts(q); j++) w.p[j] = world(local(q.p[j]));
    curve.push_back(w);
  }
  Pt s = local(segs[0].p[0]), e = local(segs.back().last());
  double side = js::orElse(sign(e[0] - s[0]), 1);
  // The cap: under the curve, between the stroke's sides: the curve, then out level to the side (if it ends inside
  // it), down to the band's bottom, in a little and down past it, across and back up. One closed region, so no edge
  // of it lies on another input's (the rect's sides on the sides of a separate continuation broke the boolean); it
  // narrows below the band, so its sides cross the band's bottom and never run along a curved stroke's sides.
  std::vector<Pt> ps{e, {side, e[1]}, {side, bottom}, {side * 0.8, floor}, {-side * 0.8, floor}, {-side, bottom}, {-side, s[1]}, s};
  Contour capC = curve;
  for (size_t i = 0; i + 1 < ps.size(); i++)
    if (dist(ps[i], ps[i + 1]) > 1e-9) capC.push_back(line(world(ps[i]), world(ps[i + 1])));
  std::vector<Pt> bp{{-W, bottom}, {-W, top}, {W, top}, {W, bottom}};
  Contour band;
  for (int i = 0; i < 4; i++) band.push_back(line(world(bp[i]), world(bp[(i + 1) % 4])));
  // The stroke cut off at the band, then the cap in the band added. In one operation the stroke's sides inside the
  // band, a hair from the cap's sides on a curved stroke, both counted as edges and the loops did not close.
  // Where either fails to close, the end is left as it was rather than half cut.
  std::vector<std::vector<Contour>> in1{std::vector<Contour>(cs.begin(), cs.end()), {band}};
  auto cut = tryBoolean(in1, [](const std::vector<int>& w) { return w[0] != 0 && w[1] == 0; });
  if (cut) {
    std::vector<std::vector<Contour>> in2{*cut, {capC}, {band}};
    auto out = tryBoolean(in2, [](const std::vector<int>& w) { return w[0] != 0 || (w[1] != 0 && w[2] != 0); });
    if (out) return *out;
  }
  return std::vector<Contour>(cs.begin(), cs.end());
}

}  // namespace curves
