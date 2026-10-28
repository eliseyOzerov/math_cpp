// See include/curves/ink.hpp. Ported from fountain-ts/src/ink/ink.ts line for line.
#include "curves/ink.hpp"

#include "js.hpp"
#include "jsmath.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

#include <algorithm>
#include <array>
#include <cmath>

namespace curves {

namespace {

/** A cut closer than this to the stroke's own direction would run on without end; it is cut square instead. */
const double SQUARE_CUT = rad(15);
/** How far a taper or flare eases in from its end, in font units, when the cut gives no `reach`. */
constexpr double TAPER_REACH = 150;
/** The run-on's points lie at most this far apart (font units): close enough that the hulls follow the line. */
constexpr double RUN_ON_STEP = 8;
/** The nib's heading turns at most this much between copies when it sweeps round a sharp corner. */
const double SWEEP_STEP = rad(6);
/** How near (font units) an end must lie to one of the `levels` to be cut level there. */
constexpr double ON_HEIGHT = 0.5;

constexpr InkEnd ENDS[2] = {InkEnd::start, InkEnd::end};
constexpr int at(InkEnd e) { return e == InkEnd::start ? 0 : 1; }

/** The stroke's curves along it, sorted. */
struct Curves {
  std::optional<Keys<double>> size, rotate, follow, offset;
  std::optional<Keys<Shape>> shapes;
};
/** What the ends ask for: each end's own terminal (none for 'auto' or none), its taper and how far that reaches. */
struct Ends {
  std::optional<Cut> term[2];
  double taperv[2] = {0, 0}, reachv[2] = {TAPER_REACH, TAPER_REACH};
  double taper(InkEnd e) const { return taperv[at(e)]; }
  double reach(InkEnd e) const { return reachv[at(e)]; }
};

Ends endsOf(const InkStroke& st) {
  Ends r;
  for (InkEnd e : ENDS) {
    std::optional<Cut> c;
    if (st.cuts) c = e == InkEnd::start ? st.cuts->start : st.cuts->end;
    int i = at(e);
    if (c && c->mode != CutMode::auto_) r.term[i] = c;
    r.taperv[i] = js::orElse(c && c->taper ? *c->taper : 0, 0);
    r.reachv[i] = js::orElse(c && c->reach ? *c->reach : 0, TAPER_REACH);
  }
  return r;
}

/** Furthest a stamp reaches from its centre, for the run-ons and how far a plane trims. */
double reachOf(const Nib& N, const Curves& cv, double scale, const Ends& ends) {
  double hy;
  if (cv.shapes) {
    hy = -INFINITY;
    for (const auto& k : *cv.shapes) hy = js::max(hy, curves::hypot(k.v.w * N.w, k.v.h * N.h));
  } else {
    hy = curves::hypot(1 * N.w, 1 * N.h);
  }
  double flare = 1;
  for (InkEnd e : ENDS) flare = js::max(flare, 1 + ends.taper(e));
  double sz = 1, off = 0;
  if (cv.size) for (const auto& p : *cv.size) sz = js::max(sz, p.v);
  if (cv.offset) for (const auto& p : *cv.offset) off = js::max(off, std::abs(p.v));
  return hy / 2 * scale * sz * (1 + off) * flare;
}

struct Sampled {
  std::vector<Pt> pts;
  std::vector<std::optional<double>> heads;
  std::vector<int> corners;
};

/** The centreline as points, `perSpan` per segment (or by `spacing`), and the indices where segments meet.
 *  Where the skeleton turns sharply between segments (a pointed or boxed bowl), the nib sweeps round the corner in
 *  place: copies of the corner point whose `heads` (directions along the stroke) turn from the one segment's to the
 *  next's, so the outer edge is the nib's own arc. The sweep is a span of its own, from one corner index to the next. */
Sampled sample(const std::vector<Cubic>& segs, double perSpan, std::optional<double> spacing) {
  Sampled s;
  auto& pts = s.pts; auto& heads = s.heads; auto& corners = s.corners;
  // Not endTangents and turn: those skip a handle shorter than 1e-12 (this, 1e-6) and fall back to the next handle
  // (this, the chord); tried, it changed the corners found in fonts/Wave's W and moved other outlines by 1e-3.
  auto wrap = [](double a) { return js::atan2(js::sin(a), js::cos(a)); };
  auto headOf = [](const Cubic& p, bool end) {
    Pt v = end ? p[3] - p[2] : p[1] - p[0], w = p[3] - p[0];
    return len(v) > 1e-6 ? js::atan2(v[1], v[0]) : js::atan2(w[1], w[0]);
  };
  for (size_t k = 0; k < segs.size(); k++) {
    const Cubic& p = segs[k];
    if (k > 0) {
      double a0 = headOf(segs[k - 1], true), t = wrap(headOf(p, false) - a0);
      double m = std::abs(t) > CORNER ? std::ceil(std::abs(t) / SWEEP_STEP) : 0;
      int last = int(pts.size()) - 1;
      if (m) heads[last] = a0;
      corners.push_back(last);
      if (m) corners.push_back(last + int(m));
      for (double j = 1; j <= m; j++) { pts.push_back(pts.back()); heads.push_back(a0 + t * j / m); }
    }
    double m = spacing && js::truthy(*spacing) ? js::max(2, std::ceil(arcLength(p) / *spacing)) : perSpan;
    for (double i = pts.size() ? 1 : 0; i <= m; i++) { pts.push_back(bez(p, i / m)); heads.push_back(std::nullopt); }
  }
  return s;
}

/** Direction of a cut line, or none for the stroke's own end (square, or round with round ends). */
std::optional<Pt> cutDirection(const std::optional<Cut>& cut) {
  if (!cut || (cut->mode != CutMode::level && cut->mode != CutMode::angle)) return std::nullopt;
  return js::fromAngle(cut->mode == CutMode::level ? 0 : rad(cut->deg.value_or(NAN)));
}

struct RunOns {
  std::vector<EndPlane> planes;
  double reach[2] = {0, 0};
  int lead = 0;
};

/** The ends' run-ons and planes, in place on `segs`, `pts`, `ts` and `heads`. A cut end (its own, or bare on one of
 *  the `levels`; none when `levels` is null, cuts off) runs the centreline on until the stamps reach the cut line,
 *  as a line segment of its own, and a plane trims there; any other end is trimmed square across the stroke unless
 *  it is 'nib' or 'round', or bare with nib caps. `reach` is how far from its end each plane trims; `lead` the
 *  run-on points put before the start. A closed stroke has no ends. */
RunOns runOns(const InkStroke& st, std::vector<Cubic>& segs, std::vector<Pt>& pts, std::vector<double>& ts,
              std::vector<std::optional<double>>& heads, const Ends& ends, double big, Caps caps,
              const std::optional<std::vector<double>>& levels) {
  Pt S = st.S ? *st.S : segs[0][0], E = st.E ? *st.E : segs.back()[3];
  auto unit = [](Pt v) -> std::optional<Pt> { return len(v) > 1e-9 ? std::optional(norm(v)) : std::nullopt; };
  Pt chord = unit({E[0] - S[0], E[1] - S[1]}).value_or(Pt{1, 0});
  RunOns r;
  if (st.closed) return r;
  for (InkEnd end : ENDS) {
    bool first = end == InkEnd::start;
    const Cubic& seg = first ? segs[0] : segs.back();
    Pt P = first ? seg[0] : seg[3];
    // Outward tangent at the end.
    Pt t = unit(first ? Pt{seg[0][0] - seg[1][0], seg[0][1] - seg[1][1]} : Pt{seg[3][0] - seg[2][0], seg[3][1] - seg[2][1]})
               .value_or(first ? Pt{-chord[0], -chord[1]} : chord);
    const std::optional<Cut>& term = ends.term[at(end)];
    bool hasMode = term.has_value();
    CutMode mode = hasMode ? term->mode : CutMode::auto_;
    std::optional<Pt> dir;
    if (!levels) dir = std::nullopt;
    else if (hasMode) dir = cutDirection(term);
    else {
      bool onLevel = false;
      for (double y : *levels) if (std::abs(P[1] - y) < ON_HEIGHT) { onLevel = true; break; }
      if (onLevel) dir = Pt{1, 0};
    }
    double sin = dir ? std::abs(cross(*dir, t)) : 1;
    if (dir && sin >= js::sin(SQUARE_CUT)) {
      double more = big * std::sqrt(1 - sin * sin) / sin + big;
      double steps = js::max(1, std::ceil(more / RUN_ON_STEP));
      std::vector<Pt> on;
      for (double i = 1; i <= steps; i++) on.push_back({P[0] + t[0] * more * i / steps, P[1] + t[1] * more * i / steps});
      if (first) {
        std::reverse(on.begin(), on.end());
        pts.insert(pts.begin(), on.begin(), on.end());
        ts.insert(ts.begin(), on.size(), 0.0);
        heads.insert(heads.begin(), on.size(), std::nullopt);
        r.lead = int(on.size());
      } else {
        pts.insert(pts.end(), on.begin(), on.end());
        ts.insert(ts.end(), on.size(), 1.0);
        heads.insert(heads.end(), on.size(), std::nullopt);
      }
      // Normal of the cut line, pointing back into the stroke.
      Pt n = {-(*dir)[1], (*dir)[0]};
      if (dot(n, t) > 0) n = {-n[0], -n[1]};
      // As in the TS: `on` was reversed in place for a start, so its last is the run-on point nearest the end.
      Pt edge = on.back();
      if (first) segs.insert(segs.begin(), lineCubic(edge, P)); else segs.push_back(lineCubic(P, edge));
      r.planes.push_back({{P, n}, end});
      r.reach[at(end)] = more + 2 * big;
    } else if ((hasMode && mode == CutMode::flat) || (!(hasMode && mode == CutMode::nib) && !(hasMode && mode == CutMode::round) && caps != Caps::nib)) {
      r.planes.push_back({{P, Pt{-t[0], -t[1]}}, end});
      r.reach[at(end)] = 2 * big;
    }
  }
  return r;
}

struct Dir { double raw, phi; };

/** Each point's heading along the stroke, `raw` (set by a sweep, or from its neighbours) and `phi` unwound from
 *  the one before, so the nib turns smoothly with the stroke. */
std::vector<Dir> headings(const std::vector<Pt>& pts, const std::vector<std::optional<double>>& heads, bool closed) {
  int n = int(pts.size());
  std::vector<Dir> dirs;
  std::optional<double> prev;
  for (int i = 0; i < n; i++) {
    Pt a = pts[i > 0 ? i - 1 : closed ? n - 2 : 0], b = pts[i < n - 1 ? i + 1 : closed ? 1 : n - 1];
    double raw = heads[i] ? *heads[i] : js::atan2(b[1] - a[1], b[0] - a[0]);
    double phi = !prev ? halfTurn(raw) : raw;
    if (prev) { while (phi - *prev > PI) phi -= 2 * PI; while (phi - *prev <= -PI) phi += 2 * PI; }
    prev = phi;
    dirs.push_back({raw, phi});
  }
  return dirs;
}

/** What placing a stamp reads. */
struct Stamping {
  Nib N;
  double scale, perCorner;
  Model model;
  Curves curves;
  Ends ends;
  const std::vector<double>& ts;
  double total;
  const std::vector<Pt>& pts;
  const std::vector<Dir>& dirs;
};

struct Placed {
  std::vector<Pt> stamp;
  Frame frame;
  double width;
};

using Discs = std::array<std::optional<double>, 2>;

/** The nib at point `i`: shaped, sized, turned and moved across by the stroke's curves. Terminals along the stroke:
 *  a taper (below zero) or flare scales the nib by 1 + `taper` at its end, easing in over its reach; a round end
 *  eases the stamp into a disc as wide as the ink across the stroke there (`discs`), over that width. A pointed pen
 *  stamps a disc as wide as the nib's ink across, or the `sweep` width round a corner. */
Placed place(const Stamping& S, size_t i, const Discs& discs, const std::vector<std::optional<double>>& sweep) {
  const Nib& N = S.N;
  double raw = S.dirs[i].raw, phi = S.dirs[i].phi, t = S.ts[i];
  auto fromEnd = [&](InkEnd e) { return (e == InkEnd::start ? S.ts[i] : 1 - S.ts[i]) * S.total; };
  double s = keysAt<double>(S.curves.size, t, 1) * S.scale;
  for (InkEnd e : ENDS) if (js::truthy(S.ends.taper(e))) s *= 1 + S.ends.taper(e) * smoothstep(1 - fromEnd(e) / S.ends.reach(e));
  Shape k = keysAt<Shape>(S.curves.shapes, t, Shape{1, 1, N.rx, N.ry}, mixShape);
  Shape shape = {k.w * N.w * s, k.h * N.h * s, k.rx, k.ry};
  for (InkEnd e : ENDS) {
    double d = discs[at(e)].value_or(0);
    if (js::truthy(d)) {
      double u = smoothstep(1 - fromEnd(e) / d);
      if (u > 0) shape = mixShape(shape, {d, d, 1.0, 1.0}, u);
    }
  }
  double fol = clamp(N.follow + keysAt<double>(S.curves.follow, t, 0), 0, 1);
  double rot = rad(N.angle + keysAt<double>(S.curves.rotate, t, 0)) + fol * (phi - PI / 2);
  std::vector<Pt> poly = stampPolygon(shape, rot, S.perCorner);
  Pt nn = {-js::sin(raw), js::cos(raw)};
  auto across = [&](const std::vector<Pt>& P) { std::vector<double> d; for (Pt p : P) d.push_back(dot(p, nn)); return d; };
  if (S.model == Model::pointed) {
    auto a = across(poly);
    double wide = sweep[i] ? *sweep[i] : js::maxOf(a) - js::minOf(a);
    shape = {wide, wide, 1.0, 1.0}; rot = 0; poly = stampPolygon(shape, 0, S.perCorner);
  }
  auto d = across(poly);
  double hi = js::maxOf(d), lo = js::minOf(d), o = keysAt<double>(S.curves.offset, t, 0);
  // Offset −1 puts the left edge (hi) on the centreline, 1 the right edge (lo).
  double shift = o < 0 ? o * hi : -o * lo;
  Pt c = {S.pts[i][0] + nn[0] * shift, S.pts[i][1] + nn[1] * shift};
  Placed out{{}, Frame{{shape.w, shape.h, shape.rx, shape.ry}, c, rot}, hi - lo};
  for (Pt p : poly) out.stamp.push_back({p.x + c[0], p.y + c[1]});
  return out;
}

/** Hulls of neighbouring stamps, each trimmed by the planes within their `reach` of its end. */
std::vector<std::vector<Pt>> hulls(const std::vector<std::vector<Pt>>& stamps, const std::vector<Pt>& pts,
                                   const std::vector<EndPlane>& planes, const double reach[2]) {
  size_t n = stamps.size();
  std::vector<double> along = runLengths(pts);
  double length = along[n - 1];
  std::vector<std::vector<Pt>> polys;
  for (size_t i = 0; i < std::max<size_t>(1, n - 1); i++) {
    std::vector<Pt> poly = n > 1 ? mergeHulls(stamps[i], stamps[i + 1]) : hull(stamps[0]);
    for (const EndPlane& pl : planes) {
      double from = pl.end == InkEnd::start ? along[i] : length - along[std::min(n - 1, i + 1)];
      if (from <= reach[at(pl.end)]) poly = clipPolygon(poly, pl);
      if (poly.size() < 3) break;
    }
    if (poly.size() >= 3) polys.push_back(poly);
  }
  return polys;
}

}  // namespace

InkResult ink(const InkStroke& st, const NibInput& nib, const InkOptions& opts) {
  Caps caps = opts.caps;
  Model model = opts.model;
  Nib N = nibOf(nib);
  double scale = st.weight ? *st.weight : 1, weight = N.w * scale;
  double perSpan = js::max(1, js::round(opts.perSpan ? *opts.perSpan : N.perSpan));
  double perCorner = js::max(1, js::round(N.perCorner));
  Curves cv = {sorted<double>(st.size), sorted<double>(st.rotate), sorted<double>(st.follow), sorted<double>(st.offset), sorted<Shape>(st.shapes)};
  Ends ends = endsOf(st);
  std::vector<Cubic> segs = st.segments;
  Sampled sm = sample(segs, perSpan, opts.spacing);
  std::vector<Pt>& pts = sm.pts;
  std::vector<std::optional<double>>& heads = sm.heads;
  size_t sampled = pts.size();
  std::vector<double> run = runLengths(pts);
  double total = js::orElse(run.back(), 1);
  std::vector<double> ts;
  for (double r : run) ts.push_back(r / total);
  std::optional<std::vector<double>> levels;
  if (opts.cuts) levels = opts.levels;
  RunOns ro = runOns(st, segs, pts, ts, heads, ends, reachOf(N, cv, scale, ends), caps, levels);
  std::vector<Dir> dirs = headings(pts, heads, st.closed);
  Stamping S{N, scale, perCorner, model, cv, ends, ts, total, pts, dirs};
  size_t n = pts.size();
  std::vector<std::vector<Pt>> stamps;
  std::vector<double> widths;
  std::vector<Frame> frames;
  // A pointed pen sweeping round a sharp corner (a run of points with headings set) eases its disc from the width it
  // arrives with to the width it leaves with: taken from the turning heading, it would swell through the thick
  // directions in between and leave a bump on the corner.
  std::vector<std::optional<double>> sweep(n);
  if (model == Model::pointed) for (size_t i = 0; i < n; i++) {
    if (!heads[i] || (i > 0 && heads[i - 1])) continue;
    size_t j = i;
    while (j + 1 < n && heads[j + 1]) j++;
    double a = place(S, i, {}, sweep).width, b = place(S, j, {}, sweep).width;
    for (size_t k = i; k <= j; k++) sweep[k] = a + (b - a) * (j > i ? double(k - i) / double(j - i) : 0);
  }
  Discs discs;
  for (InkEnd e : ENDS) if (ends.term[at(e)] && ends.term[at(e)]->mode == CutMode::round) discs[at(e)] = place(S, e == InkEnd::start ? 0 : n - 1, {}, sweep).width;
  for (size_t i = 0; i < n; i++) {
    Placed p = place(S, i, discs, sweep);
    stamps.push_back(p.stamp); frames.push_back(p.frame); widths.push_back(p.width);
  }
  auto polys = hulls(stamps, pts, ro.planes, ro.reach);
  // The skeleton's own nodes among the points, run-ons aside: where its segments meet.
  std::vector<int> nodes = {ro.lead};
  for (int c : sm.corners) nodes.push_back(ro.lead + c);
  nodes.push_back(ro.lead + int(sampled) - 1);
  return {segs, weight, ro.planes, pts, ts, widths, stamps, frames, polys, nodes, heads, caps, st.closed};
}

}  // namespace curves
