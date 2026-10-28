// See include/curves/outline.hpp. Ported from fountain-ts/src/ink/outline.ts line for line.
#include "curves/outline.hpp"

#include "curves/fit.hpp"
#include "curves/nib.hpp"
#include "js.hpp"
#include "jsmath.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

#include <algorithm>
#include <cmath>

namespace curves {

const OutlineSettings OUTLINE{};

namespace {

/** A rail point `p` with `at` the stamp's index (fractional where a rail was cut). */
struct RailPt {
  Pt p;
  double at;
};
using Rail = std::vector<RailPt>;
struct Fit {
  Contour contour;
  std::vector<Contour> contours;
  Errs errs;
  Runs runs;
};
struct Parts {
  Rail left, right;
  bool trimmedStart, trimmedEnd;
  bool trimmed(InkEnd e) const { return e == InkEnd::start ? trimmedStart : trimmedEnd; }
};

/** The rail point's direction is searched within this of square across the stroke, by this many halvings. */
const double SEARCH = rad(80);
constexpr int HALVINGS = 24;
/** An untrimmed end's stamp is sampled with this many points per corner, finer than the ink's, as the cap is fitted
 *  to it directly. */
constexpr double CAP_CORNER = 24;
/** The cap leaves smoothly along the rail where the two differ by less than this share of CORNER. A stamp turning
 *  with the stroke meets its rails at a slight angle that a smooth join should still take. */
constexpr double CAP_JOIN = 1.5;
/** The angle between two unit directions, 0…π. */
double angleOf(Pt a, Pt b) { return std::abs(js::atan2(cross(a, b), dot(a, b))); }

const std::optional<int>& planEnd(const OutlinePlan& p, InkEnd e) { return e == InkEnd::start ? p.start : p.end; }

/** The furthest point of a placed nib (a frame from ink) in the unit direction m, on its exact rounded
 *  rectangle: each corner's ellipse arc answers for the directions in its own quadrant. A side lying across m
 *  touches it all along: its middle stands for it. */
Pt support(const Frame& f, Pt m) {
  Pt l = js::rotate(m, -f.rot);
  double eps = 1e-9 * (f.w / 2 + f.h / 2 + 1);
  double best = -INFINITY;
  std::vector<Pt> at;
  for (const CornerArc& arc : cornerArcs(f)) {
    double sx = arc.s[0], sy = arc.s[1], ca = arc.r[0], cb = arc.r[1];
    double nx = sx * js::max(0, sx * l[0]), ny = sy * js::max(0, sy * l[1]), den = curves::hypot(ca * nx, cb * ny);
    Pt q = arc.o;
    if (den > 1e-12) q = {q[0] + ca * ca * nx / den, q[1] + cb * cb * ny / den};
    else if (ca > 1e-9 || cb > 1e-9) q = {q[0] + ca * sx * (js::truthy(nx) ? 1 : 0), q[1] + cb * sy * (js::truthy(ny) ? 1 : 0)};
    double v = dot(q, l);
    if (v > best + eps) { best = v; at = {q}; } else if (v > best - eps) at.push_back(q);
  }
  Pt q = mean(at);
  // Written out rather than add(c, rotate(q)): the sum in this order is operations.js's, which the tests match to 1e-9
  // and the fit downstream amplifies any last bit.
  double cs = js::cos(f.rot), sn = js::sin(f.rot);
  return {f.c[0] + q[0] * cs - q[1] * sn, f.c[1] + q[0] * sn + q[1] * cs};
}

/** The two rails. A rail point is where the stamps' envelope touches the stamp: the direction, near square across
 *  the stroke, in which the nib reaches no further from one stamp to the next. Turning or growing nibs lean it off
 *  square. Round a sharp corner (stamps with `heads`) it is square across each heading in turn. */
void rails(const InkResult& r, Rail& left, Rail& right) {
  const auto& pts = r.pts; const auto& frames = r.frames;
  int n = int(pts.size());
  auto reach = [](const Frame& f, Pt m) { return dot(support(f, m), m); };
  for (int i = 0; i < n; i++) {
    const std::optional<double>& h = i < int(r.heads.size()) ? r.heads[i] : std::optional<double>();
    const Frame& f = frames[i];
    int ib = i > 0 ? i - 1 : r.closed ? n - 2 : 0, ia = i < n - 1 ? i + 1 : r.closed ? 1 : n - 1;
    Pt d = h ? js::fromAngle(*h) : norm(pts[ia] - pts[ib]);
    const Frame& before = frames[ib]; const Frame& after = frames[ia];
    for (int side = 0; side < 2; side++) {
      Rail& rail = side == 0 ? left : right;
      double sgn = side == 0 ? 1 : -1;
      double base = js::atan2(sgn * d[0], -sgn * d[1]);
      auto g = [&](double ang) { Pt m = js::fromAngle(ang); return reach(after, m) - reach(before, m); };
      double lo = base - SEARCH, hi = base + SEARCH, glo = g(lo), ghi = g(hi), ang = base;
      if (n > 1 && !h && glo * ghi < 0) {
        for (int it = 0; it < HALVINGS; it++) {
          double mid = (lo + hi) / 2, gm = g(mid);
          if ((gm < 0) == (glo < 0)) { lo = mid; glo = gm; } else hi = mid;
        }
        ang = (lo + hi) / 2;
      }
      rail.push_back({support(f, js::fromAngle(ang)), double(i)});
    }
  }
}

/** The first index of a rail (its stamp indices never fall) at or past `at` (`inclusive`), or past it. */
int firstAt(const Rail& rail, double at, bool inclusive) {
  int lo = 0, hi = int(rail.size());
  while (lo < hi) {
    int m = (lo + hi) >> 1;
    if (inclusive ? rail[m].at >= at : rail[m].at > at) hi = m; else lo = m + 1;
  }
  return lo;
}

/** The rail's point at a stamp index, between its samples. */
Pt railAt(const Rail& rail, double at) {
  if (at <= rail[0].at) return rail[0].p;
  if (at > rail.back().at) return rail.back().p;
  int k = firstAt(rail, at, true);
  const RailPt& a = rail[k - 1]; const RailPt& b = rail[k];
  return lerp(a.p, b.p, b.at > a.at ? (at - a.at) / (b.at - a.at) : 1);
}

/** A rail trimmed by a plane at its own end: from where it crosses into the kept side. A rail that stops short
 *  of the plane (a flat end with the nib turned) runs on along its last direction to meet it. */
Rail clipRail(const Rail& rail, const EndPlane& pl) {
  auto side = [&](const RailPt& q) { return dot(q.p - pl.at, pl.n); };
  bool fwd = pl.end == InkEnd::start;
  Rail R = rail;
  if (!fwd) std::reverse(R.begin(), R.end());
  int k = -1;
  for (size_t i = 0; i < R.size(); i++) if (side(R[i]) >= -1e-9) { k = int(i); break; }
  if (k < 0) return rail;
  Rail out;
  if (k > 0) {
    const RailPt& a = R[k - 1]; const RailPt& b = R[k];
    double sa = side(a), sb = side(b), u = sa / (sa - sb);
    out.push_back({lerp(a.p, b.p, u), a.at + (b.at - a.at) * u});
    out.insert(out.end(), R.begin() + k, R.end());
  } else {
    const RailPt& e = R[0];
    const RailPt* q = nullptr;
    for (const RailPt& x : R) if (dist(x.p, e.p) > 1e-6) { q = &x; break; }
    std::optional<Pt> back = q ? std::optional(norm(e.p - q->p)) : std::nullopt;
    double along = back ? dot(*back, pl.n) : 0;
    out = R;
    if (back && along < -1e-3 && side(e) > 1e-6) {
      double s = side(e) / -along;
      out.insert(out.begin(), {Pt{e.p[0] + (*back)[0] * s, e.p[1] + (*back)[1] * s}, e.at + (fwd ? -1e-6 : 1e-6)});
    }
  }
  if (!fwd) std::reverse(out.begin(), out.end());
  return out;
}

/** A rail without the loops a turn tighter than the nib makes on its inner side: the points of a loop all
 *  move to where it crosses itself, so the stamp indices still find their places. A closed rail is a ring: of
 *  the two loops a crossing makes, the shorter is the tangle. A tangle winds against the centerline under it (the
 *  rail runs backwards round the inside of the turn); a loop that winds with it is the stroke's own (e's bar
 *  running into its bowl and the bowl crossing the bar's start), whose counter must stay open, so it is kept. */
Rail untangle(const Rail& rail, const std::vector<Pt>& pts, bool closed) {
  Rail out = rail;
  auto crossAt = [](Pt a, Pt b, Pt c, Pt d) -> std::optional<Pt> {
    auto m = meet(a, b - a, c, d - c, 1e-12, 1e-9);
    return m ? std::optional(lerp(a, b, (*m)[0])) : std::nullopt;
  };
  int N = int(out.size());
  // Shortest loops first, so a tangle inside a kept loop (where e's bar turns up into its bowl) is collapsed before
  // the loop round it is judged: its backward area would otherwise count against the loop's own. Taking the
  // outermost crossing first did that, and once a lean cut the rail finer the tangle outweighed e's eye and filled
  // it. A collapsed loop's points sit on one point, so its segments cross nothing after.
  for (int span = 2; span < N - 1; span++) {
    if (closed && 2 * span > N) break;
    for (int i = 0; i + span + 1 < N; i++) {
      int j = i + span;
      auto x = crossAt(out[i].p, out[i + 1].p, out[j].p, out[j + 1].p);
      if (!x) continue;
      std::vector<Pt> loop = {*x};
      for (int k = i + 1; k <= j; k++) loop.push_back(out[k].p);
      size_t lo = size_t(std::floor(out[i + 1].at)), hi = std::min(pts.size(), size_t(std::ceil(out[j].at)) + 1);
      std::vector<Pt> under;
      for (size_t k = lo; k < hi; k++) under.push_back(pts[k]);
      double a = area(under);
      if (std::abs(a) > 1e-6 && a * area(loop) > 0) continue;
      for (int k = i + 1; k <= j; k++) out[k].p = *x;
    }
  }
  return out;
}

struct Anchors {
  std::vector<double> ats;
  std::vector<int> spans;
};

/** A rail's anchors as stamp indices, by the plan (pieces per skeleton span), each with its span. */
Anchors railAnchors(const Rail& rail, const std::vector<int>& nodes, const std::vector<int>& plan) {
  double s = rail[0].at, e = rail.back().at;
  int S = int(nodes.size()) - 1;
  Anchors a;
  a.ats = {s};
  for (int k = 0; k < S; k++) for (int j = 1; j <= plan[k]; j++) {
    double at = k == S - 1 && j == plan[k] ? e : nodes[k] + double(nodes[k + 1] - nodes[k]) * j / plan[k];
    a.ats.push_back(js::min(e, js::max(a.ats.back(), at)));
    a.spans.push_back(k);
  }
  return a;
}

/** The rail's direction leaving `at` forwards (side 1) or arriving there (−1), from its own samples on that
 *  side only, so a joint where the curvature jumps (a straight run into a bowl) doesn't tilt it. */
Pt railDir(const Rail& rail, double at, double side) {
  double s = rail[0].at, e = rail.back().at;
  auto P = [&](double x) { return railAt(rail, js::max(s, js::min(e, x))); };
  for (double h : {1, 2, 4, 8, 16, 32}) {
    // A parabola through three samples, parametrized by the distance along them (the stamps' spacing
    // changes at the nodes and the run-ons), differentiated at the first.
    Pt p0 = P(at), p1 = P(at + side * h), p2 = P(at + side * 2 * h);
    double d1 = dist(p0, p1), d2 = d1 + dist(p1, p2);
    if (d1 < 1e-9 || d2 - d1 < 1e-9) continue;
    double k0 = -(1 / d1 + 1 / d2), k1 = d2 / (d1 * (d2 - d1)), k2 = -d1 / (d2 * (d2 - d1));
    Pt v = {side * (k0 * p0[0] + k1 * p1[0] + k2 * p2[0]), side * (k0 * p0[1] + k1 * p1[1] + k2 * p2[1])};
    if (curves::hypot(v[0], v[1]) > 1e-9) return norm(v);
  }
  Pt q = P(at + side), r = P(at);
  return dist(q, r) > 1e-9 ? norm({side * (q[0] - r[0]), side * (q[1] - r[1])}) : Pt{0, 0};
}

struct Railed {
  std::vector<Cubic> curves;
  std::vector<double> errs;
};

/** A rail as cubics by the plan: { curves, errs } with the worst error in each skeleton span. An anchor whose
 *  two sides agree on the rail's direction is smooth and both share it; otherwise it is a corner. */
Railed fitRail(const Rail& rail, const InkResult& r, const std::vector<int>& plan) {
  Anchors an = railAnchors(rail, r.nodes, plan);
  const auto& ats = an.ats; const auto& spans = an.spans;
  int last = int(ats.size()) - 1;
  struct Dirs { std::optional<Pt> out, inn; };
  std::vector<Dirs> dirs;
  for (int i = 0; i <= last; i++) {
    std::optional<Pt> out, inn;
    if (i < last) out = railDir(rail, ats[i], 1);
    if (i > 0) inn = railDir(rail, ats[i], -1);
    if (out && inn && angleOf(*out, *inn) < CORNER) {
      Pt m = norm({(*out)[0] + (*inn)[0], (*out)[1] + (*inn)[1]});
      dirs.push_back({m, m});
    } else dirs.push_back({out, inn});
  }
  Railed res;
  res.errs.assign(plan.size(), 0);
  for (int i = 0; i < last; i++) {
    double a = ats[i], b = ats[i + 1];
    std::vector<Pt> P = {railAt(rail, a)};
    for (int k = firstAt(rail, a, false); k < int(rail.size()) && rail[k].at < b; k++) P.push_back(rail[k].p);
    P.push_back(railAt(rail, b));
    Pt out = *dirs[i].out, inn = *dirs[i + 1].inn;
    CubicFit f = fitCubic(P, out, -inn);
    res.curves.push_back(f.c);
    res.errs[spans[i]] = js::max(res.errs[spans[i]], f.err);
  }
  return res;
}

struct Capped {
  std::vector<Piece> curves;
  double err;
  std::string runs;
};

/** An untrimmed end: the end stamp's far side from one rail's end to the other's, in `pieces` cubics of equal
 *  length. Where the stamp meets a rail without a corner, the cap leaves along the rail's own direction. */
Capped fitCap(const InkResult& r, InkEnd end, Pt from, Pt to, Pt tFrom, Pt tTo, int pieces) {
  int n = int(r.pts.size()), i = end == InkEnd::end ? n - 1 : 0;
  const Frame& f = r.frames[i];
  std::vector<Pt> poly = stampPolygon(f, f.rot, CAP_CORNER);
  for (Pt& p : poly) p = {p[0] + f.c[0], p[1] + f.c[1]};
  int m = int(poly.size());
  Pt d = norm(end == InkEnd::end ? r.pts[n - 1] - r.pts[std::max(0, n - 2)] : r.pts[std::min(n - 1, 1)] - r.pts[0]);
  Pt o = end == InkEnd::end ? d : -d;
  auto ahead = [&](Pt q, Pt p) { return dot(q - p, o) > 1e-9; };
  auto near = [&](Pt p) {
    int b = 0;
    for (int k = 0; k < m; k++) if (dist(poly[k], p) < dist(poly[b], p)) b = k;
    return b;
  };
  // Clockwise round the stamp (its polygon runs the other way), over its far side.
  int i0 = near(from), i1 = near(to);
  std::vector<Pt> arc = {from};
  if (ahead(poly[i0], from)) arc.push_back(poly[i0]);
  for (int k = (i0 - 1 + m) % m, g = 0; k != i1 && g < m; k = (k - 1 + m) % m, g++) arc.push_back(poly[k]);
  if (i1 != i0 && ahead(poly[i1], to)) arc.push_back(poly[i1]);
  arc.push_back(to);
  // Runs between the nib's sharp corners: a straight one is a line, a rounded one is `pieces` cubics.
  std::vector<std::vector<Pt>> split = {{arc[0]}};
  for (size_t k = 1; k < arc.size(); k++) {
    Pt p = arc[k];
    std::vector<Pt>& run = split.back();
    if (dist(p, run.back()) < 1e-6) continue;
    run.push_back(p);
    if (k + 1 < arc.size()) {
      Pt q = arc[k + 1];
      if (k < arc.size() - 1 && dist(q, p) > 1e-6 && angleOf(norm(p - run[run.size() - 2]), norm(q - p)) > CORNER) split.push_back({p});
    }
  }
  std::vector<std::vector<Pt>> kept;
  for (auto& run : split) if (run.size() > 1) kept.push_back(run);
  auto straight = [](const std::vector<Pt>& run) {
    for (Pt p : run) if (!nearSegment(p, run[0], run.back(), 1e-3, INFINITY)) return false;
    return true;
  };
  // Smooth into the rails when the stamp turns with them; a corner otherwise.
  auto join = [](Pt rail, Pt own) { return angleOf(rail, own) < CORNER * CAP_JOIN ? rail : own; };
  Capped res{{}, 0, ""};
  for (size_t ri = 0; ri < kept.size(); ri++) {
    const std::vector<Pt>& ar = kept[ri];
    if (straight(ar)) { res.curves.push_back(Piece::of(ar[0], ar.back())); res.runs += 'l'; continue; }
    res.runs += 'c';
    std::vector<double> run = runLengths(ar);
    double L = run.back();
    auto at = [&](double s) { return pointAlong(ar, run, s); };
    // Tangents by central differences, a quarter piece wide (at most a sixteenth of the run): wide enough to step
    // over the stamp's sampling, narrow enough to stay on one side of a rounded corner.
    double step = L / pieces, h = js::min(step / 4, L / 16);
    auto tan = [&](double s) -> Pt {
      return s <= 0 ? norm(at(h) - at(0)) : s >= L ? norm(at(L) - at(L - h)) : norm(at(s + h) - at(s - h));
    };
    for (int k = 0; k < pieces; k++) {
      double a = k * step, b = (k + 1) * step;
      std::vector<Pt> P = {at(a)};
      for (size_t j = 0; j < run.size(); j++) if (run[j] > a && run[j] < b) P.push_back(ar[j]);
      P.push_back(at(b));
      Pt out = k == 0 && ri == 0 ? join(tFrom, tan(0)) : tan(a);
      Pt inn = k == pieces - 1 && ri == kept.size() - 1 ? join(tTo, tan(L)) : tan(b);
      CubicFit fit = fitCubic(P, out, -inn);
      res.curves.push_back(Piece::of(fit.c));
      res.err = js::max(res.err, fit.err);
    }
  }
  if (res.curves.empty()) res.curves.push_back(Piece::of(from, to));
  return res;
}

/** The rails, trimmed and untangled, and which ends a plane trims. */
Parts outlineParts(const InkResult& r) {
  Parts parts;
  rails(r, parts.left, parts.right);
  for (const EndPlane& pl : r.planes) { parts.left = clipRail(parts.left, pl); parts.right = clipRail(parts.right, pl); }
  parts.left = untangle(parts.left, r.pts, r.closed); parts.right = untangle(parts.right, r.pts, r.closed);
  parts.trimmedStart = parts.trimmedEnd = false;
  for (const EndPlane& pl : r.planes) (pl.end == InkEnd::start ? parts.trimmedStart : parts.trimmedEnd) = true;
  return parts;
}

Piece reversed(const Piece& q) { return reverse(q); }

/** The whole contour by a plan: the left rail, the end (a line along its plane, or the stamp's far side), the right
 *  rail backwards and the start. A closed stroke is two rings instead, each rail closed on itself: the left one
 *  forward, the right one back. */
Fit fitOutline(const InkResult& r, const OutlinePlan& plan, const Parts& parts) {
  const Rail& left = parts.left; const Rail& right = parts.right;
  Railed L = fitRail(left, r, plan.left), R = fitRail(right, r, plan.right);
  auto pieces = [](const std::vector<Cubic>& cs) { Contour c; for (const Cubic& x : cs) c.push_back(Piece::of(x)); return c; };
  if (r.closed) {
    // The last piece ends exactly where the first starts.
    auto ring = [](Contour out) {
      Piece z = out.back();
      z.p[z.line ? 1 : 3] = out[0].p[0];
      out.back() = z;
      return out;
    };
    Contour lr = ring(pieces(L.curves));
    Contour rv;
    for (size_t k = R.curves.size(); k-- > 0;) rv.push_back(reversed(Piece::of(R.curves[k])));
    Contour rr = ring(rv);
    return {lr, {lr, rr}, {L.errs, R.errs, 0, 0}, {"l", "l"}};
  }
  auto tangent = [](const std::vector<Cubic>& curves, InkEnd end) {
    return end == InkEnd::start ? endTangents(Piece::of(curves[0]))[0] : endTangents(Piece::of(curves.back()))[1];
  };
  Capped ends[2];
  for (InkEnd end : {InkEnd::start, InkEnd::end}) {
    Pt l = end == InkEnd::start ? left[0].p : left.back().p, rr = end == InkEnd::start ? right[0].p : right.back().p;
    const std::optional<int>& pl = planEnd(plan, end);
    int idx = end == InkEnd::start ? 0 : 1;
    if (!pl) { ends[idx] = {{end == InkEnd::end ? Piece::of(l, rr) : Piece::of(rr, l)}, 0, "l"}; continue; }
    Pt tl = tangent(L.curves, end), tr = tangent(R.curves, end);
    ends[idx] = end == InkEnd::end ? fitCap(r, InkEnd::end, l, rr, tl, -tr, *pl) : fitCap(r, InkEnd::start, rr, l, -tr, tl, *pl);
  }
  Contour contour = pieces(L.curves);
  contour.insert(contour.end(), ends[1].curves.begin(), ends[1].curves.end());
  for (size_t k = R.curves.size(); k-- > 0;) contour.push_back(reversed(Piece::of(R.curves[k])));
  contour.insert(contour.end(), ends[0].curves.begin(), ends[0].curves.end());
  return {contour, {contour}, {L.errs, R.errs, ends[0].err, ends[1].err}, {ends[0].runs, ends[1].runs}};
}

struct Planned {
  OutlinePlan plan;
  std::optional<Fit> fit;
};

/** The plan an inked stroke needs alone, with the fit it was last checked with: pieces per skeleton span on each
 *  rail, and per rounded run of each end (a line where a plane trims it), split until every piece is within the
 *  tolerance. The fit is none when the last round still split a piece (the plan then has no fit yet). */
Planned planned(const InkResult& r, double tolerance) {
  Parts parts = outlineParts(r);
  size_t S = r.nodes.size() - 1;
  auto ends = [&](InkEnd e) -> std::optional<int> { return r.closed || parts.trimmed(e) ? std::nullopt : std::optional(2); };
  OutlinePlan plan;
  plan.left.assign(S, 1); plan.right.assign(S, 1); plan.start = ends(InkEnd::start); plan.end = ends(InkEnd::end);
  for (int round = 0; round < OUTLINE.most; round++) {
    Fit fit = fitOutline(r, plan, parts);
    const Errs& errs = fit.errs;
    plan.runs = fit.runs;
    bool more = false;
    for (size_t k = 0; k < errs.left.size(); k++) if (errs.left[k] > tolerance && plan.left[k] < OUTLINE.most) { plan.left[k]++; more = true; }
    for (size_t k = 0; k < errs.right.size(); k++) if (errs.right[k] > tolerance && plan.right[k] < OUTLINE.most) { plan.right[k]++; more = true; }
    for (InkEnd end : {InkEnd::start, InkEnd::end}) {
      std::optional<int>& p = end == InkEnd::start ? plan.start : plan.end;
      double e = end == InkEnd::start ? errs.start : errs.end;
      if (p && e > tolerance && *p < OUTLINE.most) { p = *p + 1; more = true; }
    }
    if (!more) return {plan, fit};
  }
  return {plan, std::nullopt};
}

/** Each end's runs as a plan's key writes them: empty for a line end. */
std::string runsKey(const OutlinePlan& p, InkEnd e) {
  return p.runs && planEnd(p, e) ? ":" + (e == InkEnd::start ? p.runs->start : p.runs->end) : "";
}

}  // namespace

OutlinePlan outlinePlan(const InkResult& r, double tolerance) { return planned(r, tolerance).plan; }

std::optional<OutlinePlan> sharedPlan(const std::vector<OutlinePlan>& plans) {
  if (plans.empty()) return std::nullopt;
  const OutlinePlan& a = plans[0];
  auto differ = [&](const OutlinePlan& p) {
    if (p.left.size() != a.left.size()) return true;
    for (InkEnd e : {InkEnd::start, InkEnd::end})
      if (planEnd(p, e).has_value() != planEnd(a, e).has_value() || runsKey(p, e) != runsKey(a, e)) return true;
    return false;
  };
  for (size_t k = 1; k < plans.size(); k++) if (differ(plans[k])) return std::nullopt;
  auto most = [](const std::optional<int>& x, const std::optional<int>& y) -> std::optional<int> {
    return !x ? x : std::optional(std::max(*x, y ? *y : 0));
  };
  OutlinePlan m = plans[0];
  for (size_t k = 1; k < plans.size(); k++) {
    const OutlinePlan& p = plans[k];
    OutlinePlan next;
    for (size_t i = 0; i < m.left.size(); i++) next.left.push_back(std::max(m.left[i], p.left[i]));
    for (size_t i = 0; i < m.right.size(); i++) next.right.push_back(std::max(m.right[i], p.right[i]));
    next.start = most(m.start, p.start); next.end = most(m.end, p.end);
    if (a.runs) next.runs = a.runs;
    m = next;
  }
  return m;
}

OutlineResult outlineInk(const InkResult& r, const std::optional<OutlinePlan>& given, double tolerance) {
  std::optional<Planned> own;
  if (!given) own = planned(r, tolerance);
  OutlinePlan plan = given ? *given : own->plan;
  Fit fit = own && own->fit ? *own->fit : fitOutline(r, plan, outlineParts(r));
  std::vector<double> all = fit.errs.left;
  all.insert(all.end(), fit.errs.right.begin(), fit.errs.right.end());
  all.push_back(fit.errs.start); all.push_back(fit.errs.end);
  double err = js::maxOf(all);
  int anchors = 0;
  for (const Contour& c : fit.contours) anchors += int(c.size());
  return {fit.contour, fit.contours, plan, anchors, err, fit.errs};
}

std::string planKey(const OutlinePlan& p) {
  auto join = [](const std::vector<int>& v) { std::string s; for (size_t i = 0; i < v.size(); i++) s += (i ? "." : "") + std::to_string(v[i]); return s; };
  auto end = [](const std::optional<int>& e) { return e ? std::to_string(*e) : std::string("line"); };
  return "L" + join(p.left) + " E" + end(p.end) + runsKey(p, InkEnd::end) + " R" + join(p.right) + " S" + end(p.start) + runsKey(p, InkEnd::start);
}

}  // namespace curves
