#include "curves/analysis.hpp"

#include "js.hpp"
#include "curves/intersect.hpp"
#include "curves/selection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

Cubic iso(const Path& path, const Cubic& c) {
  Cubic o;
  for (int k = 0; k < 4; k++) o[k] = toMetric(c[k], path.metric());
  return o;
}
Cubic back(const Path& path, const Cubic& c) {
  Cubic o;
  for (int k = 0; k < 4; k++) o[k] = fromMetric(c[k], path.metric());
  return o;
}

}  // namespace

PathProjection project(const Path& path, Pt p) {
  if (path.segs().empty()) {
    Pt d = toMetric({p.x - path.start().x, p.y - path.start().y}, path.metric());
    return {0, path.start(), curves::hypot(d.x, d.y)};
  }
  std::vector<Cubic> segs;
  for (auto& c : path.segs()) segs.push_back(iso(path, c));
  Nearest n = nearest(segs, toMetric(p, path.metric()), 64);
  return {path.progress(n.i, n.t), fromMetric(n.p, path.metric()), n.d};
}

std::vector<Progress> extrema(const Path& path) {
  std::vector<Progress> out;
  for (int i = 0; i < int(path.segs().size()); i++)
    for (double t : curves::extrema(path.segs()[i]).values)
      if (t > 1e-9 && t < 1 - 1e-9) out.push_back(path.progress(i, t));
  return out;
}

std::vector<Progress> inflections(const Path& path) {
  std::vector<Progress> out;
  for (int i = 0; i < int(path.segs().size()); i++)
    for (double t : curves::inflections(path.segs()[i])) out.push_back(path.progress(i, t));
  return out;
}

Progress side(const Path& path, Way way) {
  int k = way == Way::left || way == Way::right ? 0 : 1;
  double sign = way == Way::left || way == Way::bottom ? -1 : 1;
  std::vector<Progress> at;
  for (int i = 0; i < int(path.segs().size()); i++) at.push_back(path.progress(i, 0));
  for (double e : extrema(path)) at.push_back(e);
  at.push_back(1);
  double best = 0, v = -std::numeric_limits<double>::infinity();
  for (double a : at) {
    double q = sign * path.at(a)[k];
    if (q > v + 1e-12) { v = q; best = a; }
  }
  return best;
}

std::vector<Turn> turns(const Path& path, double minDeg) {
  std::vector<Turn> out;
  int n = int(path.segs().size());
  for (int i = path.closed() ? 0 : 1; i < n; i++) {
    double at = path.progress(i, 0);
    Pt a = path.direction((i == 0 ? 1 : at) - 1e-9), b = path.direction(at + 1e-9);
    double turn = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y) * 180 / PI;
    if (std::abs(turn) < minDeg) continue;
    Pt d = {a.x - b.x, a.y - b.y};
    double l = js::orElse(curves::hypot(d.x, d.y), 1);
    out.push_back({at, path.at(at), turn, {d.x / l, d.y / l}});
  }
  return out;
}

std::function<double(Progress)> nearness(const Path& path, const std::vector<Progress>& ats, double reach,
                                         const FalloffSpec& fall) {
  Falloff f = falloff(fall);
  double len = path.length();
  bool closed = path.closed();
  return [=](Progress u) {
    double w = 0;
    for (double a : ats) {
      double d = std::abs(u - a);
      if (closed) d = js::min(d, 1 - d);
      double x = reach > 0 ? d * len / reach : std::numeric_limits<double>::infinity();
      if (x < 1) w = js::max(w, f(x));
    }
    return w;
  };
}

Box bbox(const Path& path) {
  std::vector<Box> bs;
  if (!path.segs().empty()) for (auto& c : path.segs()) bs.push_back(curves::bbox(c));
  else bs.push_back({path.start(), path.start()});
  std::vector<double> x0, y0, x1, y1;
  for (auto& b : bs) { x0.push_back(b[0].x); y0.push_back(b[0].y); x1.push_back(b[1].x); y1.push_back(b[1].y); }
  return {Pt{js::minOf(x0), js::minOf(y0)}, Pt{js::maxOf(x1), js::maxOf(y1)}};
}

double curvature(const Path& path, Progress at) {
  Located l = path.locate(at);
  return !path.segs().empty() ? curves::curvature(iso(path, path.segs()[l.i]), l.t) : 0;
}

Pt normal(const Path& path, Progress at) {
  Pt d = path.direction(at);
  return {-d.y, d.x};
}

Osculating osculating(const Path& path, Progress at) {
  Located l = path.locate(at);
  if (path.segs().empty()) return {std::nullopt, std::numeric_limits<double>::infinity(), 0};
  Osculating o = curves::osculating(iso(path, path.segs()[l.i]), l.t);
  if (o.center) o.center = fromMetric(*o.center, path.metric());
  return o;
}

Path mold(const Path& path, Progress at, Pt to) {
  if (path.segs().empty()) return path;
  Located l = path.locate(at);
  Cubic c = curves::mold(path.segs()[l.i], l.t, to);
  return path.handle(l.i, HandleSide::out, c[1]).handle((l.i + 1) % path.nodeCount(), HandleSide::in, c[2]);
}

Path offset(const Path& path, const Reach& d) {
  double L = js::orElse(path.length(), 1);
  std::vector<double> D = path.distances();
  std::vector<std::vector<Cubic>> pieces;
  for (int i = 0; i < int(path.segs().size()); i++) {
    // A graduated distance runs along the whole path: map the segment's t to the path's progress.
    Reach r = std::holds_alternative<double>(d)
                  ? d
                  : Reach([&d, &D, i, L](double u) { return std::get<1>(d)((D[i] + (D[i + 1] - D[i]) * u) / L); });
    std::vector<Cubic> ps;
    for (auto& q : curves::offset(iso(path, path.segs()[i]), r)) ps.push_back(back(path, q));
    pieces.push_back(std::move(ps));
  }
  std::vector<Cubic> segs;
  auto bevel = [&](Pt a, Pt b) {
    if (curves::hypot(a.x - b.x, a.y - b.y) > 1e-9)
      segs.push_back({a, a + Pt{b.x - a.x, b.y - a.y} * (1.0 / 3), a + Pt{b.x - a.x, b.y - a.y} * (2.0 / 3), b});
  };
  for (size_t k = 0; k < pieces.size(); k++) {
    auto& ps = pieces[k];
    if (k && !ps.empty() && !segs.empty()) bevel(segs.back()[3], ps[0][0]);
    segs.insert(segs.end(), ps.begin(), ps.end());
  }
  if (path.closed() && !segs.empty()) bevel(segs.back()[3], segs[0][0]);
  return !segs.empty() ? Path::of(segs, path.closed(), path.metric()) : path;
}

std::vector<Contour> outline(const Path& path, double d1) { return outline(path, d1, d1); }

std::vector<Contour> outline(const Path& path, double d1, double d2) {
  std::vector<Cubic> left = offset(path, d1).segs(), right = offset(path, -d2).segs();
  std::vector<Cubic> rev;
  for (auto it = right.rbegin(); it != right.rend(); ++it) rev.push_back(reverse(*it));
  auto cubicsOf = [](const std::vector<Cubic>& cs) {
    Contour o;
    for (auto& c : cs) o.push_back(Piece::of(c));
    return o;
  };
  if (path.closed()) return {cubicsOf(left), cubicsOf(rev)};
  if (left.empty() || rev.empty()) return {};
  Contour c = cubicsOf(left);
  c.push_back(Piece::of(left.back()[3], rev[0][0]));
  for (auto& q : rev) c.push_back(Piece::of(q));
  c.push_back(Piece::of(rev.back()[3], left[0][0]));
  return {c};
}

std::vector<PathLineCrossing> lineCrossings(const Path& path, Pt a, Pt b, bool segment) {
  std::vector<PathLineCrossing> out;
  for (int i = 0; i < int(path.segs().size()); i++)
    for (auto& h : curves::lineCrossings(path.segs()[i], a, b, segment)) out.push_back({path.progress(i, h.t), h.s, h.p});
  return out;
}

std::vector<PathCircleCrossing> circleCrossings(const Path& path, Pt o, double r) {
  Pt O = toMetric(o, path.metric());
  std::vector<PathCircleCrossing> out;
  for (int i = 0; i < int(path.segs().size()); i++)
    for (auto& h : curves::circleCrossings(iso(path, path.segs()[i]), O, r))
      out.push_back({path.progress(i, h.t), fromMetric(h.p, path.metric())});
  return out;
}

std::vector<PathSelfCrossing> selfCrossings(const Path& path) {
  std::vector<PathSelfCrossing> out;
  for (auto& x : curves::selfCrossings(path.segs(), path.closed()))
    out.push_back({path.progress(x.a.i, x.a.t), path.progress(x.b.i, x.b.t), x.p});
  std::stable_sort(out.begin(), out.end(), [](const auto& x, const auto& y) { return x.a < y.a; });
  return out;
}

std::vector<PathArc> arcs(const Path& path, double threshold) {
  std::vector<PathArc> out;
  for (int i = 0; i < int(path.segs().size()); i++)
    for (auto& a : curves::arcs(iso(path, path.segs()[i]), threshold)) {
      PathArc r;
      static_cast<Arc&>(r) = a;
      if (a.center) r.center = fromMetric(*a.center, path.metric());
      r.from = fromMetric(a.from, path.metric());
      r.to = fromMetric(a.to, path.metric());
      r.at0 = path.progress(i, a.t0);
      r.at1 = path.progress(i, a.t1);
      out.push_back(r);
    }
  return out;
}

Selection selectAt(const Path& path, Progress at, double reach, const SelectAtOptions& o) {
  Path p = o.cut ? path.cut({at}) : path;
  std::vector<double> D = p.distances();
  for (double& d : D) d = d / js::orElse(p.length(), 1);
  int k = 0;
  for (int i = 0; i < int(D.size()); i++)
    if (std::abs(D[i] - at) < std::abs(D[k] - at)) k = i;
  if (p.closed() && k == p.nodeCount()) k = 0;
  return p.select(NodePick(std::vector<int>{k})).soften(reach, SoftenOptions{o.falloff, o.along});
}

// Path's forwarders.
PathProjection Path::project(Pt p) const { return curves::project(*this, p); }
std::vector<Progress> Path::extrema() const { return curves::extrema(*this); }
std::vector<Progress> Path::inflections() const { return curves::inflections(*this); }
Box Path::bbox() const { return curves::bbox(*this); }
Progress Path::side(Way way) const { return curves::side(*this, way); }
std::vector<Turn> Path::turns(double minDeg) const { return curves::turns(*this, minDeg); }
std::function<double(Progress)> Path::nearness(const std::vector<Progress>& ats, double reach,
                                               const FalloffSpec& falloff) const {
  return curves::nearness(*this, ats, reach, falloff);
}
double Path::curvature(Progress at) const { return curves::curvature(*this, at); }
Pt Path::normal(Progress at) const { return curves::normal(*this, at); }
Osculating Path::osculating(Progress at) const { return curves::osculating(*this, at); }
Path Path::mold(Progress at, Pt to) const { return curves::mold(*this, at, to); }
Path Path::offset(const Reach& d) const { return curves::offset(*this, d); }
std::vector<Contour> Path::outline(double d1) const { return curves::outline(*this, d1, d1); }
std::vector<Contour> Path::outline(double d1, double d2) const { return curves::outline(*this, d1, d2); }
std::vector<PathLineCrossing> Path::lineCrossings(Pt a, Pt b, bool segment) const {
  return curves::lineCrossings(*this, a, b, segment);
}
std::vector<PathCircleCrossing> Path::circleCrossings(Pt o, double r) const { return curves::circleCrossings(*this, o, r); }
std::vector<PathSelfCrossing> Path::selfCrossings() const { return curves::selfCrossings(*this); }
std::vector<PathArc> Path::arcs(double threshold) const { return curves::arcs(*this, threshold); }
Selection Path::selectAt(Progress at, double reach, const SelectAtOptions& o) const {
  return curves::selectAt(*this, at, reach, o);
}

}  // namespace curves
