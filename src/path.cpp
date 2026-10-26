#include "curves/path.hpp"

#include "curves/boolean.hpp"
#include "js.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <map>
#include <stdexcept>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** A number as JS writes it in a message (near enough: the shortest digits that read back). */
std::string jsNum(double x) {
  char buf[32];
  auto r = std::to_chars(buf, buf + sizeof buf, x);
  return std::string(buf, r.ptr);
}

/** The sum of the first `i` of `xs` (all of them when `i` is past the end). */
double sumTo(const std::vector<double>& xs, int i) {
  double s = 0;
  for (int k = 0; k < i && k < int(xs.size()); k++) s = s + xs[k];
  return s;
}

/** The segments holding node `i`'s handles: the one arriving and the one leaving; −1 where there is none. */
struct Handles {
  int inSeg, outSeg;
};
Handles handlesOf(int n, bool closed, int i) { return {i > 0 ? i - 1 : closed ? n - 1 : -1, i < n ? i : -1}; }

/** Node `i`'s kind read off its handles in the metric: lined up through the node (within a millionth of a radian) is
 *  smooth, and as long as each other as well, symmetric. A handle on its node takes the direction to the next
 *  control point instead. */
NodeKind kindOf(const std::vector<Cubic>& segs, bool closed, int i, Pt metric) {
  auto [inSeg, outSeg] = handlesOf(int(segs.size()), closed, i);
  if (inSeg < 0 || outSeg < 0) return NodeKind::corner;
  auto iso = [&](Pt p) { return toMetric(p, metric); };
  Cubic a, b;
  for (int k = 0; k < 4; k++) { a[k] = iso(segs[inSeg][k]); b[k] = iso(segs[outSeg][k]); }
  Pt N = b[0];
  auto dir = [&](std::array<Pt, 3> qs) -> std::optional<Pt> {
    for (Pt q : qs) { Pt d = q - N; if (curves::hypot(d[0], d[1]) > 1e-12) return d; }
    return std::nullopt;
  };
  auto di = dir({a[2], a[1], a[0]}), dO = dir({b[1], b[2], b[3]});
  if (!di || !dO) return NodeKind::corner;
  Pt ui = norm(*di), uo = norm(*dO);
  if (std::abs(cross(ui, uo)) > 1e-6 || dot(ui, uo) > 0) return NodeKind::corner;
  Pt hi = a[2] - N, ho = b[1] - N;
  double li = curves::hypot(hi[0], hi[1]), lo = curves::hypot(ho[0], ho[1]);
  return li > 1e-12 && std::abs(li - lo) <= 1e-6 * js::max(li, lo) ? NodeKind::symmetric : NodeKind::smooth;
}

template <class T>
const T& pick(const std::vector<T>& ps, const Which& which, const std::string& what) {
  const T* p = nullptr;
  if (auto e = std::get_if<End>(&which)) {
    if (!ps.empty()) p = *e == End::first ? &ps.front() : &ps.back();
  } else {
    int i = std::get<int>(which);
    if (i >= 0 && i < int(ps.size())) p = &ps[i];
  }
  if (!p) throw std::invalid_argument("the path doesn't cross " + what);
  return *p;
}

/** Segments cut at the parameters in `cuts` (by segment), each run sorted. */
std::vector<Cubic> cutSegs(const std::vector<Cubic>& segs, std::map<int, std::vector<double>>& cuts) {
  std::vector<Cubic> out;
  for (int i = 0; i < int(segs.size()); i++) {
    std::vector<double> ts{0};
    if (auto c = cuts.find(i); c != cuts.end()) {
      std::stable_sort(c->second.begin(), c->second.end());
      ts.insert(ts.end(), c->second.begin(), c->second.end());
    }
    ts.push_back(1);
    for (size_t j = 1; j < ts.size(); j++) out.push_back(trim(segs[i], ts[j - 1], ts[j]));
  }
  return out;
}

}  // namespace

std::vector<Cubic> aligned(const std::vector<Cubic>& segs, bool closed, int i, NodeKind kind, HandleLead lead, Pt metric) {
  auto [inSeg, outSeg] = handlesOf(int(segs.size()), closed, i);
  if (kind == NodeKind::corner || inSeg < 0 || outSeg < 0) return segs;
  auto iso = [&](Pt p) { return toMetric(p, metric); };
  Pt N = iso(segs[outSeg][0]);
  Pt hi = N - iso(segs[inSeg][2]), ho = iso(segs[outSeg][1]) - N;
  double li = curves::hypot(hi[0], hi[1]), lo = curves::hypot(ho[0], ho[1]);
  if (li < 1e-12 || lo < 1e-12) return segs;
  Pt both = hi * (1 / li) + ho * (1 / lo);
  if (lead == HandleLead::both && curves::hypot(both[0], both[1]) < 1e-9) return segs;
  Pt T = lead == HandleLead::in ? hi * (1 / li) : lead == HandleLead::out ? ho * (1 / lo) : norm(both);
  if (kind == NodeKind::symmetric) li = lo = lead == HandleLead::in ? li : lead == HandleLead::out ? lo : (li + lo) / 2;
  std::vector<Cubic> out = segs;
  out[inSeg][2] = fromMetric(N - T * li, metric);
  out[outSeg][1] = fromMetric(N + T * lo, metric);
  return out;
}

Path::Path(Pt start, std::vector<Cubic> segs, bool closed, std::optional<Heading> arrives, Pt metric,
           std::optional<std::vector<std::optional<NodeKind>>> kinds)
    : start_(start), segs_(std::move(segs)), closed_(closed), arrives_(arrives), metric_(metric), kinds_(std::move(kinds)) {}

Path Path::of(std::vector<Cubic> segs, bool closed, Pt metric) {
  if (segs.empty()) throw std::invalid_argument("a path of no segments has no start");
  Pt s = segs[0][0];
  return Path(s, std::move(segs), closed, std::nullopt, metric);
}

Path Path::measured(Pt metric) const { return Path(start_, segs_, closed_, arrives_, metric, kinds_); }

Path Path::reshaped(std::vector<Cubic> segs) const {
  auto kinds = segs.size() == segs_.size() ? kinds_ : std::nullopt;
  Pt s = segs.empty() ? start_ : segs[0][0];
  return Path(s, std::move(segs), closed_, arrives_, metric_, kinds);
}

NodeKind Path::kind(int i) const {
  if (kinds_ && i >= 0 && i < int(kinds_->size()) && (*kinds_)[i]) return *(*kinds_)[i];
  return kindOf(segs_, closed_, i, metric_);
}

std::vector<NodeKind> Path::kinds() const {
  std::vector<NodeKind> out;
  for (int i = 0; i < nodeCount(); i++) out.push_back(kind(i));
  return out;
}

Path Path::handle(int i, HandleSide side, Pt to) const {
  NodeKind k = kind(i);
  auto [inSeg, outSeg] = handlesOf(int(segs_.size()), closed_, i);
  int seg = side == HandleSide::in ? inSeg : outSeg;
  if (seg < 0) throw std::invalid_argument("node " + std::to_string(i) + " has no " + (side == HandleSide::in ? "in" : "out") + " handle");
  std::vector<Cubic> moved = segs_;
  moved[seg][side == HandleSide::in ? 2 : 1] = to;
  auto segs = aligned(moved, closed_, i, k, side == HandleSide::in ? HandleLead::in : HandleLead::out, metric_);
  auto kinds = explicitKinds();
  kinds[i] = k;
  return Path(start_, segs, closed_, arrives_, metric_, kinds);
}

std::vector<std::optional<NodeKind>> Path::explicitKinds() const {
  std::vector<std::optional<NodeKind>> out;
  for (int i = 0; i < nodeCount(); i++) out.push_back(kinds_ && i < int(kinds_->size()) ? (*kinds_)[i] : std::nullopt);
  return out;
}

Cubic Path::iso(int i) const {
  Cubic c = segs_[i];
  for (Pt& p : c) p = toMetric(p, metric_);
  return c;
}

const std::vector<double>& Path::lens() const {
  if (!lens_) {
    std::vector<double> ls;
    for (int i = 0; i < int(segs_.size()); i++) ls.push_back(arcLength(iso(i)));
    lens_ = std::move(ls);
  }
  return *lens_;
}

double Path::length() const { return sumTo(lens(), int(lens().size())); }

Located Path::locate(Progress p) const {
  const auto& ls = lens();
  double total = length();
  double s = js::min(js::max(p, 0), 1) * total;
  for (int i = 0; i < int(ls.size()); i++) {
    if (s <= ls[i] || i == int(ls.size()) - 1) return {i, tAtLength(iso(i), s, ls[i])};
    s -= ls[i];
  }
  return {0, 0};
}

Progress Path::progress(int i, double t) const {
  double total = js::orElse(length(), 1);
  double before = sumTo(lens(), i);
  return (before + arcLength(iso(i), t)) / total;
}

Pt Path::end() const { return segs_.empty() ? start_ : segs_.back()[3]; }

Path Path::then(const Cubic& c, std::optional<Heading> arrives) const {
  auto segs = segs_;
  segs.push_back(c);
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (kinds_) { kinds = explicitKinds(); kinds->push_back(std::nullopt); }
  return Path(start_, segs, false, arrives, metric_, kinds);
}

Path Path::line(Pt to) const { return then(lineCubic(end(), to)); }

Path Path::curve(Pt c1, Pt c2, Pt to) const { return then({end(), c1, c2, to}); }

Path Path::quarter(Pt to, const QuarterOptions& o) const {
  Pt a = end();
  Heading axis = o.from ? *o.from : arrives_ ? *arrives_ : Heading::v;
  double k = o.tension;
  Cubic c = axis == Heading::v
      ? Cubic{a, Pt{a[0], a[1] + k * (to[1] - a[1])}, Pt{to[0] + k * (a[0] - to[0]), to[1]}, to}
      : Cubic{a, Pt{a[0] + k * (to[0] - a[0]), a[1]}, Pt{to[0], to[1] + k * (a[1] - to[1])}, to};
  return then(c, axis == Heading::v ? Heading::h : Heading::v);
}

Path Path::arch(Pt to, double peak, double tension) const {
  return quarter({(end()[0] + to[0]) / 2, peak}, {Heading::v, tension}).quarter(to, {std::nullopt, tension});
}

Path Path::join(const Path& next) const {
  Pt e = end();
  std::vector<Cubic> link;
  if (!(curves::hypot(next.start_[0] - e[0], next.start_[1] - e[1]) < 1e-9)) link.push_back(lineCubic(e, next.start_));
  // This one's nodes, then next's (its start a node of its own when a line runs to it); the node where they meet is
  // read afresh.
  std::optional<std::vector<std::optional<NodeKind>>> ks;
  if (kinds_ || next.kinds_) {
    auto mine = explicitKinds(), theirs = next.explicitKinds();
    ks.emplace(mine.begin(), mine.end() - (mine.empty() ? 0 : 1));
    ks->push_back(std::nullopt);
    if (!link.empty()) ks->push_back(std::nullopt);
    if (!theirs.empty()) ks->insert(ks->end(), theirs.begin() + 1, theirs.end());
  }
  auto segs = segs_;
  segs.insert(segs.end(), link.begin(), link.end());
  segs.insert(segs.end(), next.segs_.begin(), next.segs_.end());
  return Path(start_, segs, false, next.arrives_, metric_, ks);
}

Path Path::close() const {
  Pt e = end();
  auto segs = segs_;
  if (!(curves::hypot(e[0] - start_[0], e[1] - start_[1]) < 1e-9)) segs.push_back(lineCubic(e, start_));
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (kinds_) {
    auto k = explicitKinds();
    k.resize(std::min(k.size(), segs.size()));
    kinds = k;
  }
  return Path(start_, segs, true, std::nullopt, metric_, kinds);
}

Pt Path::at(Progress p) const {
  if (segs_.empty()) return start_;
  auto [i, t] = locate(p);
  return bez(segs_[i], t);
}

std::vector<Pt> Path::crossY(double y) const {
  std::vector<Pt> out;
  for (auto& c : cross(1, y)) out.push_back(c.p);
  return out;
}
std::vector<Pt> Path::crossX(double x) const {
  std::vector<Pt> out;
  for (auto& c : cross(0, x)) out.push_back(c.p);
  return out;
}

Pt Path::atY(double y, Which which) const { return pick(cross(1, y), which, "height " + jsNum(y)).p; }
Pt Path::atX(double x, Which which) const { return pick(cross(0, x), which, "x " + jsNum(x)).p; }
Progress Path::tAtY(double y, Which which) const { return pick(cross(1, y), which, "height " + jsNum(y)).t; }
Progress Path::tAtX(double x, Which which) const { return pick(cross(0, x), which, "x " + jsNum(x)).t; }

Path Path::chordY(double y) const { return chord(atY(y, End::first), atY(y, End::last)); }
Path Path::chordX(double x) const { return chord(atX(x, End::first), atX(x, End::last)); }

Path Path::chord(Pt a, Pt b) const { return Path(a, {lineCubic(a, b)}, false, std::nullopt, metric_); }

Path Path::slice(Progress p0, Progress p1) const {
  if (!(p1 > p0)) throw std::invalid_argument("nothing between " + jsNum(p0) + " and " + jsNum(p1));
  auto a = locate(p0), b = locate(p1);
  std::vector<Cubic> segs;
  for (int i = a.i; i <= b.i; i++) {
    double u0 = i == a.i ? a.t : 0, u1 = i == b.i ? b.t : 1;
    if (u1 - u0 > 1e-9) segs.push_back(trim(segs_[i], u0, u1));
  }
  return Path::of(segs, false, metric_);
}

Path Path::aperture(double offset, double angle) const {
  if (!closed_) throw std::invalid_argument("only a closed path has an aperture");
  double a = rad(angle);
  Pt d = toMetric({std::cos(a), std::sin(a)}, metric_);
  auto along = [&](Pt p) { return p[0] * d[0] + p[1] * d[1]; };
  double lo = INFINITY, hi = -INFINITY;
  for (auto& c : segs_)
    for (int k = 0; k <= 32; k++) { double v = along(bez(c, k / 32.0)); lo = js::min(lo, v); hi = js::max(hi, v); }
  double cut = (lo + hi) / 2 + offset * (hi - lo) / 2;
  std::vector<Progress> ts;
  for (int i = 0; i < int(segs_.size()); i++) {
    const Cubic& c = segs_[i];
    Cubic flat{Pt{along(c[0]), 0}, Pt{along(c[1]), 0}, Pt{along(c[2]), 0}, Pt{along(c[3]), 0}};
    for (double t : roots(flat, 0, cut)) {
      double p = progress(i, t);
      if (!std::any_of(ts.begin(), ts.end(), [&](double q) { return std::abs(q - p) < 1e-9 || std::abs(std::abs(q - p) - 1) < 1e-9; }))
        ts.push_back(p);
    }
  }
  if (ts.size() < 2) throw std::invalid_argument("the cut at offset " + jsNum(offset) + " doesn't open the path");
  std::stable_sort(ts.begin(), ts.end());
  // The pieces between crossings in turn, the last wrapping round the start; keep the longest behind the cut.
  std::optional<std::array<Progress, 2>> best;
  double most = 0;
  for (size_t k = 0; k < ts.size(); k++) {
    double p0 = ts[k], p1 = ts[(k + 1) % ts.size()], span = p1 > p0 ? p1 - p0 : 1 - p0 + p1, mid = p0 + span / 2;
    if (along(at(mid > 1 ? mid - 1 : mid)) < cut && span > most) { best = {p0, p1}; most = span; }
  }
  if (!best) throw std::invalid_argument("the cut at offset " + jsNum(offset) + " leaves nothing");
  auto [p0, p1] = *best;
  if (p1 > p0) return slice(p0, p1);
  std::vector<Cubic> segs;
  if (p0 < 1 - 1e-9) for (auto& c : slice(p0, 1).segs_) segs.push_back(c);
  if (p1 > 1e-9) for (auto& c : slice(0, p1).segs_) segs.push_back(c);
  return Path::of(segs, false, metric_);
}

Path Path::divide(int n) const {
  std::map<int, std::vector<double>> cuts;
  for (int k = 1; k < n; k++) {
    auto [i, t] = locate(double(k) / n);
    if (t > 1e-9 && t < 1 - 1e-9) cuts[i].push_back(t);
  }
  return Path(start_, cutSegs(segs_, cuts), closed_, arrives_, metric_);
}

Pt Path::direction(Progress p) const {
  if (segs_.empty()) return {1, 0};
  auto [i, t] = locate(p);
  Cubic c = iso(i);
  Pt d = bezD(c, t);
  if (curves::hypot(d[0], d[1]) > 1e-12) return norm(d);
  Pt e = t < 0.5 ? c[2] - c[0] : c[3] - c[1];
  return curves::hypot(e[0], e[1]) > 1e-12 ? norm(e) : norm(c[3] - c[0]);
}

std::vector<PathCrossing> Path::crossings(const Path& other) const {
  std::vector<PathCrossing> out;
  for (int i = 0; i < int(segs_.size()); i++)
    for (int j = 0; j < int(other.segs_.size()); j++) {
      const Cubic &a = segs_[i], &b = other.segs_[j];
      auto cs = curves::crossings(a, b);
      if (!cs) continue;
      for (auto [t, u] : *cs) {
        Pt p = bez(a, t);
        if (!std::any_of(out.begin(), out.end(), [&](const PathCrossing& o) { return curves::hypot(o.p[0] - p[0], o.p[1] - p[1]) < 1e-6; }))
          out.push_back({p, progress(i, t), other.progress(j, u)});
      }
    }
  std::stable_sort(out.begin(), out.end(), [](const PathCrossing& x, const PathCrossing& y) { return x.t < y.t; });
  return out;
}

Path Path::cut(const std::vector<Progress>& ps) const {
  std::map<int, std::vector<double>> cuts;
  for (double p : ps) {
    auto [i, t] = locate(p);
    auto& at = cuts[i];
    if (t > 1e-9 && t < 1 - 1e-9 && !std::any_of(at.begin(), at.end(), [&](double u) { return std::abs(u - t) < 1e-9; }))
      at.push_back(t);
    if (at.empty()) cuts.erase(i);
  }
  if (cuts.empty()) return *this;
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (kinds_) {
    auto k = explicitKinds();
    kinds.emplace();
    for (int i = 0; i < int(segs_.size()); i++) {
      kinds->push_back(k[i]);
      if (auto c = cuts.find(i); c != cuts.end()) kinds->insert(kinds->end(), c->second.size(), std::nullopt);
    }
    if (!closed_) kinds->push_back(k.empty() ? std::nullopt : k.back());
  }
  return Path(start_, cutSegs(segs_, cuts), closed_, arrives_, metric_, kinds);
}

double Path::angle() const {
  Pt a = start_, b = end();
  return std::atan2(b[1] - a[1], b[0] - a[0]) * 180 / PI;
}

std::vector<Path::Cross> Path::cross(int coord, double v) const {
  std::vector<Cross> out;
  for (int i = 0; i < int(segs_.size()); i++) {
    const Cubic& c = segs_[i];
    for (double t : roots(c, coord, v)) {
      Pt p = bez(c, t);
      if (out.empty() || curves::hypot(p[0] - out.back().p[0], p[1] - out.back().p[1]) > 1e-7) out.push_back({p, progress(i, t)});
    }
  }
  return out;
}

int Path::nodeCount() const { return closed_ ? int(segs_.size()) : int(segs_.size()) + 1; }

Pt Path::node(int i) const { return i == 0 ? start_ : segs_.at(i - 1)[3]; }

Progress Path::within(Segs segs, Progress p) const {
  int n = int(segs_.size());
  auto idx = [&](int i) { return i < 0 ? n + i : i; };
  int i0 = idx(segs.a), i1 = idx(segs.b);
  if (!(i0 >= 0 && i1 < n && i0 <= i1)) {
    std::string s = segs.one ? std::to_string(segs.a) : std::to_string(segs.a) + "," + std::to_string(segs.b);
    throw std::invalid_argument("segments " + s + " are not in a path of " + std::to_string(n));
  }
  double a = nodeProgress(i0), b = nodeProgress(i1 + 1);
  return a + (b - a) * p;
}

Progress Path::nodeProgress(int i) const {
  double total = length();
  return js::truthy(total) ? sumTo(lens(), i) / total : 0;
}

std::vector<Pt> Path::nodes() const {
  std::vector<Pt> out{start_};
  for (auto& c : segs_) out.push_back(c[3]);
  return out;
}

std::vector<double> Path::distances() const {
  std::vector<double> out{0};
  for (double l : lens()) out.push_back(out.back() + l);
  return out;
}

Path Path::reverse() const {
  std::vector<Cubic> segs;
  for (auto it = segs_.rbegin(); it != segs_.rend(); ++it) segs.push_back(curves::reverse(*it));
  int n = int(segs_.size());
  // Open, the nodes run backwards; closed, node j is the old node n − j (its start stays its start).
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (kinds_) {
    auto k = explicitKinds();
    if (closed_) {
      kinds.emplace();
      for (int j = 0; j < int(k.size()); j++) kinds->push_back(k[(n - j) % n]);
    } else {
      std::reverse(k.begin(), k.end());
      kinds = k;
    }
  }
  Pt s = segs.empty() ? start_ : segs[0][0];
  return Path(s, segs, closed_, std::nullopt, metric_, kinds);
}

Path Path::startAt(int r) const {
  int n = int(segs_.size());
  std::vector<Cubic> segs(segs_.begin() + r, segs_.end());
  segs.insert(segs.end(), segs_.begin(), segs_.begin() + r);
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (kinds_) {
    auto k = explicitKinds();
    kinds.emplace();
    for (int j = 0; j < int(k.size()); j++) kinds->push_back(k[(j + r) % n]);
  }
  return Path(segs_.at(r % n)[0], segs, true, std::nullopt, metric_, kinds);
}

Path Path::map(const std::function<Pt(Pt)>& f) const {
  std::vector<Cubic> segs = segs_;
  Pt s = f(start_);
  for (auto& c : segs) for (Pt& p : c) p = f(p);
  return Path(s, segs, closed_, arrives_, metric_, kinds_);
}

Path Path::warp(const Warp& f, const WarpOptions& o) const {
  auto segs = warpCubics(segs_, f, o);
  return segs.size() ? reshaped(segs) : Path(f(start_), segs, closed_, arrives_, metric_, kinds_);
}

Path from(Pt p) { return Path(p); }

Path line(Pt a, Pt b) { return from(a).line(b); }

Path poly(Pt p, const std::vector<Pt>& ps) {
  Path path = from(p);
  for (Pt q : ps) path = path.line(q);
  return path;
}

}  // namespace curves
