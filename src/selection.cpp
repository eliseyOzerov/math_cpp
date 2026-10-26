#include "curves/selection.hpp"

#include "js.hpp"

#include <map>
#include <set>
#include <stdexcept>
#include <string>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** Math.max(a, Math.min(b, v)) */
double clampJs(double v, double a, double b) { return js::max(a, js::min(b, v)); }

/** Which of `path`'s nodes or cubics index `k` means, counted from the end when below 0. */
int index(int k, int n) { return k < 0 ? n + k : k; }

/** The distinct values of `xs`, in order of first appearance (a JS Set). */
std::vector<int> distinct(const std::vector<int>& xs) {
  std::vector<int> out;
  std::set<int> seen;
  for (int x : xs) if (seen.insert(x).second) out.push_back(x);
  return out;
}

Picked makePicked(const Path& path, const std::vector<double>& weights) {
  Picked p{path, {}, weights};
  for (size_t i = 0; i < weights.size(); i++) if (weights[i] > 0) p.nodes.push_back(int(i));
  return p;
}

/** The transition's first handle, as a share of the way from its start on the line to the corner. */
constexpr double LEAN = 0.3;

/** How closely two directions must agree for segments to meet along a tangent rather than at a corner: 2°. */
const double TANGENT = std::cos(rad(2));

/** The line `L` running into the curve `C`, eased (see selection.hpp): the line shortened, the transition, and what
 *  is left of the curve; nothing when the line is too short even for a sliver of the curve. */
std::optional<std::vector<Cubic>> ease(const Cubic& L, const Cubic& C, double reach) {
  Pt N = L[3], t = norm(N - L[0]);
  double clen = arcLength(C);
  for (double r = js::min(reach, 1); r > 1.0 / 64; r /= 2) {
    double u = r >= 1 ? 1 : tAtLength(C, r * clen, clen);
    std::optional<Cubic> rest;
    if (u < 1 - 1e-9) rest = split(C, u)[1];
    Pt Q = rest ? (*rest)[0] : C[3], T = norm(bezD(C, u));
    double k = std::fabs(curvature(C, u)), sin = cross(t, T);
    // The corner X, where the line and the tangent at Q cross: N + s·t = Q + w·T.
    auto m = meet(N, t, Q, T);
    if (!m) continue;
    double s = (*m)[0];
    if (s <= 0) continue;
    Pt X = N + t * s;
    double b = dist(Q, X), x = 1.5 * k * b * b / std::fabs(sin) / (1 - LEAN);
    Pt B0 = X - t * x;
    if (dot(B0 - L[0], t) <= 1e-9 * clen) continue;
    std::vector<Cubic> out = {lineCubic(L[0], B0), Cubic{B0, B0 + t * (LEAN * x), X, Q}};
    if (rest) out.push_back(*rest);
    return out;
  }
  return std::nullopt;
}

/** The curves `A` and `B`, meeting along a tangent, with their handles at the node resized so their curvatures meet
 *  at the geometric mean; as they were when they bend opposite ways or a handle sits on the node. */
std::array<Cubic, 2> match(const Cubic& A, const Cubic& B) {
  Pt N = A[3], T = endTangents(Piece::of(A))[1];
  double hA = dist(A[3], A[2]), hB = dist(B[1], B[0]);
  double cA = cross(T, A[1] - N), cB = cross(T, B[2] - N);
  if (hA < 1e-12 || hB < 1e-12 || cA * cB <= 0) return {A, B};
  double kA = 2.0 / 3 * std::fabs(cA) / (hA * hA), kB = 2.0 / 3 * std::fabs(cB) / (hB * hB), k = std::sqrt(kA * kB);
  double ha = std::sqrt(2.0 / 3 * std::fabs(cA) / k), hb = std::sqrt(2.0 / 3 * std::fabs(cB) / k);
  return {Cubic{A[0], A[1], N - T * ha, N}, Cubic{N, N + T * hb, B[2], B[3]}};
}

Pt metricOf(const std::vector<Picked>& p) { return p.empty() ? Pt{1, 1} : p[0].path.metric(); }

}  // namespace

Pick nodeAt(std::vector<int> i) {
  return {[i](const Path& path) {
    std::vector<int> pick;
    for (int k : i) pick.push_back(index(k, path.nodeCount()));
    return PickOn{path, NodePick(pick)};
  }};
}

Pick cubicAt(std::vector<int> i) {
  return {[i](const Path& path) {
    std::vector<int> all;
    for (int k : i) { int c = index(k, int(path.segs().size())); all.push_back(c); all.push_back(c + 1); }
    return PickOn{path, NodePick(distinct(all))};
  }};
}

Pick section(Progress from, Progress to) {
  return {[from, to](const Path& path) {
    Path cut = path.cut({from, to});
    double a = from <= to ? from : to, b = from <= to ? to : from;
    std::function<double(Pt, int, Progress)> f = [cut, a, b](Pt, int i, Progress) {
      double t = cut.nodeProgress(i);
      return t >= a - 1e-6 && t <= b + 1e-6 ? 1.0 : 0.0;
    };
    return PickOn{cut, NodePick(f)};
  }};
}

Falloff falloff(const FalloffSpec& f) {
  if (auto fn = std::get_if<Falloff>(&f)) return *fn;
  switch (std::get<FalloffName>(f)) {
    case FalloffName::smooth: return [](double x) { return 1 - x * x * (3 - 2 * x); };
    case FalloffName::linear: return [](double x) { return 1 - x; };
    case FalloffName::sphere: return [](double x) { return std::sqrt(js::max(0, 1 - x * x)); };
    case FalloffName::root: return [](double x) { return std::sqrt(js::max(0, 1 - x)); };
    case FalloffName::sharp: return [](double x) { double v = 1 - x; return v * v; };
    case FalloffName::constant: return [](double) { return 1.0; };
  }
  return [](double) { return 1.0; };
}

std::vector<Cubic> moveNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                             const std::function<Pt(Pt, int)>& f) {
  int n = int(segs.size());
  std::vector<Cubic> out = segs;
  for (int i : distinct(nodes)) {
    if (i < n) { out[i][0] = f(segs[i][0], i); out[i][1] = f(segs[i][1], i); }
    int j = i > 0 ? i - 1 : closed ? n - 1 : -1;
    if (j >= 0) { out.at(j)[3] = f(segs.at(j)[3], i); out[j][2] = f(segs[j][2], i); }
  }
  return out;
}

std::vector<Cubic> smoothNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                               double amount, Pt metric) {
  return smoothNodes(segs, closed, nodes, std::function<double(int)>([amount](int) { return amount; }), metric);
}

std::vector<Cubic> smoothNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes,
                               const std::function<double(int)>& amount, Pt metric) {
  int n = int(segs.size());
  auto iso = [&](Pt p) { return toMetric(p, metric); };
  std::vector<Pt> P;
  P.push_back(iso(segs[0][0]));
  for (auto& c : segs) P.push_back(iso(c[3]));
  int count = closed ? n : n + 1;
  // A direction, or nothing for a zero vector (the fallbacks below take over).
  auto unit = [](Pt v) -> std::optional<Pt> { Pt u = norm(v); return js::truthy(u[0]) || js::truthy(u[1]) ? std::optional<Pt>(u) : std::nullopt; };
  // The tangent at each node, along the path.
  std::vector<std::optional<Pt>> T(count);
  for (int i = 0; i < count; i++) {
    int prev = i > 0 ? i - 1 : closed ? n - 1 : -1, next = i < n ? i + 1 : -1;
    std::optional<Pt> a = prev >= 0 ? unit(P[i] - P[prev]) : std::nullopt, b = next >= 0 ? unit(P[next] - P[i]) : std::nullopt;
    if (a && b) { auto u = unit(*a + *b); T[i] = u ? u : b; continue; }
    // An open end: the way its handle leaves, or the chord.
    if (i == 0) { auto u = unit(iso(segs[0][1]) - P[0]); T[i] = u ? u : b; }
    else { auto u = unit(P[n] - iso(segs[n - 1][2])); T[i] = u ? u : a; }
  }
  // How long a handle segment `j` needs: its chord over 3, longer as its ends' tangents turn.
  auto reach = [&](int j) {
    auto ta = T[j % count], tb = T[(j + 1) % count];
    double c = dist(P[j + 1], P[j]);
    double turnA = ta && tb ? std::acos(clampJs(dot(*ta, *tb), -1, 1)) : 0;
    double co = std::cos(turnA / 4);
    return c / (3 * (co * co));
  };
  std::vector<Cubic> out = segs;
  std::vector<int> picks;
  for (int i : nodes) picks.push_back(closed ? i % n : i);
  for (int i : distinct(picks)) {
    if (i < 0 || i >= count) continue;
    auto t = T[i];
    double k = t ? amount(i) : 0;
    if (!t || !js::truthy(k)) continue;
    auto mix = [&](Pt p, Pt q) -> Pt { return fromMetric(lerp(iso(p), q, k), metric); };
    if (i < n) out[i][1] = mix(segs[i][1], P[i] + *t * reach(i));
    int j = i > 0 ? i - 1 : closed ? n - 1 : -1;
    if (j >= 0) out[j][2] = mix(segs[j][2], P[i] - *t * reach(j));
  }
  return out;
}

Pt inflation(Pt p, Pt from, double r) {
  double dx = p[0] - from[0], dy = p[1] - from[1], l = curves::hypot(dx, dy);
  return l < 1e-12 ? Pt{0, 0} : Pt{r * dx / l, r * dy / l};
}

Faired fairNodes(const std::vector<Cubic>& segs, bool closed, const std::vector<int>& nodes, double reach, Pt metric,
                 const std::optional<std::vector<double>>& weights) {
  std::vector<Cubic> S;
  for (auto& c : segs) { Cubic d = c; for (Pt& p : d) p = toMetric(p, metric); S.push_back(d); }
  int n = int(S.size());
  // sel[i]: node i's weight (the start of segment i, or the open end), 0 when it isn't selected.
  std::vector<double> sel(closed ? n : n + 1, 0.0);
  for (int i : nodes) {
    int at = closed ? i % n : i;
    if (at < 0 || at >= int(sel.size())) continue;
    sel[at] = weights && i >= 0 && i < int(weights->size()) ? (*weights)[i] : 1;
  }
  auto into = [&](int i) { return i > 0 ? i - 1 : closed ? int(S.size()) - 1 : -1; };
  auto outOf = [&](int i) { return i < int(S.size()) ? i : -1; };
  auto tangent = [&](int i) {
    int a = into(i), b = outOf(i);
    return a >= 0 && b >= 0 && a != b && dot(endTangents(Piece::of(S[a]))[1], endTangents(Piece::of(S[b]))[0]) > TANGENT;
  };
  auto lineAt = [&](int i, int seg) { return sel[i] > 0 && tangent(i) && isLine(S[seg]); };

  // A curve with lines at both ends is split at its middle (its shape unchanged), each half eased on its own.
  for (int j = int(S.size()) - 1; j >= 0; j--) {
    int end = closed ? (j + 1) % int(S.size()) : j + 1;
    if (isLine(S[j]) || !lineAt(j, into(j)) || outOf(end) < 0 || !lineAt(end, outOf(end))) continue;
    Cubic c = S[j];
    double clen = arcLength(c);
    auto h = split(c, tAtLength(c, clen / 2, clen));
    S[j] = h[0];
    S.insert(S.begin() + j + 1, h[1]);
    sel.insert(sel.begin() + j + 1, js::min(sel[j], sel[end]));
  }

  // From the last node back, so easing one doesn't renumber the ones still to come.
  for (int i = int(sel.size()) - 1; i >= 0; i--) {
    if (!js::truthy(sel[i]) || !tangent(i)) continue;
    int a = into(i), b = outOf(i);
    Cubic A = S[a], B = S[b];
    if (isLine(A) && isLine(B)) continue;
    if (!isLine(A) && !isLine(B)) { auto m = match(A, B); S[a] = m[0]; S[b] = m[1]; continue; }
    double r = reach * sel[i];
    std::optional<std::vector<Cubic>> out;
    if (isLine(A)) out = ease(A, B, r);
    else {
      out = ease(reverse(B), reverse(A), r);
      if (out) { for (Cubic& c : *out) c = reverse(c); std::reverse(out->begin(), out->end()); }
    }
    if (!out) continue;
    std::vector<Cubic> mid(out->begin() + 1, out->end() - 1);
    double w = sel[i];
    if (b > a) {
      S.erase(S.begin() + a, S.begin() + a + 2);
      S.insert(S.begin() + a, out->begin(), out->end());
      sel.erase(sel.begin() + i);
      sel.insert(sel.begin() + i, out->size() - 1, w);
    } else {
      // Node 0 of a closed path: the last segment and the first change, and what comes between goes on the end.
      S[a] = out->front(); S[0] = out->back();
      S.insert(S.end(), mid.begin(), mid.end());
      sel.insert(sel.end(), mid.size(), w);
    }
  }
  Faired r;
  for (auto& c : S) { Cubic d = c; for (Pt& p : d) p = fromMetric(p, metric); r.segs.push_back(d); }
  for (size_t i = 0; i < sel.size(); i++) if (sel[i] > 0) r.nodes.push_back(int(i));
  r.weights = sel;
  return r;
}

Selection Selection::of(Path path) {
  return of(std::move(path), NodePick(std::function<double(Pt, int, Progress)>([](Pt, int, Progress) { return 1.0; })));
}

Selection Selection::of(Path path, const Pick& pick) {
  PickOn po = pick.on(path);
  return of(std::move(po.path), po.pick);
}

Selection Selection::of(Path path, const NodePick& pick) {
  int n = int(path.segs().size()), count = path.nodeCount();
  std::vector<double> weights;
  for (int i = 0; i < count; i++) {
    if (auto f = std::get_if<std::function<double(Pt, int, Progress)>>(&pick))
      weights.push_back(clampJs(js::orElse((*f)(path.node(i), i, path.nodeProgress(i)), 0), 0, 1));
    else {
      const auto& v = std::get<std::vector<int>>(pick);
      auto has = [&](int k) { return std::find(v.begin(), v.end(), k) != v.end(); };
      weights.push_back(has(i) || (path.closed() && i == 0 && has(n)) ? 1 : 0);
    }
  }
  Picked p = makePicked(path, weights);
  std::vector<Picked> ps;
  ps.push_back(std::move(p));
  return Selection(std::move(ps));
}

Selection Selection::and_(const Selection& other) const { return and_(std::vector<Selection>{other}); }

Selection Selection::and_(const std::vector<Selection>& others) const {
  auto ps = picked;
  for (auto& s : others) ps.insert(ps.end(), s.picked.begin(), s.picked.end());
  return Selection(std::move(ps));
}

std::vector<std::vector<double>> Selection::weights() const {
  std::vector<std::vector<double>> out;
  for (auto& p : picked) out.push_back(p.weights);
  return out;
}

std::vector<Pt> Selection::points() const {
  std::vector<Pt> whole;
  for (auto& p : picked)
    for (size_t i = 0; i < p.weights.size(); i++) if (p.weights[i] >= 1) whole.push_back(p.path.node(int(i)));
  if (!whole.empty()) return whole;
  for (auto& p : picked) for (int i : p.nodes) whole.push_back(p.path.node(i));
  return whole;
}

Pt Selection::center() const { auto ps = points(); return curves::center(ps); }
Box Selection::bounds() const { auto ps = points(); return curves::bounds(ps); }
Pt Selection::metric() const { return metricOf(picked); }

Selection Selection::soften(double reach, const SoftenOptions& o) const {
  Falloff f = falloff(o.falloff ? *o.falloff : FalloffSpec(FalloffName::smooth));
  Along along = o.along ? *o.along : Along::path;
  Pt metric = this->metric();
  auto share = [&](double d) { return d < reach ? clampJs(f(d / reach), 0, 1) : 0.0; };
  std::vector<std::vector<Pt>> at;
  for (auto& p : picked) {
    at.emplace_back();
    for (int i = 0; i < p.path.nodeCount(); i++) at.back().push_back(toMetric(p.path.node(i), metric));
  }
  std::vector<Picked> out;
  for (size_t k = 0; k < picked.size(); k++) {
    const Path& path = picked[k].path;
    const auto& weights = picked[k].weights;
    auto D = path.distances();
    double L = path.length();
    auto along_ = [&](int i, int j) { double d = std::fabs(D[i] - D[j]); return path.closed() ? js::min(d, L - d) : d; };
    std::vector<double> ws;
    for (size_t j = 0; j < weights.size(); j++) {
      double w = weights[j];
      if (along == Along::path) {
        for (size_t i = 0; i < weights.size(); i++) if (weights[i] > 0) w = js::max(w, weights[i] * share(along_(int(i), int(j))));
      } else {
        for (size_t m = 0; m < picked.size(); m++)
          for (size_t i = 0; i < picked[m].weights.size(); i++)
            if (picked[m].weights[i] > 0) w = js::max(w, picked[m].weights[i] * share(dist(at[m][i], at[k][j])));
      }
      ws.push_back(w);
    }
    out.push_back(makePicked(path, ws));
  }
  return Selection(std::move(out));
}

Selection Selection::weigh(const Strength& strength) const {
  Wave s = wave(strength);
  std::vector<Picked> out;
  for (auto& p : picked) {
    std::vector<double> ws;
    for (size_t i = 0; i < p.weights.size(); i++) ws.push_back(clampJs(p.weights[i] * s(p.path.nodeProgress(int(i))), 0, 1));
    out.push_back(makePicked(p.path, ws));
  }
  return Selection(std::move(out));
}

Selection Selection::invert() const {
  std::vector<Picked> out;
  for (auto& p : picked) {
    std::vector<double> ws;
    for (double w : p.weights) ws.push_back(1 - w);
    out.push_back(makePicked(p.path, ws));
  }
  return Selection(std::move(out));
}

Selection Selection::transform(const Affine& m) const { return moveBy([m](double w) { return partly(m, w); }); }

Selection Selection::moveEach(const NodeMap& f) const {
  return reshape([&f](Pt node, const Path& path, int i, double w) {
    auto g = f(node, path, i);
    return std::function<Pt(Pt)>([g, w](Pt p) { return lerp(p, g(p), w); });
  });
}

Selection Selection::put(const Place& to) const {
  Pt c = center();
  Pt d = {to.x ? *to.x - c[0] : 0, to.y ? *to.y - c[1] : 0};
  return moveEach([d](Pt, const Path&, int) { return std::function<Pt(Pt)>([d](Pt p) { return Pt{p[0] + d[0], p[1] + d[1]}; }); });
}

Selection Selection::kind(NodeKind kind) const {
  std::vector<Picked> out;
  for (auto& pk : picked) {
    const Path& path = pk.path;
    std::vector<int> sharp;
    if (!(kind == NodeKind::corner || path.segs().empty()))
      for (int i : pk.nodes) if (path.kind(i) == NodeKind::corner) sharp.push_back(i);
    Path p = !sharp.empty() ? Path::of(smoothNodes(path.segs(), path.closed(), sharp, 1.0, path.metric()), path.closed(), path.metric()) : path;
    out.push_back(makePicked(p.withKind(kind, NodePick(pk.nodes)), pk.weights));
  }
  return Selection(std::move(out));
}

Selection Selection::handles(const HandlesOptions& o) const {
  std::vector<Picked> out;
  for (auto& pk : picked) {
    const Path& path = pk.path;
    int n = int(path.segs().size());
    Pt m = path.metric();
    Path p = path;
    for (int i : pk.nodes) {
      double w = pk.weights[i];
      Pt node = toMetric(path.node(i), m);
      std::vector<HandleSide> sides;
      for (HandleSide s : {HandleSide::in, HandleSide::out}) {
        if (o.side == HandleLead::in && s != HandleSide::in) continue;
        if (o.side == HandleLead::out && s != HandleSide::out) continue;
        if (s == HandleSide::in ? (i > 0 || path.closed()) : i < n) sides.push_back(s);
      }
      for (HandleSide s : sides) {
        // Read off the path as it was: the first side set has already turned a smooth node's other handle.
        int seg = s == HandleSide::out ? i % n : (i > 0 ? i : n) - 1;
        const Cubic& c = path.segs().at(seg);
        Pt v = toMetric(c[s == HandleSide::out ? 1 : 2], m) - node;
        if (len(v) < 1e-12) continue;
        // The turn: to the aim along the shorter way, or as given, then the weight's share of it.
        double now = std::atan2(v[1], v[0]);
        double d;
        if (!o.angle) d = rad(o.turn);
        else {
          double aim = rad(*o.angle) + (s == HandleSide::in ? PI : 0);
          d = std::atan2(std::sin(aim - now), std::cos(aim - now)) + rad(o.turn);
        }
        Pt to = node + curves::rotate(v, d * w) * (1 + (o.length - 1) * w);
        p = p.handle(i, s, fromMetric(to, m));
      }
    }
    out.push_back(makePicked(p, pk.weights));
  }
  return Selection(std::move(out));
}

Selection Selection::inflate(double r, const InflateOptions& o) const {
  Pt metric = this->metric(), O = toMetric(within(bounds(), o.align), metric);
  Wave s = wave(o.strength);
  Selection moved = reshape([&](Pt node, const Path& path, int i, double w) {
    Pt n = toMetric(node, metric);
    double l = dist(n, O);
    double x = o.along == InflateAlong::path ? path.nodeProgress(i)
                                             : js::mod(js::mod(std::atan2(n[1] - O[1], n[0] - O[0]) / (2 * PI), 1) + 1, 1);
    double ri = r * s(x) * w;
    Pt d = inflation(n, O, ri);
    double k = 1 + o.handles * ((l > 1e-12 ? js::max(0, l + ri) / l : 1) - 1);
    return std::function<Pt(Pt)>([=](Pt p) { return fromMetric((n + d) + (toMetric(p, metric) - n) * k, metric); });
  });
  return js::truthy(o.smooth) ? moved.smooth(o.smooth) : moved;
}

Selection Selection::smooth(double amount) const {
  std::vector<Picked> out;
  for (auto& p : picked) {
    if (!p.path.segs().empty()) {
      const auto& ws = p.weights;
      Path np = Path::of(smoothNodes(p.path.segs(), p.path.closed(), p.nodes, std::function<double(int)>([&](int i) { return amount * ws[i]; }), p.path.metric()), p.path.closed(), p.path.metric());
      out.push_back(Picked{np, p.nodes, p.weights});
    } else out.push_back(p);
  }
  return Selection(std::move(out));
}

Selection Selection::fair(double reach) const {
  std::vector<Picked> out;
  for (auto& p : picked) {
    if (p.path.segs().empty()) { out.push_back(p); continue; }
    Faired r = fairNodes(p.path.segs(), p.path.closed(), p.nodes, reach, p.path.metric(), p.weights);
    out.push_back(makePicked(Path::of(r.segs, p.path.closed(), p.path.metric()), r.weights));
  }
  return Selection(std::move(out));
}

Selection Selection::rotate(double deg, const Pivot& about) const {
  Pt o = pivot(about);
  return moveBy([=](double w) { return rotation(deg * w, o); });
}

Selection Selection::shear(double deg, const ShearOptions& opt) const {
  Pt o = pivot(opt.about);
  double axis = opt.axis;
  return moveBy([=](double w) { return curves::shear(deg * w, axis, o); });
}

Selection Selection::shift(double dist, double axis) const {
  return moveBy([=](double w) { return curves::shift(dist * w, axis); });
}

Selection Selection::scale(double k, const Pivot& about) const { return scale(Pt{k, k}, about); }

Selection Selection::scale(Pt k, const Pivot& about) const {
  double kx = k[0], ky = k[1];
  Pt o = pivot(about);
  return moveBy([=](double w) { return scaling(1 + (kx - 1) * w, 1 + (ky - 1) * w, o); });
}

Selection Selection::mirror(double axis, const Pivot& about) const {
  Pt o = pivot(about);
  Affine m = then(then(rotation(-axis, o), scaling(1, -1, o)), rotation(axis, o));
  return moveBy([m](double w) { return partly(m, w); });
}

std::vector<Path> Selection::paths() const {
  std::vector<Path> out;
  for (auto& p : picked) out.push_back(p.path);
  return out;
}

Selection Selection::select() const {
  std::vector<Picked> out;
  for (auto& p : paths()) { auto s = Selection::of(p); out.insert(out.end(), s.picked.begin(), s.picked.end()); }
  return Selection(std::move(out));
}

Selection Selection::select(const NodePick& pick) const {
  std::vector<Picked> out;
  for (auto& p : paths()) { auto s = Selection::of(p, pick); out.insert(out.end(), s.picked.begin(), s.picked.end()); }
  return Selection(std::move(out));
}

Selection Selection::select(const Pick& pick) const {
  std::vector<Picked> out;
  for (auto& p : paths()) { auto s = Selection::of(p, pick); out.insert(out.end(), s.picked.begin(), s.picked.end()); }
  return Selection(std::move(out));
}

Path Selection::path() const {
  if (picked.size() != 1) throw std::runtime_error("the selection spans " + std::to_string(picked.size()) + " paths; take .paths");
  return picked[0].path;
}

Selection Selection::moveBy(const std::function<Affine(double)>& at) const {
  Pt metric = this->metric();
  return reshape([&at, metric](Pt, const Path&, int, double w) {
    Affine m = inMetric(at(w), metric);
    return std::function<Pt(Pt)>([m](Pt p) { return curves::apply(m, p); });
  });
}

Selection Selection::reshape(const std::function<std::function<Pt(Pt)>(Pt, const Path&, int, double)>& f) const {
  std::vector<Picked> out;
  for (auto& pk : picked) {
    const Path& path = pk.path;
    std::map<int, std::function<Pt(Pt)>> maps;
    for (int i : pk.nodes) maps[i] = f(path.node(i), path, i, pk.weights[i]);
    if (path.segs().empty()) {
      bool has0 = std::find(pk.nodes.begin(), pk.nodes.end(), 0) != pk.nodes.end();
      out.push_back(Picked{has0 ? Path(maps.at(0)(path.start()), {}, path.closed(), std::nullopt, path.metric()) : path, pk.nodes, pk.weights});
      continue;
    }
    out.push_back(Picked{path.reshaped(moveNodes(path.segs(), path.closed(), pk.nodes, [&](Pt p, int i) { return maps.at(i)(p); })), pk.nodes, pk.weights});
  }
  return Selection(std::move(out));
}

Pt Selection::pivot(const Pivot& about) const { return toMetric(about ? *about : center(), metric()); }

Selection group(const std::vector<Path>& of) {
  std::vector<Picked> out;
  for (auto& p : of) { auto s = Selection::of(p); out.insert(out.end(), s.picked.begin(), s.picked.end()); }
  return Selection(std::move(out));
}

Selection group(const std::vector<Selection>& of) {
  std::vector<Path> ps;
  for (auto& s : of) for (auto& p : s.paths()) ps.push_back(p);
  return group(ps);
}

Selection Path::select() const { return Selection::of(*this); }
Selection Path::select(const NodePick& pick) const { return Selection::of(*this, pick); }
Selection Path::select(const Pick& pick) const { return Selection::of(*this, pick); }

Path Path::withKind(NodeKind kind) const { return withKind(kind, NodePick(select().picked[0].nodes)); }

Path Path::withKind(NodeKind kind, const NodePick& pick) const {
  auto nodes = select(pick).picked[0].nodes;
  auto kinds = explicitKinds();
  std::vector<Cubic> segs = segs_;
  for (int i : nodes) { kinds[i] = kind; segs = aligned(segs, closed_, i, kind, HandleLead::both, metric_); }
  return Path(segs.empty() ? start_ : segs[0][0], segs, closed_, arrives_, metric_, kinds);
}

}  // namespace curves
