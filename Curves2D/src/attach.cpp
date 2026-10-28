#include "curves/attach.hpp"

#include "js.hpp"
#include "curves/selection.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** A mount's point on the path it is attached to. */
Pt mounted(const Mount& a) { return a.on.at(!a.segs ? a.at : a.on.within(*a.segs, a.at)); }

/** The progress pair `k` of `n` takes on the attached path by default. */
double defaultFrom(int k, int n) { return n == 1 ? 0 : double(k) / (n - 1); }

using Map = std::function<Pt(Pt)>;

/** The affine map taking the three points `s` onto `d`, or none when `s` lie on a line. */
Map exact(const std::vector<Pt>& s, const std::vector<Pt>& d) {
  Pt a = s[0], b = s[1], c = s[2], u = {b.x - a.x, b.y - a.y}, v = {c.x - a.x, c.y - a.y};
  double det = u.x * v.y - u.y * v.x;
  if (std::abs(det) < 1e-9 * js::max(1, curves::hypot(u.x, u.y) * curves::hypot(v.x, v.y))) return nullptr;
  Pt A = d[0], B = d[1], C = d[2], U = {B.x - A.x, B.y - A.y}, V = {C.x - A.x, C.y - A.y};
  return [=](Pt q) {
    // q's coordinates in (u, v) from a, carried to (U, V) from A.
    Pt r = {q.x - a.x, q.y - a.y};
    double x = (r.x * v.y - r.y * v.x) / det, y = (u.x * r.y - u.y * r.x) / det;
    return Pt{A.x + x * U.x + y * V.x, A.y + x * U.y + y * V.y};
  };
}

/** The affine map nearest to taking `s` onto `d` in least squares, or none when `s` lie on a line. */
Map fitted(const std::vector<Pt>& s, const std::vector<Pt>& d) {
  double n = double(s.size());
  double cx = 0, cy = 0, dx = 0, dy = 0;
  for (auto& p : s) cx += p.x;
  for (auto& p : s) cy += p.y;
  for (auto& p : d) dx += p.x;
  for (auto& p : d) dy += p.y;
  cx /= n; cy /= n; dx /= n; dy /= n;
  // Centred, the translation drops out: solve [sxx sxy; sxy syy] [a; c] = [Σx·X; Σy·X], and the same for Y.
  double sxx = 0, sxy = 0, syy = 0, xX = 0, yX = 0, xY = 0, yY = 0;
  for (size_t k = 0; k < s.size(); k++) {
    Pt p = s[k];
    double x = p.x - cx, y = p.y - cy, X = d[k].x - dx, Y = d[k].y - dy;
    sxx += x * x; sxy += x * y; syy += y * y; xX += x * X; yX += y * X; xY += x * Y; yY += y * Y;
  }
  double det = sxx * syy - sxy * sxy;
  if (std::abs(det) < 1e-9 * js::max(1, sxx * syy)) return nullptr;
  double a = (xX * syy - yX * sxy) / det, c = (yX * sxx - xX * sxy) / det;
  double b = (xY * syy - yY * sxy) / det, e = (yY * sxx - xY * sxy) / det;
  return [=](Pt q) { return Pt{dx + a * (q.x - cx) + c * (q.y - cy), dy + b * (q.x - cx) + e * (q.y - cy)}; };
}

/** `p` bent node by node so its points at progress `from` land on `dst` (in the metric): a node cut in at each, moved
 *  the rest of the way with its handles, and the nodes between moved by a blend of their neighbours' moves. */
Path bent(const Path& p, const std::vector<Progress>& from, const std::vector<Pt>& dst) {
  Pt m = p.metric();
  Path q = p.cut(from);
  std::vector<double> at;
  for (double l : q.distances()) at.push_back(l / js::orElse(q.length(), 1));
  struct Move {
    Progress s;
    Pt r;
  };
  std::vector<Move> order;
  for (size_t k = 0; k < from.size(); k++) {
    Pt w = toMetric(q.at(from[k]), m);
    order.push_back({from[k], {dst[k].x - w.x, dst[k].y - w.y}});
  }
  std::stable_sort(order.begin(), order.end(), [](const Move& x, const Move& y) { return x.s - y.s < 0; });
  auto moveAt = [&](Progress s) -> Pt {
    int j = -1;
    for (int k = 0; k < int(order.size()); k++)
      if (order[k].s >= s) { j = k; break; }
    if (j == 0) return order[0].r;
    if (j < 0) return order.back().r;
    const Move &a = order[j - 1], &b = order[j];
    double w = b.s - a.s < 1e-12 ? 1 : (s - a.s) / (b.s - a.s);
    return {a.r.x + (b.r.x - a.r.x) * w, a.r.y + (b.r.y - a.r.y) * w};
  };
  std::vector<int> nodes(q.nodeCount());
  for (int i = 0; i < q.nodeCount(); i++) nodes[i] = i;
  auto segs = moveNodes(q.segs(), q.closed(), nodes, [&](Pt pt, int i) {
    Pt t = toMetric(pt, m), d = moveAt(at[i]);
    return fromMetric(Pt{t.x + d.x, t.y + d.y}, m);
  });
  return q.reshaped(segs);
}

}  // namespace

Path attach(const Path& p, const Mount& a, const std::optional<Mount>& b) {
  Pt m = p.metric();
  auto to = [&](Pt q) { return toMetric(q, m); };
  auto back = [&](Pt q) { return fromMetric(q, m); };
  Pt pa = to(p.at(a.from.value_or(0))), A = to(mounted(a));
  auto shift = [&](Pt q) -> Pt {
    Pt d = to(q);
    return back({d.x - pa.x + A.x, d.y - pa.y + A.y});
  };
  if (!b) return p.map(shift);
  Pt pb = to(p.at(b->from.value_or(1))), B = to(mounted(*b));
  Pt u = {pb.x - pa.x, pb.y - pa.y}, v = {B.x - A.x, B.y - A.y};
  double lu = curves::hypot(u.x, u.y), lv = curves::hypot(v.x, v.y);
  if (lu < 1e-9 || lv < 1e-9) return p.map(shift);
  // In u's frame: along it scaled to v's length, across it kept, then turned onto v.
  double ux = u.x / lu, uy = u.y / lu, vx = v.x / lv, vy = v.y / lv, k = lv / lu;
  return p.map([&](Pt q) -> Pt {
    Pt r = to(q), d = {r.x - pa.x, r.y - pa.y};
    double s = (d.x * ux + d.y * uy) * k, c = -d.x * uy + d.y * ux;
    return back({A.x + s * vx - c * vy, A.y + s * vy + c * vx});
  });
}

Pt target(const Target& t) {
  if (auto* p = std::get_if<TargetPoint>(&t)) return p->point;
  if (auto* c = std::get_if<TargetCross>(&t)) {
    auto xs = c->cross.crossings(c->with);
    Which w = c->which.value_or(Which{End::first});
    const PathCrossing* x = nullptr;
    if (auto* e = std::get_if<End>(&w)) {
      if (!xs.empty()) x = *e == End::first ? &xs.front() : &xs.back();
    } else {
      int i = std::get<int>(w);
      if (i >= 0 && i < int(xs.size())) x = &xs[i];
    }
    if (!x) {
      std::string name = std::holds_alternative<End>(w) ? (std::get<End>(w) == End::first ? "first" : "last")
                                                        : std::to_string(std::get<int>(w));
      throw std::runtime_error(std::string("the paths don't cross ") + (!xs.empty() ? name + " time" : "at all"));
    }
    return x->p;
  }
  auto& o = std::get<TargetOn>(t);
  return o.on.at(!o.segs ? o.at : o.on.within(*o.segs, o.at));
}

Path place(const Path& p, const std::vector<Pair>& pairs, const std::optional<Align>& align) {
  if (align && pairs.size() > 1)
    throw std::runtime_error("an alignment goes with one pair at most: two already decide the turn");
  Pt m = p.metric();
  auto to = [&](Pt q) { return toMetric(q, m); };
  auto back = [&](Pt q) { return fromMetric(q, m); };
  int n = int(pairs.size());
  std::vector<Progress> from;
  std::vector<Pt> goal;
  for (int k = 0; k < n; k++) from.push_back(pairs[k].from.value_or(defaultFrom(k, n)));
  for (auto& q : pairs) goal.push_back(target(q.target));
  Path out = p;
  if (n == 1 || n == 2) {
    // As attach above, which these reproduce exactly: a target is a mount on a path of one point.
    auto mount = [&](int k) { return Mount{Path(goal[k], {}, false, {}, m), 0, from[k], {}}; };
    out = attach(p, mount(0), n == 2 ? std::optional<Mount>(mount(1)) : std::nullopt);
  } else if (n >= 3) {
    std::vector<Pt> src, dst;
    for (double s : from) src.push_back(to(p.at(s)));
    for (Pt g : goal) dst.push_back(to(g));
    Map f = n == 3 ? exact(src, dst) : fitted(src, dst);
    // Points on a line decide no affine map (nothing says how far across to stretch): place by the outer two, then bend.
    out = f ? p.map([&](Pt q) { return back(f(to(q))); })
            : place(p, {Pair{TargetPoint{goal[0]}, from[0]}, Pair{TargetPoint{goal[n - 1]}, from[n - 1]}});
    if (!f || n > 3) out = bent(out, from, dst);
  }
  if (!align) return out;
  double s = align->from.value_or(!from.empty() ? from[0] : 0);
  Pt pivot = to(!goal.empty() ? goal[0] : out.at(s));
  Pt want = align->on.direction(!align->segs ? align->at : align->on.within(*align->segs, align->at));
  if (align->to == AlignTo::normal) want = {-want.y, want.x};
  double a = turn(out.direction(s), want) + rad(align->angle.value_or(0));
  return out.map([&](Pt q) {
    Pt d = to(q);
    Pt r = rotate({d.x - pivot.x, d.y - pivot.y}, a);
    return back({pivot.x + r.x, pivot.y + r.y});
  });
}

Path Path::attach(const Path& on, Progress at) const { return attach(on, at, AttachOptions{}); }

Path Path::attach(const Path& on, Progress at, const AttachOptions& o) const {
  std::optional<Mount> b;
  if (o.end) b = Mount{o.end->on.value_or(on), o.end->at, o.end->from, o.end->segs};
  return curves::attach(*this, Mount{on, at, o.from, o.segs}, b);
}

}  // namespace curves
