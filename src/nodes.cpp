#include "curves/nodes.hpp"

#include "curves/bezier.hpp"
#include "curves/fit.hpp"
#include "js.hpp"
#include "jsmath.hpp"

#include <cmath>
#include <stdexcept>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** A handle closer to its anchor than this is on it. */
constexpr double ON = 1e-9;
/** How far a handle on its anchor is nudged towards the next control point. */
constexpr double NUDGE = 1e-3;

bool zero(const std::optional<Pt>& v) { return !v || len(*v) < ON; }

/** The cubic with its handles on their anchors nudged off them. */
Cubic nudged(const Cubic& c) {
  Pt a = c[0], c1 = c[1], c2 = c[2], b = c[3];
  bool z1 = dist(a, c1) < ON, z2 = dist(b, c2) < ON;
  if (z1 && z2) return {a, lerp(a, b, 1.0 / 3), lerp(a, b, 2.0 / 3), b};
  return {a, z1 ? lerp(a, c2, NUDGE) : c1, z2 ? lerp(b, c1, NUDGE) : c2, b};
}

HandleSide other(HandleSide side) { return side == HandleSide::in ? HandleSide::out : HandleSide::in; }

std::optional<Pt>& handleOf(Node& n, HandleSide side) { return side == HandleSide::in ? n.in : n.out; }
const std::optional<Pt>& handleOf(const Node& n, HandleSide side) { return side == HandleSide::in ? n.in : n.out; }

bool same(const HandleRef& a, const HandleRef& b) { return a.s == b.s && a.i == b.i && a.side == b.side; }

/** The handle `r` in `lists`, if the lists have that node and it has the handle. */
std::optional<Pt> handleAt(const std::vector<std::vector<Node>>& lists, const HandleRef& r) {
  if (r.s < 0 || r.s >= int(lists.size())) return std::nullopt;
  auto& ns = lists[r.s];
  if (r.i < 0 || r.i >= int(ns.size())) return std::nullopt;
  return handleOf(ns[r.i], r.side);
}

constexpr double TURN = 8;
constexpr int SAMPLES = 12;

}  // namespace

std::vector<Node> nodesOf(const Path& path) {
  const auto& segs = path.segs();
  int n = path.closed() ? int(segs.size()) : int(segs.size()) + 1;
  std::vector<Node> r;
  for (int i = 0; i < n; i++) {
    Pt p = path.node(i);
    int inSeg = i > 0 ? i - 1 : path.closed() ? int(segs.size()) - 1 : -1, outSeg = i < int(segs.size()) ? i : -1;
    Node node{p, std::nullopt, std::nullopt, path.kind(i)};
    if (inSeg >= 0) node.in = segs[inSeg][2] - p;
    if (outSeg >= 0) node.out = segs[outSeg][1] - p;
    r.push_back(node);
  }
  return r;
}

Cubic segmentOf(const Node& a, const Node& b) {
  return {a.p, a.p + a.out.value_or(Pt{0, 0}), b.p + b.in.value_or(Pt{0, 0}), b.p};
}

std::vector<Cubic> segmentsOf(const std::vector<Node>& nodes, bool closed) {
  std::vector<Cubic> r;
  for (size_t i = 0; i < nodes.size(); i++) {
    if (i < nodes.size() - 1) r.push_back(segmentOf(nodes[i], nodes[i + 1]));
    else if (closed && nodes.size() > 1) r.push_back(segmentOf(nodes[i], nodes[0]));
  }
  return r;
}

Path pathOf(const std::vector<Node>& nodes, bool closed, Pt metric) {
  std::vector<Cubic> segs;
  for (auto& c : segmentsOf(nodes, closed)) segs.push_back(nudged(c));
  if (segs.empty()) throw std::invalid_argument("a path needs two nodes");
  std::vector<std::optional<NodeKind>> kinds;
  for (auto& n : nodes) kinds.push_back(n.kind);
  return Path(segs[0][0], segs, closed, std::nullopt, metric, kinds);
}

Pt pointOn(const std::vector<Node>& nodes, bool closed, int seg, double t) {
  return bez(nudged(segmentsOf(nodes, closed).at(seg)), t);
}

NodesNearest nearestOn(const std::vector<Node>& nodes, bool closed, Pt p) {
  std::vector<Cubic> segs;
  for (auto& c : segmentsOf(nodes, closed)) segs.push_back(nudged(c));
  Nearest n = nearest(segs, p, 64);
  return {n.i, n.t, n.p, n.d};
}

std::vector<NodesCrossing> crossingsOn(const std::vector<Node>& nodes, bool closed, int j, double v) {
  std::vector<NodesCrossing> r;
  int seg = 0;
  for (auto& raw : segmentsOf(nodes, closed)) {
    Cubic c = nudged(raw);
    for (double t : coordRoots(c[0][j], c[1][j], c[2][j], c[3][j], v)) r.push_back({seg, t, bez(c, t)});
    seg++;
  }
  return r;
}

std::vector<Node> moldNodes(const std::vector<Node>& nodes, bool closed, int seg, double t, Pt to) {
  int j = (seg + 1) % int(nodes.size());
  Cubic c = mold(nudged(segmentsOf(nodes, closed).at(seg)), t, to);
  std::vector<Node> out = nodes;
  out.at(seg) = withHandle(out[seg], HandleSide::out, c[1] - c[0]);
  out.at(j) = withHandle(out[j], HandleSide::in, c[2] - c[3]);
  return out;
}

NodeKind nodeKind(const Node& n) {
  if (n.kind) return *n.kind;
  if (zero(n.in) || zero(n.out)) return NodeKind::corner;
  Pt a = norm(*n.in), b = norm(*n.out);
  if (a[0] * b[0] + a[1] * b[1] > -0.9999) return NodeKind::corner;
  return std::abs(len(*n.in) - len(*n.out)) < 1e-6 * js::max(1, len(*n.in)) ? NodeKind::symmetric : NodeKind::smooth;
}

Node withHandle(const Node& n, HandleSide side, Pt to) {
  NodeKind kind = nodeKind(n);
  HandleSide o = other(side);
  Node out = n;
  handleOf(out, side) = to;
  if (!handleOf(n, o) || kind == NodeKind::corner) return out;
  if (kind == NodeKind::symmetric) {
    Node r = out;
    handleOf(r, o) = to * -1.0;
    return r;
  }
  if (zero(to) || zero(handleOf(n, o))) return out;
  Node r = out;
  handleOf(r, o) = norm(to) * -len(*handleOf(n, o));
  return r;
}

Node withNodeKind(const Node& n, NodeKind kind) {
  if (kind == NodeKind::corner || !n.in || !n.out) {
    Node r = n;
    r.kind = kind;
    return r;
  }
  double li = len(*n.in), lo = len(*n.out);
  if (li < ON && lo < ON) {
    Node r = n;
    r.kind = kind;
    return r;
  }
  Pt d = li < ON ? norm(*n.out) : lo < ON ? norm(*n.in) * -1.0 : norm(norm(*n.out) - norm(*n.in));
  double m = (li + lo) / 2, a = kind == NodeKind::symmetric ? m : li, b = kind == NodeKind::symmetric ? m : lo;
  Node r = n;
  r.kind = kind;
  r.in = d * -a;
  r.out = d * b;
  return r;
}

std::vector<Node> insertNode(const std::vector<Node>& nodes, int seg, double t) {
  int n = int(nodes.size()), j = (seg + 1) % n;
  const Node& a = nodes.at(seg);
  const Node& b = nodes.at(j);
  auto lr = split(segmentOf(a, b), t);
  const Cubic &l = lr[0], &r = lr[1];
  Pt p = l[3];
  Node mid{p, l[2] - p, r[1] - p, NodeKind::smooth};
  std::vector<Node> out = nodes;
  out[seg].out = l[1] - a.p;
  out[j].in = r[2] - b.p;
  out.insert(out.begin() + seg + 1, mid);
  return out;
}

std::vector<Node> shiftNodes(const std::vector<Node>& nodes, Pt d, const std::function<bool(int)>& pick) {
  std::vector<Node> r = nodes;
  for (size_t i = 0; i < r.size(); i++)
    if (!pick || pick(int(i))) r[i].p = r[i].p + d;
  return r;
}

std::vector<Node> simplified(const std::vector<Node>& nodes, bool closed, double tol) {
  std::vector<Cubic> segs;
  for (auto& c : segmentsOf(nodes, closed)) segs.push_back(nudged(c));
  int n = int(nodes.size());
  if (segs.size() < 2) return nodes;
  auto dir = [](const Cubic& c, double t) { return norm(bezD(c, t)); };
  auto isCorner = [&](int i) {
    const Cubic* a = i > 0 ? &segs[i - 1] : closed ? &segs[segs.size() - 1] : nullptr;
    const Cubic* b = i < int(segs.size()) ? &segs[i] : nullptr;
    if (!a || !b) return true;
    Pt u = dir(*a, 1), v = dir(*b, 0);
    return std::acos(js::max(-1, js::min(1, u[0] * v[0] + u[1] * v[1]))) > TURN * PI / 180;
  };
  std::vector<int> cuts;
  for (int i = 0; i < n; i++)
    if (isCorner(i)) cuts.push_back(i);
  bool seam = cuts.empty();
  if (seam) cuts.push_back(0);
  if (closed) cuts.push_back(cuts[0] + n);
  else if (cuts.back() != n - 1) cuts.push_back(n - 1);
  std::vector<Cubic> cubics;
  std::vector<bool> starts;
  for (size_t k = 0; k + 1 < cuts.size(); k++) {
    std::vector<Pt> pts;
    for (int i = cuts[k]; i < cuts[k + 1]; i++) {
      const Cubic& c = segs[i % segs.size()];
      for (int j = i == cuts[k] ? 0 : 1; j <= SAMPLES; j++) pts.push_back(bez(c, double(j) / SAMPLES));
    }
    FitOptions fo;
    fo.corner = 180;
    auto fit = fitCubics(pts, tol, fo);
    for (size_t j = 0; j < fit.size(); j++) {
      cubics.push_back(fit[j]);
      starts.push_back(j == 0);
    }
  }
  auto straight = [&](const Cubic& c) {
    for (Pt q : {c[1], c[2]})
      if (!(std::abs(cross(c[3] - c[0], q - c[0])) / js::max(dist(c[0], c[3]), 1e-9) < tol / 4)) return false;
    return true;
  };
  for (auto& c : cubics)
    if (straight(c)) c = {c[0], c[0], c[3], c[3]};
  std::vector<Node> out;
  for (size_t k = 0; k < cubics.size(); k++) {
    auto& c = cubics[k];
    out.push_back({c[0], std::nullopt, c[1] - c[0], starts[k] && !(seam && k == 0) ? NodeKind::corner : NodeKind::smooth});
  }
  for (size_t k = 0; k < cubics.size(); k++) {
    auto& c = cubics[k];
    size_t j = k + 1;
    if (j < out.size()) out[j].in = c[2] - c[3];
    else if (closed) out[0].in = c[2] - c[3];
  }
  if (!closed) {
    const Cubic& last = cubics.at(cubics.size() - 1);
    out.push_back({last[3], last[2] - last[3], std::nullopt, NodeKind::corner});
  }
  if (seam) out[0] = withNodeKind(out[0], NodeKind::smooth);
  return out;
}

// ---- Handle locks ----

const HandleLock* lockOf(const std::vector<HandleLock>& locks, const HandleRef& r) {
  for (auto& l : locks)
    for (auto& h : l.handles)
      if (same(h, r)) return &l;
  return nullptr;
}

std::vector<std::vector<Node>> withLockedHandle(const std::vector<std::vector<Node>>& lists,
                                                const std::vector<HandleLock>& locks, const HandleRef& ref, Pt to) {
  std::vector<std::vector<Node>> out = lists;
  auto set = [&](const HandleRef& r, Pt v) {
    if (r.s < 0 || r.s >= int(out.size()) || r.i < 0 || r.i >= int(out[r.s].size())) return;
    handleOf(out[r.s][r.i], r.side) = v;
  };
  set(ref, to);
  const HandleLock* lock = lockOf(locks, ref);
  std::optional<Pt> from = handleAt(lists, ref);
  if (!lock || zero(from)) return out;
  const LockHandle* me = nullptr;
  for (auto& h : lock->handles)
    if (same(h, ref)) {
      me = &h;
      break;
    }
  // Math.atan2, sin and cos as V8 has them (jsmath.hpp), so the turned handles agree with the TS to the bit.
  double by = lock->angle ? js::atan2(cross(*from, to), dot(*from, to)) : 0;
  double k = lock->length && !zero(to) ? len(to) / len(*from) : 1;
  for (auto& h : lock->handles) {
    std::optional<Pt> v = handleAt(lists, h);
    if (!same(h, ref) && !zero(v)) set(h, js::rotate(*v, !h.flip == !me->flip ? by : -by) * k);
  }
  return out;
}

std::vector<HandleLock> locked(const std::vector<HandleLock>& locks, const std::vector<HandleRef>& refs, LockHow how) {
  std::vector<bool> isHit;
  std::vector<LockHandle> handles;
  for (auto& l : locks) {
    bool h = false;
    for (auto& x : l.handles)
      for (auto& r : refs)
        if (same(x, r)) h = true;
    isHit.push_back(h);
    if (h) handles.insert(handles.end(), l.handles.begin(), l.handles.end());
  }
  for (auto& r : refs) {
    bool found = false;
    for (auto& h : handles)
      if (same(h, r)) found = true;
    if (!found) handles.push_back(LockHandle(r.s, r.i, r.side));
  }
  std::vector<HandleLock> out;
  for (size_t i = 0; i < locks.size(); i++)
    if (!isHit[i]) out.push_back(locks[i]);
  HandleLock l;
  l.handles = handles;
  l.angle = how.angle;
  l.length = how.length;
  out.push_back(l);
  return out;
}

std::vector<HandleLock> unlocked(const std::vector<HandleLock>& locks, const std::vector<HandleRef>& refs) {
  std::vector<HandleLock> out;
  for (auto& l : locks) {
    HandleLock m = l;
    m.handles.clear();
    for (auto& h : l.handles) {
      bool drop = false;
      for (auto& r : refs)
        if (same(h, r)) drop = true;
      if (!drop) m.handles.push_back(h);
    }
    if (m.handles.size() > 1) out.push_back(m);
  }
  return out;
}

std::vector<HandleLock> remapLocks(const std::vector<HandleLock>& locks, int s,
                                   const std::function<std::optional<int>(int)>& to) {
  std::vector<HandleLock> out;
  for (auto& l : locks) {
    HandleLock m = l;
    m.handles.clear();
    for (auto& h : l.handles) {
      if (h.s != s) {
        m.handles.push_back(h);
        continue;
      }
      auto i = to(h.i);
      if (!i) continue;
      LockHandle g = h;
      g.i = *i;
      m.handles.push_back(g);
    }
    if (m.handles.size() > 1) out.push_back(m);
  }
  return out;
}

NodeKind lockedKind(const std::vector<HandleLock>& locks, const Node& n, int s, int i) {
  const HandleLock* l = lockOf(locks, {s, i, HandleSide::in});
  if (!l || !l->angle || l != lockOf(locks, {s, i, HandleSide::out})) return NodeKind::corner;
  return l->length && n.in && n.out && std::abs(len(*n.in) - len(*n.out)) < 1e-6 * js::max(1, len(*n.in))
             ? NodeKind::symmetric
             : NodeKind::smooth;
}

std::vector<std::vector<Node>> lockedKinds(const std::vector<std::vector<Node>>& lists, const std::vector<HandleLock>& locks) {
  std::vector<std::vector<Node>> out;
  for (size_t s = 0; s < lists.size(); s++) {
    std::vector<Node> ns;
    for (size_t i = 0; i < lists[s].size(); i++) {
      Node n = lists[s][i];
      n.kind = lockedKind(locks, lists[s][i], int(s), int(i));
      ns.push_back(n);
    }
    out.push_back(ns);
  }
  return out;
}

KindLocks kindLocks(const std::vector<std::vector<Node>>& lists) {
  KindLocks r;
  for (size_t s = 0; s < lists.size(); s++) {
    std::vector<Node> ns;
    for (size_t i = 0; i < lists[s].size(); i++) {
      Node n = lists[s][i];
      auto kind = n.kind;
      n.kind.reset();
      // Handles on their anchor have no angle to keep, so they are left out.
      if (!zero(n.in) && !zero(n.out) && (kind == NodeKind::smooth || kind == NodeKind::symmetric)) {
        HandleLock l;
        l.handles = {LockHandle(int(s), int(i), HandleSide::in), LockHandle(int(s), int(i), HandleSide::out)};
        l.angle = true;
        l.length = kind == NodeKind::symmetric;
        r.locks.push_back(l);
      }
      ns.push_back(n);
    }
    r.lists.push_back(ns);
  }
  return r;
}

}  // namespace curves
