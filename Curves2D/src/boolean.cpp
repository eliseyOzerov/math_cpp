// See include/curves/boolean.hpp. Ported from fountain-ts/src/curves/boolean.ts line for line.
#include "curves/boolean.hpp"

#include "js.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

/** Whether a piece is straight to within `flat` units: its handles that close to the line between its ends. */
bool straight(const Piece& q, double flat) {
  return q.line || (nearSegment(q.p[1], q.p[0], q.p[3], flat) && nearSegment(q.p[2], q.p[0], q.p[3], flat));
}
Piece ends(const Piece& q) { return Piece::of(q.first(), q.last()); }
int size(const Piece& q) { return q.line ? 2 : 4; }

/** How many box pairs crossings may halve before giving up: curves running along each other never stop splitting. */
constexpr int MAX_WORK = 20000;
/** How many raw hits crossings collects before giving up, for the same reason. */
constexpr size_t MAX_HITS = 400;
/** The most points two cubics can cross in (3 × 3, by Bézout); more means they run along each other. */
constexpr size_t MAX_CROSSINGS = 9;

/** A fragment of an input piece: which piece, the parameters it spans and its ends (snapped to the crossings), in
 *  its own direction: a reversed fragment (`rev`) starts at `a`, the end of its span on the piece. */
struct Frag {
  int g, c, i;
  double t0, t1;
  Pt a, b;
  bool rev = false;
};

/** An input piece: its group, contour and place in the contour, and whether it is a line. */
struct Entry {
  int g, c, i;
  Piece q;
  bool line;
};

using Groups = std::vector<std::vector<Contour>>;
using Src = std::function<const Piece&(const Frag&)>;

/** The piece a fragment stands for, in its direction. It is built along the source piece, from t0's end to t1's,
 *  and turned round if reversed (its ends are already swapped, so they are swapped back first). */
Piece fragPiece(const Frag& f, const Piece& src) {
  Pt p0 = f.rev ? f.b : f.a, p1 = f.rev ? f.a : f.b;
  Piece q;
  if (src.line) q = Piece::of(p0, p1);
  else { Cubic c = trim(asCubic(src), f.t0, f.t1); q = Piece::of(Cubic{p0, c[1], c[2], p1}); }
  return f.rev ? reverse(q) : q;
}

/** Where two straight cubics cross, as parameters; an empty list where they are parallel (the caller then cuts each
 *  where the other ends on it). */
std::vector<Crossing> lineHits(const Cubic& a, const Cubic& b, double eps) {
  Pt p = a[0], r = a[3] - a[0], q = b[0], s = b[3] - b[0];
  double d = cross(r, s);
  if (std::abs(d) < 1e-9 * len(r) * len(s)) return {};
  double t = cross(q - p, s) / d, u = cross(q - p, r) / d;
  double et = eps / len(r), eu = eps / len(s);
  if (t < -et || t > 1 + et || u < -eu || u > 1 + eu) return {};
  // Lines through each other's ends meet there; nothing to cut on the one that ends.
  return {{clamp(t, 0, 1), clamp(u, 0, 1)}};
}

/** The pieces cut into fragments wherever they cross each other (bounding-box subdivision). Pieces that run along
 *  each other are cut at each other's ends, so their fragments match and keep can drop the second copy. Fragments
 *  shorter than `eps` are left out. */
std::vector<Frag> cut(const std::vector<Entry>& all, double eps) {
  std::vector<Cubic> cubics;
  std::vector<Box> boxes;
  for (auto& e : all) { cubics.push_back(asCubic(e.q)); boxes.push_back(bounds(cubics.back())); }
  // Where each piece is cut: parameters and the points they are snapped to.
  struct Cut { double t; Pt p; };
  std::vector<std::vector<Cut>> cuts(all.size());
  auto at = [&](size_t k, double t, Pt p) { if (t > 1e-6 && t < 1 - 1e-6) cuts[k].push_back({t, p}); };
  for (size_t j = 0; j < all.size(); j++) for (size_t k = j + 1; k < all.size(); k++) {
    if (!overlaps(boxes[j], boxes[k], eps)) continue;
    const Entry &A = all[j], &B = all[k];
    const Cubic &a = cubics[j], &b = cubics[k];
    std::vector<SharedEnd> shared;
    for (int ea : {0, 1}) for (int eb : {0, 1}) if (dist(a[ea * 3], b[eb * 3]) < eps) shared.push_back({ea, eb});
    bool lines = A.line && B.line;
    std::optional<std::vector<Crossing>> hits = lines ? lineHits(a, b, eps) : crossings(a, b, 1e-3, shared);
    if (!hits || hits->empty() || lines) {
      // Along each other: each is cut where the other ends on it.
      for (auto [x, y, kx] : {std::tuple{&a, &b, j}, std::tuple{&b, &a, k}})
        for (Pt e : {(*y)[0], (*y)[3]}) {
          Nearest q = nearest(std::span(x, 1), e);
          if (q.d < eps) at(kx, q.t, e);
        }
      if (!hits) continue;
    }
    for (auto [t, u] : *hits) {
      Pt p = lerp(bez(a, t), bez(b, u), 0.5);
      at(j, t, p); at(k, u, p);
    }
  }
  std::vector<Frag> frags;
  for (size_t k = 0; k < all.size(); k++) {
    const Entry& e = all[k];
    auto& sorted = cuts[k];
    std::stable_sort(sorted.begin(), sorted.end(), [](const Cut& x, const Cut& y) { return x.t < y.t; });
    std::vector<Cut> stops{{0, e.q.first()}};
    for (size_t i = 0; i < sorted.size(); i++)
      if (i == 0 || sorted[i].t - sorted[i - 1].t > 1e-7) stops.push_back(sorted[i]);
    stops.push_back({1, e.q.last()});
    for (size_t i = 0; i + 1 < stops.size(); i++) {
      const Cut &u = stops[i], &v = stops[i + 1];
      // A fragment shorter than eps is a crossing seen twice, or a sliver between ends that nearly meet.
      if (dist(u.p, v.p) > eps || (!e.line && arcLength(trim(asCubic(e.q), u.t, v.t)) > eps))
        frags.push_back({e.g, e.c, e.i, u.t, v.t, u.p, v.p});
    }
  }
  return frags;
}

/** The fragments where the rule changes across them, the winding of each group counted at a point just to the left
 *  and just to the right; each turned so the inside is on its right. A copy of an edge already kept is dropped. */
std::vector<Frag> keep(const std::vector<Frag>& frags, const Groups& groups, const Src& src, const Rule& inside,
                       double eps) {
  std::vector<Frag> kept;
  auto windings = [&](Pt p) {
    std::vector<int> w;
    for (auto& cs : groups) w.push_back(winding(cs, p));
    return w;
  };
  for (const Frag& f : frags) {
    // Probed on the piece itself, not its snapped ends: an end snapped to a crossing up to eps away tilts a short
    // fragment enough to put both probes on one side.
    Cubic c = trim(asCubic(src(f)), f.t0, f.t1);
    Pt m = bez(c, 0.5), d = norm(bezD(c, 0.5));
    double length = arcLength(c), h = js::min(0.05, length / 8);
    if (length < 1e-7) continue;
    Pt left{m.x - d.y * h, m.y + d.x * h}, right{m.x + d.y * h, m.y - d.x * h};
    bool L = inside(windings(left)), R = inside(windings(right));
    if (L == R) continue;
    Frag g = f;
    if (L) { g.rev = true; g.a = f.b; g.b = f.a; }
    Pt mid = bez(asCubic(fragPiece(g, src(g))), 0.5);
    bool dup = false;
    for (const Frag& k : kept)
      if (dist(k.a, g.a) < eps && dist(k.b, g.b) < eps && dist(bez(asCubic(fragPiece(k, src(k))), 0.5), mid) < eps) {
        dup = true; break;
      }
    if (dup) continue;
    kept.push_back(g);
  }
  return kept;
}

/** A loop of fragments as a contour: neighbours cut from the same piece are put back together, and each piece starts
 *  exactly where the one before ends. */
std::pair<Contour, std::vector<int>> rejoin(const std::vector<Frag>& loop, const Src& src) {
  std::vector<Frag> merged;
  for (const Frag& f : loop) {
    if (!merged.empty()) {
      Frag& p = merged.back();
      if (p.g == f.g && p.c == f.c && p.i == f.i && p.rev == f.rev &&
          std::abs((p.rev ? p.t0 : p.t1) - (f.rev ? f.t1 : f.t0)) < 1e-9) {
        if (p.rev) p.t0 = f.t0; else p.t1 = f.t1;
        p.b = f.b;
        continue;
      }
    }
    merged.push_back(f);
  }
  // Lines from different pieces that run on in one direction become one line. The joined line counts as the first
  // piece's, as it ran first.
  Contour pieces;
  std::vector<int> from;
  for (const Frag& f : merged) {
    Piece q = fragPiece(f, src(f));
    if (!pieces.empty() && pieces.back().line && q.line && nearSegment(pieces.back().p[1], pieces.back().p[0], q.p[1], 0.01, 0))
      pieces.back() = Piece::of(pieces.back().p[0], q.p[1]);
    else { pieces.push_back(q); from.push_back(f.g); }
  }
  if (pieces.size() > 2) {
    Piece p = pieces.back(), q = pieces[0];
    if (p.line && q.line && nearSegment(p.p[1], p.p[0], q.p[1], 0.01, 0)) {
      pieces.pop_back(); pieces[0] = Piece::of(p.p[0], q.p[1]);
      from[0] = from.back(); from.pop_back();
    }
  }
  // A piece that stays within a unit of its start is dropped and its neighbours joined: it would not show, but it
  // reads as two corners, pits where the outline runs on smoothly.
  for (int i = int(pieces.size()) - 1; i >= 0 && pieces.size() > 2; i--) {
    const Piece& q = pieces[i];
    bool small = true;
    for (int k = 0; k < size(q); k++) if (!(dist(q.p[k], q.p[0]) < 1)) small = false;
    if (small) { pieces.erase(pieces.begin() + i); from.erase(from.begin() + i); }
  }
  stitch(pieces);
  return {std::move(pieces), std::move(from)};
}

/** The kept fragments chained end to start into loops, or nothing where a loop does not close. A loop thinner than
 *  eps (twice its area over its length) is a sliver where two edges graze: it is left out rather than failing the
 *  whole outline. */
std::optional<Traced> chain(const std::vector<Frag>& kept, const Src& src, double eps) {
  std::vector<bool> used(kept.size(), false);
  Traced out;
  auto tiny = [&](const std::vector<Frag>& loop) {
    std::vector<Pt> ps;
    for (const Frag& f : loop) {
      Cubic c = asCubic(fragPiece(f, src(f)));
      for (double t : {0.0, 0.25, 0.5, 0.75}) ps.push_back(bez(c, t));
    }
    double length = 0;
    for (size_t i = 0; i < ps.size(); i++) length += dist(ps[i], ps[(i + 1) % ps.size()]);
    return 2 * std::abs(area(ps)) / js::max(length, 1e-9) < eps;
  };
  for (size_t s = 0; s < kept.size(); s++) {
    if (used[s]) continue;
    std::vector<Frag> loop{kept[s]};
    used[s] = true;
    bool open = false;
    for (size_t guard = 0; guard <= kept.size(); guard++) {
      Pt end = loop.back().b;
      if (loop.size() > 1 && dist(end, loop[0].a) < eps) break;
      int best = -1;
      double bd = eps;
      for (size_t k = 0; k < kept.size(); k++)
        if (!used[k] && dist(kept[k].a, end) < bd) { best = int(k); bd = dist(kept[k].a, end); }
      if (best < 0) {
        if (dist(end, loop[0].a) < eps) break;
        open = true; break;
      }
      used[best] = true; loop.push_back(kept[best]);
    }
    if (open || dist(loop.back().b, loop[0].a) >= eps) {
      if (tiny(loop)) continue;
      return std::nullopt;
    }
    if (tiny(loop)) continue;
    auto [pieces, from] = rejoin(loop, src);
    out.contours.push_back(std::move(pieces)); out.from.push_back(std::move(from));
  }
  return out;
}

}  // namespace

std::optional<std::vector<Crossing>> crossings(const Cubic& a, const Cubic& b, double tol,
                                               std::span<const SharedEnd> shared) {
  std::vector<Crossing> hits;
  int work = 0;
  auto near = [](double t0, double t1, int e) { return (e == 0 ? t1 : 1 - t0) < 1e-4; };
  auto width = [](const Box& x) { return js::max(x[1].x - x[0].x, x[1].y - x[0].y); };
  std::function<bool(const Cubic&, double, double, const Cubic&, double, double)> go =
      [&](const Cubic& ca, double t0, double t1, const Cubic& cb, double u0, double u1) -> bool {
    if (++work > MAX_WORK) return false;
    Box ba = bounds(ca), bb = bounds(cb);
    if (!overlaps(ba, bb)) return true;
    for (auto [ea, eb] : shared) if (near(t0, t1, ea) && near(u0, u1, eb)) return true;
    if (width(ba) < tol && width(bb) < tol) {
      hits.push_back({(t0 + t1) / 2, (u0 + u1) / 2});
      return hits.size() < MAX_HITS;
    }
    double tm = (t0 + t1) / 2, um = (u0 + u1) / 2;
    auto [a0, a1] = split(ca, 0.5);
    auto [b0, b1] = split(cb, 0.5);
    return go(a0, t0, tm, b0, u0, um) && go(a0, t0, tm, b1, um, u1) && go(a1, tm, t1, b0, u0, um) &&
           go(a1, tm, t1, b1, um, u1);
  };
  if (!go(a, 0, 1, b, 0, 1)) return std::nullopt;
  // One crossing leaves a cluster of hits a few tolerances wide.
  std::vector<Crossing> out;
  for (const Crossing& h : hits) {
    Pt p = bez(a, h[0]);
    bool seen = false;
    for (const Crossing& o : out) if (dist(bez(a, o[0]), p) < 20 * tol) { seen = true; break; }
    if (!seen) out.push_back(h);
  }
  if (out.size() > MAX_CROSSINGS) return std::nullopt;
  return out;
}

int winding(std::span<const Contour> cs, Pt p) {
  double x = p.x, y = p.y + 1.2345e-7;
  int w = 0;
  for (const Contour& c : cs) for (const Piece& q : c) {
    if (q.line) {
      Pt a = q.p[0], b = q.p[1];
      if ((a.y > y) != (b.y > y)) {
        double xi = a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x);
        if (xi > x) w += b.y > a.y ? 1 : -1;
      }
      continue;
    }
    std::array<double, 4> ys{q.p[0].y, q.p[1].y, q.p[2].y, q.p[3].y}, xs{q.p[0].x, q.p[1].x, q.p[2].x, q.p[3].x};
    if (y < js::minOf(ys) || y > js::maxOf(ys) || x > js::maxOf(xs)) continue;
    for (double t : roots(q.p, 1, y)) {
      if (bez(q.p, t).x <= x) continue;
      double dy = bezD(q.p, t).y;
      if (std::abs(dy) > 1e-12) w += dy > 0 ? 1 : -1;
    }
  }
  return w;
}

std::vector<Contour> boolean(std::span<const std::vector<Contour>> input, const Rule& inside, double eps) {
  if (auto r = tryBoolean(input, inside, eps)) return *r;
  std::vector<Contour> flat;
  for (auto& cs : input) flat.insert(flat.end(), cs.begin(), cs.end());
  return flat;
}

std::optional<std::vector<Contour>> tryBoolean(std::span<const std::vector<Contour>> input, const Rule& inside,
                                               double eps) {
  if (auto t = traceBoolean(input, inside, eps)) return std::move(t->contours);
  return std::nullopt;
}

std::optional<Traced> traceBoolean(std::span<const std::vector<Contour>> input, const Rule& inside, double eps) {
  // Nearly straight pieces become lines, here and in the winding counts alike.
  Groups groups;
  for (auto& cs : input) {
    auto& g = groups.emplace_back();
    for (auto& c : cs) {
      auto& gc = g.emplace_back();
      for (auto& q : c) gc.push_back(straight(q, 2 * eps) ? ends(q) : q);
    }
  }
  std::vector<Entry> all;
  for (size_t g = 0; g < groups.size(); g++)
    for (size_t ci = 0; ci < groups[g].size(); ci++)
      for (size_t i = 0; i < groups[g][ci].size(); i++) {
        const Piece& q = groups[g][ci][i];
        if (dist(q.first(), q.last()) > 1e-9 || !q.line) all.push_back({int(g), int(ci), int(i), q, q.line});
      }
  Src src = [&](const Frag& f) -> const Piece& { return groups[f.g][f.c][f.i]; };
  return chain(keep(cut(all, eps), groups, src, inside, eps), src, eps);
}

std::vector<Contour> unite(std::span<const Contour> cs) {
  std::vector<std::vector<Contour>> input{{cs.begin(), cs.end()}};
  return boolean(input, [](const std::vector<int>& w) { return w[0] != 0; });
}

double contourArea(const Contour& c) {
  std::vector<Pt> ps;
  for (const Piece& q : c) {
    ps.push_back(q.p[0]);
    if (!q.line) { ps.push_back(q.p[1]); ps.push_back(q.p[2]); }
  }
  return area(ps);
}

Contour reverseContour(const Contour& c) {
  Contour out;
  for (auto q = c.rbegin(); q != c.rend(); ++q) out.push_back(reverse(*q));
  return out;
}

std::vector<Contour> orient(std::span<const Contour> cs) {
  std::vector<double> areas;
  for (auto& c : cs) areas.push_back(contourArea(c));
  size_t outer = 0;
  for (size_t i = 0; i < areas.size(); i++) if (std::abs(areas[i]) > std::abs(areas[outer])) outer = i;
  std::vector<Contour> out;
  for (size_t i = 0; i < cs.size(); i++) out.push_back((areas[i] < 0) == (i == outer) ? cs[i] : reverseContour(cs[i]));
  return out;
}

std::vector<std::vector<Pt>> flatten(std::span<const Contour> cs, double step, double min) {
  std::vector<std::vector<Pt>> out;
  for (const Contour& c : cs) {
    auto& poly = out.emplace_back();
    for (const Piece& q : c) {
      if (q.line) { poly.push_back(q.p[0]); continue; }
      double steps = js::max(min, std::ceil(hullLength(q.p) / step));
      for (int i = 0; i < steps; i++) poly.push_back(bez(q.p, i / steps));
    }
  }
  return out;
}

std::vector<Edge> edges(std::span<const std::vector<Pt>> polys) {
  std::vector<Edge> out;
  for (auto& p : polys)
    for (size_t i = 0; i < p.size(); i++) {
      Pt a = p[i], b = p[(i + 1) % p.size()];
      if (a.y != b.y) out.push_back({a, b});
    }
  return out;
}

std::vector<std::array<double, 2>> scanline(std::span<const Edge> es, double y) {
  std::vector<std::array<double, 2>> xs;
  for (auto& [a, b] : es) {
    if ((a.y <= y) == (b.y <= y)) continue;
    xs.push_back({a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y), b.y > a.y ? 1.0 : -1.0});
  }
  return xs;
}

std::optional<Gap> narrowest(std::span<const Pt> poly, double apart) {
  size_t n = poly.size();
  std::vector<double> s{0};
  for (size_t i = 1; i <= n; i++) s.push_back(s[i - 1] + dist(poly[i - 1], poly[i % n]));
  double total = s[n];
  std::optional<Gap> best;
  for (size_t i = 0; i < n; i++) for (size_t j = i + 1; j < n; j++) {
    double along = s[j] - s[i];
    if (js::min(along, total - along) < apart) continue;
    double d = dist(poly[i], poly[j]);
    if (!best || d < best->d) best = Gap{d, poly[i], poly[j]};
  }
  return best;
}

Gap polyGap(std::span<const Pt> p, std::span<const Pt> q) {
  Gap best{std::numeric_limits<double>::infinity(), p[0], q[0]};
  for (Pt a : p) for (Pt b : q) { double d = dist(a, b); if (d < best.d) best = {d, a, b}; }
  return best;
}

}  // namespace curves
