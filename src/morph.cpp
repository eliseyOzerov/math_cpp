#include "curves/morph.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

struct Aligned {
  std::vector<int> to;
  double cost;
};

/** The closest match, in order, of every node of `short` to a node of `long` (both ends to both ends), with its
 *  cost: the sum of the distances, in units of `metric`. Costs within `eps` count as equal, so the earlier wins. */
Aligned align(const std::vector<Pt>& shrt, const std::vector<Pt>& lng, Pt metric, double eps) {
  const int S = int(shrt.size()), L = int(lng.size());
  auto d = [&](int s, int l) { return dist(toMetric(shrt[s], metric), toMetric(lng[l], metric)); };
  // best[s][l]: the cheapest match of short's nodes 0…s with short[s] on long[l]; ends are pinned.
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> best(S, std::vector<double>(L, inf));
  std::vector<std::vector<int>> back(S, std::vector<int>(L, -1));
  best[0][0] = d(0, 0);
  for (int s = 1; s < S; s++) {
    for (int l = s; l <= L - S + s; l++) {
      if (s == S - 1 && l != L - 1) continue;
      for (int q = s - 1; q < l; q++) {
        double c = best[s - 1][q] + d(s, l);
        if (c < best[s][l] - eps) { best[s][l] = c; back[s][l] = q; }
      }
    }
  }
  std::vector<int> to(S);
  int l = L - 1;
  for (int s = S - 1; s >= 0; s--) {
    to[s] = l;
    l = l >= 0 ? back[s][l] : -1;
  }
  return {to, best[S - 1][L - 1]};
}

}  // namespace

Alike alike(const Path& a, const Path& b, double eps) {
  if (a.closed() != b.closed()) throw std::invalid_argument("a closed path morphs only into a closed one");
  if (a.segs().empty() || b.segs().empty()) throw std::invalid_argument("a path to morph needs a segment");
  const Pt m = a.metric();
  // Every way round `b` could run, and every start when closed; the closest fit wins.
  std::vector<Path> bs;
  for (const Path& p : {b, b.reverse()}) {
    if (a.closed())
      for (size_t r = 0; r < p.segs().size(); r++) bs.push_back(p.startAt(int(r)));
    else bs.push_back(p);
  }
  std::optional<Path> fitB;
  Aligned fit{{}, 0};
  for (const Path& p : bs) {
    auto an = a.nodes(), bn = p.nodes();
    bool aShort = an.size() <= bn.size();
    Aligned r = aShort ? align(an, bn, m, eps) : align(bn, an, m, eps);
    if (!fitB || r.cost < fit.cost - eps) { fitB = p; fit = r; }
  }
  const Path& t = *fitB;
  auto an = a.nodes(), bn = t.nodes();

  if (an.size() <= bn.size()) {
    // Too few nodes: add on `a` those of `b` it has none for, as far along each of `a`'s segments as they are along
    // `b`'s between the matched nodes, so `a` keeps its shape.
    const auto& to = fit.to;
    auto lb = t.distances();
    std::vector<Cubic> segs;
    std::vector<bool> matched{true};
    for (size_t i = 0; i < a.segs().size(); i++) {
      const Cubic& c = a.segs()[i];
      int j0 = to[i], j1 = to[i + 1];
      double span = lb[j1] - lb[j0];
      std::vector<double> shares;
      for (int k = 0; k < j1 - j0 - 1; k++)
        shares.push_back(span != 0 ? (lb[j0 + 1 + k] - lb[j0]) / span : double(k + 1) / (j1 - j0));
      for (auto& s : cutAt(c, shares, m)) segs.push_back(s);
      for (size_t k = 0; k < shares.size(); k++) matched.push_back(false);
      matched.push_back(true);
    }
    return {Path::of(segs, a.closed(), m), t, matched};
  }

  // Too many nodes: each of `a`'s extra nodes goes to the closer (along `a`) of the matched nodes either side, so on
  // the target it sits on that node, and the segment between them shrinks to nothing.
  const auto& of = fit.to;
  auto la = a.distances();
  std::vector<int> goes(an.size());
  std::vector<bool> matched(an.size(), false);
  for (size_t j = 0; j < of.size(); j++) { goes[of[j]] = int(j); matched[of[j]] = true; }
  for (size_t j = 0; j + 1 < of.size(); j++) {
    for (int i = of[j] + 1; i < of[j + 1]; i++) goes[i] = la[i] - la[of[j]] <= la[of[j + 1]] - la[i] ? int(j) : int(j + 1);
  }
  std::vector<Cubic> segs;
  for (size_t i = 0; i < a.segs().size(); i++) {
    int j = goes[i], k = goes[i + 1];
    segs.push_back(j == k ? Cubic{bn[j], bn[j], bn[j], bn[j]} : t.segs()[j]);
  }
  return {a, Path::of(segs, a.closed(), t.metric()), matched};
}

Path morph(const Path& a, const Path& b, double k, double eps) {
  Alike al = alike(a, b, eps);
  return between(al.from, al.to, k, eps);
}

Path between(const Path& from, const Path& to, double k, double eps) {
  const Pt m = from.metric();
  std::vector<Cubic> segs;
  for (size_t i = 0; i < from.segs().size(); i++) {
    Cubic c;
    for (int n = 0; n < 4; n++) c[n] = lerp(from.segs()[i][n], to.segs()[i][n], k);
    segs.push_back(c);
  }
  // A dropped segment's ends are within `eps`; the next one starts where the last kept one ended, so none come apart.
  std::vector<Cubic> out;
  for (const Cubic& c : segs) {
    bool some = false;
    for (Pt p : c)
      if (dist(toMetric(p, m), toMetric(c[0], m)) > eps) some = true;
    if (!some) continue;
    out.push_back(out.empty() ? c : Cubic{out.back()[3], c[1], c[2], c[3]});
  }
  if (out.empty()) out.push_back(segs[0]);
  else if (from.closed()) out.back()[3] = out[0][0];
  return Path::of(out, from.closed(), m);
}

}  // namespace curves
