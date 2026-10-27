// The golden cases of warp.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
// Warps are functions, so the cases name them by a spec (see tests/golden/cases/warp.ts).
#include "cases.hpp"

#include "curves/warp.hpp"

namespace {

using namespace golden;

std::vector<Pt> pointsOf(const Value& v) { return list(v, pt); }

Wave waveOf(const Value& s) {
  Swing o;
  if (s.has("cycles")) o.cycles = s["cycles"].num();
  if (s.has("phase")) o.phase = s["phase"].num();
  if (s.has("low")) o.low = s["low"].num();
  if (s.has("high")) o.high = s["high"].num();
  return s["w"].str() == "saw" ? saw(o) : sine(o);
}

Wobble wobbleOf(const Value& v) {
  Wobble w;
  w.amplitude = v["amplitude"].num();
  if (v.has("frequency")) w.frequency = v["frequency"].num();
  if (v.has("octaves")) w.octaves = v["octaves"].num();
  if (v.has("seed")) w.seed = v["seed"].num();
  return w;
}

Warp warpOf(const Value& s) {
  const std::string& k = s["k"].str();
  if (k == "affine") {
    Value mm = s["m"];
    double a = mm[0].num(), b = mm[1].num(), c = mm[2].num(), d = mm[3].num(), e = mm[4].num(), f = mm[5].num();
    return [=](Pt p) -> Pt { return {a * p[0] + c * p[1] + e, b * p[0] + d * p[1] + f}; };
  }
  if (k == "quad") return [](Pt p) -> Pt { return {p[0] + 0.004 * p[1] * p[1] - 0.001 * p[0] * p[1], p[1] + 0.002 * p[0] * p[0]}; };
  if (k == "sin") return [](Pt p) -> Pt { return {p[0] + 8 * std::sin(p[1] / 20), p[1] + 3 * std::cos(p[0] / 30)}; };
  if (k == "spine") {
    Value dx = s["dx"];
    std::variant<double, Wave> d = std::holds_alternative<double>(dx.v) ? std::variant<double, Wave>(dx.num()) : std::variant<double, Wave>(waveOf(dx));
    return spine(d, s["y0"].num(), s["y1"].num());
  }
  if (k == "mesh") return mesh(pt(s["lo"]), pt(s["hi"]), pointsOf(s["grid"]));
  if (k == "wobble") return wobble(wobbleOf(s["w"]));
  std::vector<double> ys;
  for (auto& y : s["ys"].arr()) ys.push_back(y.num());
  return pinned(warpOf(s["f"]), ys, s["reach"].num());
}

WarpOptions optionsOf(const Value& a, size_t i) {
  WarpOptions o;
  if (i < a.size() && a[i].isObject()) {
    if (a[i].has("pieces")) o.pieces = a[i]["pieces"].num();
    if (a[i].has("step")) o.step = a[i]["step"].num();
    if (a[i].has("eps")) o.eps = a[i]["eps"].num();
  }
  return o;
}
double epsOf(const Value& a, size_t i) { return opt(a, i).value_or(1e-3); }

struct Node {
  Pt p;
  std::optional<Pt> in, out;
};

Value outCubics(const std::vector<Cubic>& cs) { return out(cs); }

golden::Register reg("warp.json", {
  {"jacobianAt", [](auto& a) { return out(jacobian(warpOf(a[0]), pt(a[1]), epsOf(a, 2))); }},
  {"warpNodeAt", [](auto& a) {
     Node n{pt(a[0]["p"]), std::nullopt, std::nullopt};
     if (a[0].has("in")) n.in = pt(a[0]["in"]);
     if (a[0].has("out")) n.out = pt(a[0]["out"]);
     Node r = warpNode(n, warpOf(a[1]), epsOf(a, 2));
     json::Object o{{"p", out(r.p)}};
     if (r.in) o["in"] = out(*r.in);
     if (r.out) o["out"] = out(*r.out);
     if (a[0].has("tag")) o["tag"] = a[0]["tag"];
     return Value(o);
   }},
  {"meshGrid", [](auto& a) { return out(meshGrid(pt(a[0]), pt(a[1]))); }},
  {"warpAt", [](auto& a) {
     Warp f = warpOf(a[0]);
     std::vector<Pt> r;
     for (Pt p : pointsOf(a[1])) r.push_back(f(p));
     return out(r);
   }},
  {"warpCubicAt", [](auto& a) { return outCubics(warpCubic(cub(a[0]), warpOf(a[1]), optionsOf(a, 2))); }},
  {"warpCubicsAt", [](auto& a) {
     std::vector<Cubic> cs;
     for (auto& e : a[0].arr()) cs.push_back(cub(e));
     return outCubics(warpCubics(cs, warpOf(a[1]), optionsOf(a, 2)));
   }},
  {"warpContoursAt", [](auto& a) {
     json::Array r;
     for (auto& c : warpContours(contours(a[0]), warpOf(a[1]), optionsOf(a, 2))) r.push_back(out(c));
     return Value(r);
   }},
  {"wavelength", [](auto& a) { return out(wavelength(wobbleOf(a[0]))); }},
  {"wobbleStep", [](auto& a) { return out(wobbleStep(wobbleOf(a[0]))); }},
  {"wobbleWaveAt", [](auto& a) {
     Wave w = wobbleWave(wobbleOf(a[0]), a[1].num(), a[2].isNull() ? false : a[2].boolean());
     std::vector<double> r;
     for (auto& x : a[3].arr()) r.push_back(w(x.num()));
     return out(r);
   }},
  {"scatterAt", [](auto& a) {
     std::vector<double> r;
     for (auto& k : a[1].arr()) r.push_back(scatter(a[0].num(), k.num()));
     return out(r);
   }},
  {"noise2At", [](auto& a) {
     auto f = noise2(a[0].num());
     std::vector<double> r;
     for (Pt p : pointsOf(a[1])) r.push_back(f(p.x, p.y));
     return out(r);
   }},
});

}  // namespace
