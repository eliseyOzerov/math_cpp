// The golden cases of offset.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"
#include "curves/offset.hpp"

namespace {

using namespace golden;

Value outReduced(const std::vector<Reduced>& rs) {
  json::Array a;
  for (auto& r : rs) a.push_back(json::Object{{"c", out(r.c)}, {"t0", out(r.t0)}, {"t1", out(r.t1)}});
  return a;
}

Value outArcs(const std::vector<Arc>& as) {
  json::Array a;
  for (auto& r : as)
    a.push_back(json::Object{{"center", out(r.center)}, {"r", out(r.r)}, {"start", out(r.start)}, {"end", out(r.end)},
                             {"ccw", out(r.ccw)}, {"from", out(r.from)}, {"to", out(r.to)}, {"t0", out(r.t0)},
                             {"t1", out(r.t1)}});
  return a;
}

std::function<double(double)> ramp(double d0, double d1) { return [=](double t) { return d0 + (d1 - d0) * t; }; }

golden::Register reg("offset.json", {
  {"simple", [](auto& a) { return out(simple(cub(a[0]))); }},
  {"reduce", [](auto& a) { return outReduced(reduce(cub(a[0]))); }},
  {"scale", [](auto& a) { return out(scale(cub(a[0]), num(a[1]))); }},
  {"scaleRamp", [](auto& a) { return out(scale(cub(a[0]), Reach(ramp(num(a[1]), num(a[2]))))); }},
  {"offset", [](auto& a) { return out(offset(cub(a[0]), num(a[1]))); }},
  {"offsetRamp", [](auto& a) { return out(offset(cub(a[0]), Reach(ramp(num(a[1]), num(a[2]))))); }},
  {"offsetAt", [](auto& a) { return out(offsetAt(cub(a[0]), num(a[1]), num(a[2]))); }},
  {"outline", [](auto& a) { return out(outline(cub(a[0]), num(a[1]), opt(a, 2), opt(a, 3), opt(a, 4))); }},
  {"outlineShapes", [](auto& a) {
     json::Array r;
     for (auto& s : outlineShapes(cub(a[0]), num(a[1]), opt(a, 2), opt(a, 3), opt(a, 4))) r.push_back(out(s));
     return Value(r);
   }},
  {"arcs", [](auto& a) { return outArcs(arcs(cub(a[0]), opt(a, 1).value_or(0.5))); }},
});

}  // namespace
