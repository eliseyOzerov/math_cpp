// The golden cases of fit.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"
#include "curves/fit.hpp"

namespace {

using namespace golden;

std::vector<Pt> pts(const Value& v) { return list(v, pt); }

golden::Register reg("fit.json", {
  {"fitCubic", [](auto& a) {
     CubicFit f = fitCubic(pts(a[0]), pt(a[1]), pt(a[2]));
     return Value(json::Object{{"c", out(f.c)}, {"err", out(f.err)}});
   }},
  {"fitCubics", [](auto& a) {
     FitOptions o;
     if (auto c = optKey(a, 2, "corner")) o.corner = *c;
     return out(fitCubics(pts(a[0]), num(a[1]), o));
   }},
  {"cubicToQuads", [](auto& a) { return out(cubicToQuads(cub(a[0]), num(a[1]))); }},
  {"hull", [](auto& a) { return out(hull(pts(a[0]), a.size() > 1 && flag(a[1]))); }},
  {"mergeHulls", [](auto& a) { return out(mergeHulls(pts(a[0]), pts(a[1]))); }},
  {"clipPolygon", [](auto& a) { return out(clipPolygon(pts(a[0]), Plane{pt(a[1]["at"]), pt(a[1]["n"])})); }},
});

}  // namespace
