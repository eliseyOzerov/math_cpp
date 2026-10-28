// The golden cases of bezier.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"

namespace {

using namespace golden;

golden::Register reg("bezier.json", {
  {"cubicRoots", [](auto& a) { return out(cubicRoots(num(a[0]), num(a[1]), num(a[2]), num(a[3]))); }},
  {"inUnit", [](auto& a) { return out(inUnit(list(a[0], num))); }},
  {"coordRoots", [](auto& a) { return out(coordRoots(num(a[0]), num(a[1]), num(a[2]), num(a[3]), opt(a, 4).value_or(0))); }},
  {"cubicCoordRoots", [](auto& a) { return out(cubicCoordRoots(cub(a[0]), int(num(a[1])), num(a[2]))); }},
  {"extrema", [](auto& a) {
     Extrema e = extrema(cub(a[0]));
     return Value(json::Object{{"x", out(e.x)}, {"y", out(e.y)}, {"values", out(e.values)}});
   }},
  {"bbox", [](auto& a) { return out(bbox(cub(a[0]))); }},
  {"align", [](auto& a) { return out(a.size() > 1 ? align(cub(a[0]), pt(a[1]), pt(a[2])) : align(cub(a[0]))); }},
  {"inflections", [](auto& a) { return out(inflections(cub(a[0]))); }},
  {"tightBox", [](auto& a) { return out(tightBox(cub(a[0]))); }},
  {"canonical", [](auto& a) {
     Canonical k = canonical(cub(a[0]), opt(a, 1).value_or(1e-6));
     return Value(json::Object{{"kind", std::string(name(k.kind))}, {"at", out(k.at)}});
   }},
  {"lut", [](auto& a) { return out(lut(cub(a[0]), int(opt(a, 1).value_or(100)))); }},
  {"casteljau", [](auto& a) { return out(casteljau(cub(a[0]), num(a[1]))); }},
  {"tangent", [](auto& a) { return out(tangent(cub(a[0]), num(a[1]))); }},
  {"normal", [](auto& a) { return out(normal(cub(a[0]), num(a[1]))); }},
  {"osculating", [](auto& a) {
     Osculating o = osculating(cub(a[0]), num(a[1]));
     return Value(json::Object{{"center", out(o.center)}, {"r", out(o.r)}, {"k", out(o.k)}});
   }},
  {"project", [](auto& a) {
     Projection r = project(cub(a[0]), pt(a[1]));
     return Value(json::Object{{"t", out(r.t)}, {"p", out(r.p)}, {"d", out(r.d)}});
   }},
  {"mold", [](auto& a) { return out(mold(cub(a[0]), num(a[1]), pt(a[2]))); }},
  {"quadThrough", [](auto& a) { return out(quadThrough(pt(a[0]), pt(a[1]), pt(a[2]), opt(a, 3))); }},
  {"cubicThrough", [](auto& a) {
     ThroughOptions o;
     if (a.size() > 3) {
       if (a[3].has("t")) o.t = a[3]["t"].num();
       if (a[3].has("d1")) o.d1 = a[3]["d1"].num();
       if (a[3].has("dir")) o.dir = pt(a[3]["dir"]);
     }
     return out(cubicThrough(pt(a[0]), pt(a[1]), pt(a[2]), o));
   }},
});

}  // namespace
