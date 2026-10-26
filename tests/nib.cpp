// The golden cases of nib.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "inkio.hpp"

namespace {

using namespace golden;

std::vector<Pt> pts(const Value& v) { return list(v, pt); }

Value outArcs(const std::array<CornerArc, 4>& arcs) {
  json::Array a;
  for (auto& c : arcs) a.push_back(json::Object{{"s", out(c.s)}, {"o", out(c.o)}, {"r", out(c.r)}});
  return a;
}

std::optional<ShapesInput> shapesIn(const Value& v) {
  if (v.isNull()) return std::nullopt;
  if (v.size() && v[0].isArray()) return ShapesInput(keysS(v));
  std::vector<PartialShape> ps;
  for (auto& e : v.arr()) {
    PartialShape p;
    if (e.has("w")) p.w = e["w"].num();
    if (e.has("h")) p.h = e["h"].num();
    if (e.has("rx")) p.rx = radiusIn(e["rx"]);
    if (e.has("ry")) p.ry = radiusIn(e["ry"]);
    ps.push_back(p);
  }
  return ShapesInput(ps);
}

golden::Register reg("nib.json", {
  {"nibOf", [](auto& a) { return outNib(nibOf(nibArg(a, 0))); }},
  {"stemNib", [](auto& a) { return outNib(stemNib(std::get<StemPen>(nibIn(a[0])))); }},
  {"stemPenOf", [](auto& a) { return outPen(stemPenOf(nibArg(a, 0))); }},
  {"radii", [](auto& a) { return out(radii(radiusIn(a[0]))); }},
  {"uniform", [](auto& a) { return outRadius(uniform(radiusIn(a[0]))); }},
  {"mixShape", [](auto& a) { return outShape(mixShape(shapeIn(a[0]), shapeIn(a[1]), num(a[2]))); }},
  {"cornerArcs", [](auto& a) { return outArcs(cornerArcs(shapeIn(a[0]))); }},
  {"stampPolygon", [](auto& a) {
     return out(stampPolygon(shapeIn(a[0]), opt(a, 1).value_or(0), opt(a, 2).value_or(5)));
   }},
  {"nibWidth", [](auto& a) { return out(nibWidth(nibArg(a, 0), num(a[1]), opt(a, 2).value_or(1))); }},
  {"stemWidth", [](auto& a) { return out(stemWidth(nibArg(a, 0), opt(a, 1).value_or(1))); }},
  {"shapesOf", [](auto& a) {
     auto k = shapesOf(shapesIn(a[0]), std::get<Nib>(nibIn(a[1])));
     return k ? outKeysS(*k) : Value();
   }},
  {"segmentWeight", [](auto& a) { return out(segmentWeight(cub(a[0]), compIn(a[1]))); }},
  {"weightCurve", [](auto& a) {
     return outOptKeysD(weightCurve(list(a[0], cub), flag(a[1]), compIn(a[2]), num(a[3])));
   }},
});

}  // namespace
