// The golden cases of geometry.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"

namespace {

using namespace golden;

golden::Register reg("geometry.json", {
  {"norm", [](auto& a) { return out(norm(pt(a[0]))); }},
  {"halfTurn", [](auto& a) { return out(halfTurn(num(a[0]))); }},
  {"turn", [](auto& a) { return out(turn(pt(a[0]), pt(a[1]))); }},
  {"mean", [](auto& a) { return out(mean(list(a[0], pt))); }},
  {"bez", [](auto& a) { return out(bez(cub(a[0]), num(a[1]))); }},
  {"bezD", [](auto& a) { return out(bezD(cub(a[0]), num(a[1]))); }},
  {"bezDD", [](auto& a) { return out(bezDD(cub(a[0]), num(a[1]))); }},
  {"curvature", [](auto& a) { return out(curvature(cub(a[0]), num(a[1]))); }},
  {"isLine", [](auto& a) { return out(isLine(cub(a[0]))); }},
  {"split", [](auto& a) { return out(split(cub(a[0]), num(a[1]))); }},
  {"trim", [](auto& a) { return out(trim(cub(a[0]), num(a[1]), num(a[2]))); }},
  {"arcLength", [](auto& a) { return out(arcLength(cub(a[0]), opt(a, 1).value_or(1))); }},
  {"tAtLength", [](auto& a) { return out(tAtLength(cub(a[0]), num(a[1]), opt(a, 2))); }},
  {"splitEven", [](auto& a) { return out(splitEven(cub(a[0]), int(num(a[1])))); }},
  {"cutAt", [](auto& a) {
     auto shares = list(a[1], num);
     return out(a.size() > 2 ? cutAt(cub(a[0]), shares, pt(a[2])) : cutAt(cub(a[0]), shares));
   }},
  {"hullLength", [](auto& a) { return out(hullLength(cub(a[0]))); }},
  {"roots", [](auto& a) { return out(roots(cub(a[0]), int(num(a[1])), num(a[2]), int(opt(a, 3).value_or(32)))); }},
  {"nearest", [](auto& a) {
     Nearest n = nearest(list(a[0], cub), pt(a[1]), int(opt(a, 2).value_or(32)));
     return Value(json::Object{{"i", out(n.i)}, {"t", out(n.t)}, {"p", out(n.p)}, {"d", out(n.d)}});
   }},
  {"nearSegment", [](auto& a) { return out(nearSegment(pt(a[0]), pt(a[1]), pt(a[2]), num(a[3]), opt(a, 4))); }},
  {"meet", [](auto& a) {
     double flat = a.size() > 4 && a[4].has("flat") ? a[4]["flat"].num() : 1e-9;
     auto inside = a.size() > 4 && a[4].has("inside") ? std::optional(a[4]["inside"].num()) : std::nullopt;
     return out(meet(pt(a[0]), pt(a[1]), pt(a[2]), pt(a[3]), flat, inside));
   }},
  {"endTangents", [](auto& a) { return out(endTangents(piece(a[0]))); }},
  {"reverse", [](auto& a) { return out(reverse(piece(a[0]))); }},
  {"pieceLength", [](auto& a) { return out(pieceLength(piece(a[0]))); }},
  {"asCubic", [](auto& a) { return out(asCubic(piece(a[0]))); }},
  {"box", [](auto& a) { return out(box(list(a[0], piece), int(opt(a, 1).value_or(16)))); }},
  {"stitch", [](auto& a) { Contour c = list(a[0], piece); return out(stitch(c)); }},
  {"arcCubics", [](auto& a) { return out(arcCubics(pt(a[0]), pt(a[1]), num(a[2]))); }},
  {"arcCubic", [](auto& a) { return out(arcCubic(pt(a[0]), pt(a[1]), pt(a[2]), pt(a[3]))); }},
  {"runLengths", [](auto& a) { return out(runLengths(list(a[0], pt))); }},
  {"area", [](auto& a) { return out(area(list(a[0], pt))); }},
  {"pointAlong", [](auto& a) { return out(pointAlong(list(a[0], pt), list(a[1], num), num(a[2]))); }},
  {"bounds", [](auto& a) { return out(bounds(list(a[0], pt))); }},
  {"center", [](auto& a) { return out(center(list(a[0], pt))); }},
  {"overlaps", [](auto& a) { return out(overlaps(box2(a[0]), box2(a[1]), opt(a, 2).value_or(0))); }},
  {"rotation", [](auto& a) { return out(rotation(num(a[0]), a.size() > 1 ? pt(a[1]) : Pt{0, 0})); }},
  {"shear", [](auto& a) { return out(shear(num(a[0]), opt(a, 1).value_or(0), a.size() > 2 ? pt(a[2]) : Pt{0, 0})); }},
  {"inMetric", [](auto& a) { return out(inMetric(aff(a[0]), pt(a[1]))); }},
  {"scaling", [](auto& a) {
     double kx = num(a[0]);
     return out(scaling(kx, opt(a, 1).value_or(kx), a.size() > 2 ? pt(a[2]) : Pt{0, 0}));
   }},
  {"partly", [](auto& a) { return out(partly(aff(a[0]), num(a[1]))); }},
  {"shift", [](auto& a) { return out(shift(num(a[0]), opt(a, 1).value_or(0))); }},
  {"then", [](auto& a) { return out(then(aff(a[0]), aff(a[1]))); }},
  {"apply", [](auto& a) { return out(curves::apply(aff(a[0]), pt(a[1]))); }},
});

}  // namespace
