// The golden cases of corners.ts. Callbacks are passed by name (cases/corners.ts has the same tables), since the TS
// takes functions.
#include "cases.hpp"

#include "curves/corners.hpp"

namespace {

using namespace golden;
using Fnk = std::function<double(const Corner&)>;

const std::map<std::string, Fnk> radii{
  {"byTurn", [](auto& k) { return k.turn < 0 ? 10.0 : 4.0; }},
  {"byAt", [](auto& k) { return k.at % 2 ? 0.0 : 12.0; }},
};
const std::map<std::string, Fnk> reaches{
  {"ten", [](auto&) { return 10.0; }},
  {"byTurn", [](auto& k) { return k.turn < 0 ? 0.0 : 15.0; }},
  {"none", [](auto&) { return 0.0; }},
};
const std::map<std::string, MakeCorner> makes{
  {"chamfer", [](Pt a, Pt, Pt b, Pt, const Corner&) { return std::vector<Piece>{Piece::of(a, b)}; }},
  {"fillet", [](Pt a, Pt ta, Pt b, Pt tb, const Corner&) { return std::vector<Piece>{Piece::of(arcCubic(a, ta, b, tb))}; }},
  {"bulge", [](Pt a, Pt, Pt b, Pt, const Corner&) {
     std::vector<Piece> out;
     for (auto& c : arcCubics(a, b, 0.6)) out.push_back(Piece::of(c));
     return out;
   }},
};
const std::map<std::string, Fnk> sizes{
  {"ten", [](auto&) { return 10.0; }},
  {"twenty", [](auto&) { return 20.0; }},
  {"byTurn", [](auto& k) { return k.turn < 0 ? 8.0 : 14.0; }},
};
using Shp = std::function<CornerShape(const Corner&)>;
const std::map<std::string, Shp> shapes{
  {"zero", [](auto&) { return CornerShape(0); }},
  {"out", [](auto&) { return CornerShape(0.5); }},
  {"in", [](auto&) { return CornerShape(-0.5); }},
  {"outFull", [](auto&) { return CornerShape(1); }},
  {"inFull", [](auto&) { return CornerShape(-1); }},
  {"over", [](auto&) { return CornerShape(1.5); }},
  {"under", [](auto&) { return CornerShape(-1.5); }},
  {"mixed", [](auto& k) { return CornerShape(k.turn < 0 ? 0.7 : -0.7); }},
  {"rect", [](auto&) { return CornerShape(0.5, caps::rect(1)); }},
  {"rectIn", [](auto&) { return CornerShape(-0.8, caps::rect(0.6)); }},
  {"triangle", [](auto&) { return CornerShape(-0.6, caps::triangle(1)); }},
  {"arc", [](auto&) { return CornerShape(0.4, caps::arc(1)); }},
  {"flatProfile", [](auto&) { return CornerShape(0.5, {Piece::of({-1, 0}, {1, 0})}); }},
  {"emptyProfile", [](auto&) { return CornerShape(0.5, {}); }},
  {"profileOnly", [](auto&) { return CornerShape(0, caps::rect(0.5)); }},
  {"objectNoProfile", [](auto&) { return CornerShape(0.5); }},
};

CornerOpts opts(const Value& a, size_t i) {
  CornerOpts o;
  o.minDeg = optKey(a, i, "minDeg").value_or(o.minDeg);
  o.share = optKey(a, i, "share").value_or(o.share);
  return o;
}
Value out(const Corner& k) {
  return json::Object{{"contour", golden::out(k.contour)}, {"at", golden::out(k.at)}, {"p", golden::out(k.p)},
                      {"in", golden::out(k.in)}, {"out", golden::out(k.out)}, {"turn", golden::out(k.turn)},
                      {"outside", golden::out(k.outside)}};
}
Value out(const std::vector<Corner>& ks) { json::Array a; for (auto& k : ks) a.push_back(out(k)); return a; }

golden::Register reg("corners.json", {
  {"corners", [](auto& a) { return out(corners(contours(a[0]), opt(a, 1).value_or(1))); }},
  {"pits", [](auto& a) { return out(pits(contours(a[0]), opt(a, 1).value_or(170))); }},
  {"peaks", [](auto& a) { return out(peaks(contours(a[0]), opt(a, 1).value_or(10))); }},
  {"roundCorners", [](auto& a) { return golden::out(roundCorners(contours(a[0]), num(a[1]), opts(a, 2))); }},
  {"roundCornersBy", [](auto& a) { return golden::out(roundCorners(contours(a[0]), radii.at(a[1].str()), opts(a, 2))); }},
  {"replaceCornersBy", [](auto& a) {
     return golden::out(replaceCorners(contours(a[0]), reaches.at(a[1].str()), makes.at(a[2].str()), opts(a, 3)));
   }},
  {"shapeCornersBy", [](auto& a) {
     return golden::out(shapeCorners(contours(a[0]), sizes.at(a[1].str()), shapes.at(a[2].str())));
   }},
  {"capShape", [](auto& a) {
     auto h = opt(a, 1);
     std::string n = a[0].str();
     return golden::out(n == "triangle" ? (h ? caps::triangle(*h) : caps::triangle())
                        : n == "rect"   ? (h ? caps::rect(*h) : caps::rect())
                                        : (h ? caps::arc(*h) : caps::arc()));
   }},
  {"capped", [](auto& a) {
     EndFrame e{pt(a[1]["p"]), pt(a[1]["t"]), num(a[1]["r"])};
     Cap c{list(a[2]["shape"], piece), optKey(a, 2, "align")};
     return golden::out(capped(contours(a[0]), e, c));
   }},
});

}  // namespace
