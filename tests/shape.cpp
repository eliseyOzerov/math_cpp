// The golden cases of shape.ts.
#include "cases.hpp"
#include "paths.hpp"

#include "curves/shape.hpp"

namespace {

using namespace golden;

Softness softness(const Value& a, size_t i) {
  Softness s;
  if (i >= a.size() || a[i].isNull()) return s;
  const Value& v = a[i];
  if (v.has("radius")) s.radius = v["radius"].num();
  if (v.has("bulge")) {
    if (v["bulge"].isArray()) s.bulge = list(v["bulge"], num);
    else s.bulge = v["bulge"].num();
  }
  return s;
}

Pt metric(const Value& a, size_t i) { return i < a.size() && !a[i].isNull() ? pt(a[i]) : Pt{1, 1}; }

Side side(const std::string& s) {
  return s == "right" ? Side::right : s == "top" ? Side::top : s == "left" ? Side::left : Side::bottom;
}

static Register reg("shape.json", {
  {"bowed", [](const Value& a) { return out(bowed(pt(a[0]), pt(a[1]), opt(a, 2).value_or(0))); }},
  {"fitBulges", [](const Value& a) { return out(fitBulges(list(a[0], pt), list(a[1], num))); }},
  {"softPolyline", [](const Value& a) { return out(softPolyline(list(a[0], pt), softness(a, 1))); }},
  {"soft", [](const Value& a) { return ser(soft(list(a[0], pt), softness(a, 1), metric(a, 2))); }},
  {"oval", [](const Value& a) {
     OvalOptions o;
     if (a.size() > 4 && !a[4].isNull()) {
       if (a[4].has("tension")) o.tension = a[4]["tension"].num();
       if (a[4].has("start")) o.start = side(a[4]["start"].str());
     }
     return ser(oval(a[0].num(), a[1].num(), a[2].num(), a[3].num(), o));
   }},
  {"lobe", [](const Value& a) {
     LobeOptions o;
     if (a.size() > 2 && !a[2].isNull()) {
       const Value& v = a[2];
       if (v.has("reach")) o.reach = v["reach"].num();
       if (v.has("nodes")) o.nodes = v["nodes"].num();
       if (v.has("radius")) o.radius = v["radius"].num();
       if (v.has("bulge")) o.bulge = v["bulge"].num();
       if (v.has("axis")) o.axis = v["axis"].num();
       if (v.has("reachFrom")) o.reachFrom = v["reachFrom"].str() == "start" ? ReachFrom::start : ReachFrom::line;
     }
     return ser(lobe(pt(a[0]), pt(a[1]), o, metric(a, 3)));
   }},
});

}  // namespace
