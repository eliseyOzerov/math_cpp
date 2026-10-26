// The golden cases of spline.ts.
#include "cases.hpp"
#include "paths.hpp"

#include "curves/spline.hpp"

namespace {

using namespace golden;

CatmullRomOptions crOpts(const Value& a, size_t i) {
  CatmullRomOptions o;
  if (i >= a.size() || a[i].isNull()) return o;
  const Value& v = a[i];
  if (v.has("alpha")) o.alpha = v["alpha"].num();
  if (v.has("tension")) o.tension = v["tension"].num();
  if (v.has("closed")) o.closed = v["closed"].boolean();
  if (v.has("metric")) o.metric = pt(v["metric"]);
  return o;
}

BSplineOptions bOpts(const Value& a, size_t i) {
  BSplineOptions o;
  if (i >= a.size() || a[i].isNull()) return o;
  const Value& v = a[i];
  if (v.has("degree")) o.degree = integer(v["degree"]);
  if (v.has("knots")) o.knots = list(v["knots"], num);
  if (v.has("weights")) o.weights = list(v["weights"], num);
  if (v.has("closed")) o.closed = v["closed"].boolean();
  if (v.has("tol")) o.tol = v["tol"].num();
  if (v.has("metric")) o.metric = pt(v["metric"]);
  return o;
}

static Register reg("spline.json", {
  {"catmullRom", [](const Value& a) { return out(catmullRom(list(a[0], pt), crOpts(a, 1))); }},
  {"catmullRomPath", [](const Value& a) { return ser(catmullRomPath(list(a[0], pt), crOpts(a, 1))); }},
  {"bspline", [](const Value& a) { return out(bspline(list(a[0], pt), bOpts(a, 1))); }},
  {"bsplinePath", [](const Value& a) { return ser(bsplinePath(list(a[0], pt), bOpts(a, 1))); }},
  {"deBoorAt", [](const Value& a) {
     auto pts = list(a[0], pt);
     auto o = bOpts(a, 2);
     json::Array r;
     for (double u : list(a[1], num)) r.push_back(out(deBoor(pts, u, o)));
     return Value(r);
   }},
});

}  // namespace
