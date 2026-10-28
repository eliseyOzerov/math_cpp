// The golden cases of analysis.ts: each TS function by name. A path comes in as {start, segs, closed, metric?} and
// goes out as {start, segs, closed, kinds}; nearness is sampled at the progresses it is given; selectAt gives each
// picked path with its nodes and weights.
#include "cases.hpp"
#include "curves/analysis.hpp"
#include "curves/selection.hpp"

namespace {

using namespace golden;

Path pathOf(const Value& v) {
  std::vector<Cubic> segs;
  for (auto& c : v["segs"].arr()) segs.push_back(cub(c));
  return Path(pt(v["start"]), segs, flag(v["closed"]), {}, v.has("metric") ? pt(v["metric"]) : Pt{1, 1});
}

const char* kindName(NodeKind k) { return k == NodeKind::corner ? "corner" : k == NodeKind::smooth ? "smooth" : "symmetric"; }

Value outPath(const Path& p) {
  json::Array kinds;
  for (NodeKind k : p.kinds()) kinds.push_back(std::string(kindName(k)));
  return Value(json::Object{{"start", out(p.start())}, {"segs", out(p.segs())}, {"closed", out(p.closed())},
                            {"kinds", kinds}});
}

FalloffSpec fall(const Value& v) {
  const std::string& s = v.str();
  return s == "smooth" ? FalloffName::smooth : s == "linear" ? FalloffName::linear : s == "sphere" ? FalloffName::sphere
       : s == "root" ? FalloffName::root : s == "sharp" ? FalloffName::sharp : FalloffName::constant;
}

Way way(const std::string& s) {
  return s == "left" ? Way::left : s == "right" ? Way::right : s == "top" ? Way::top : Way::bottom;
}

Value outOsc(const Osculating& o) {
  json::Object r;
  if (o.center) r["center"] = out(*o.center);
  r["r"] = out(o.r);
  r["k"] = out(o.k);
  return r;
}

Value outContours(const std::vector<Contour>& cs) {
  json::Array a;
  for (auto& c : cs) a.push_back(out(c));
  return a;
}

golden::Register reg("analysis.json", {
  {"project", [](auto& a) {
     auto r = project(pathOf(a[0]), pt(a[1]));
     return Value(json::Object{{"at", out(r.at)}, {"p", out(r.p)}, {"d", out(r.d)}});
   }},
  {"extrema", [](auto& a) { return out(extrema(pathOf(a[0]))); }},
  {"inflections", [](auto& a) { return out(inflections(pathOf(a[0]))); }},
  {"side", [](auto& a) { return out(side(pathOf(a[0]), way(a[1].str()))); }},
  {"turns", [](auto& a) {
     json::Array r;
     for (auto& t : turns(pathOf(a[0]), opt(a, 1).value_or(20)))
       r.push_back(json::Object{{"at", out(t.at)}, {"p", out(t.p)}, {"turn", out(t.turn)}, {"tip", out(t.tip)}});
     return Value(r);
   }},
  {"nearness", [](auto& a) {
     auto f = pathOf(a[0]).nearness(list(a[1], num), num(a[2]), a[3].isNull() ? FalloffSpec(FalloffName::smooth) : fall(a[3]));
     json::Array r;
     for (auto& u : a[4].arr()) r.push_back(out(f(num(u))));
     return Value(r);
   }},
  {"bbox", [](auto& a) { return out(bbox(pathOf(a[0]))); }},
  {"curvature", [](auto& a) { return out(curvature(pathOf(a[0]), num(a[1]))); }},
  {"normal", [](auto& a) { return out(normal(pathOf(a[0]), num(a[1]))); }},
  {"osculating", [](auto& a) { return outOsc(osculating(pathOf(a[0]), num(a[1]))); }},
  {"mold", [](auto& a) { return outPath(mold(pathOf(a[0]), num(a[1]), pt(a[2]))); }},
  {"offset", [](auto& a) { return outPath(offset(pathOf(a[0]), Reach(num(a[1])))); }},
  {"offsetRamp", [](auto& a) {
     double d0 = num(a[1]), d1 = num(a[2]);
     return outPath(offset(pathOf(a[0]), Reach(std::function<double(double)>([=](double u) { return d0 + (d1 - d0) * u; }))));
   }},
  {"outline", [](auto& a) {
     Path p = pathOf(a[0]);
     return outContours(opt(a, 2) ? outline(p, num(a[1]), *opt(a, 2)) : outline(p, num(a[1])));
   }},
  {"lineCrossings", [](auto& a) {
     bool segment = a.size() > 3 && !a[3].isNull() ? flag(a[3]) : true;
     json::Array r;
     for (auto& x : lineCrossings(pathOf(a[0]), pt(a[1]), pt(a[2]), segment))
       r.push_back(json::Object{{"at", out(x.at)}, {"s", out(x.s)}, {"p", out(x.p)}});
     return Value(r);
   }},
  {"circleCrossings", [](auto& a) {
     json::Array r;
     for (auto& x : circleCrossings(pathOf(a[0]), pt(a[1]), num(a[2])))
       r.push_back(json::Object{{"at", out(x.at)}, {"p", out(x.p)}});
     return Value(r);
   }},
  {"selfCrossings", [](auto& a) {
     json::Array r;
     for (auto& x : selfCrossings(pathOf(a[0])))
       r.push_back(json::Object{{"a", out(x.a)}, {"b", out(x.b)}, {"p", out(x.p)}});
     return Value(r);
   }},
  {"arcs", [](auto& a) {
     json::Array r;
     for (auto& x : arcs(pathOf(a[0]), opt(a, 1).value_or(0.5)))
       r.push_back(json::Object{{"center", out(x.center)}, {"r", out(x.r)}, {"start", out(x.start)}, {"end", out(x.end)},
                                {"ccw", out(x.ccw)}, {"from", out(x.from)}, {"to", out(x.to)}, {"t0", out(x.t0)},
                                {"t1", out(x.t1)}, {"at0", out(x.at0)}, {"at1", out(x.at1)}});
     return Value(r);
   }},
  {"selectAt", [](auto& a) {
     SelectAtOptions o;
     if (a.size() > 3 && !a[3].isNull()) {
       if (a[3].has("falloff")) o.falloff = fall(a[3]["falloff"]);
       if (a[3].has("along")) o.along = a[3]["along"].str() == "plane" ? Along::plane : Along::path;
       if (a[3].has("cut")) o.cut = flag(a[3]["cut"]);
     }
     Selection s = pathOf(a[0]).selectAt(num(a[1]), num(a[2]), o);
     json::Array r;
     for (auto& p : s.picked)
       r.push_back(json::Object{{"path", outPath(p.path)}, {"nodes", out(p.nodes)}, {"weights", out(p.weights)}});
     return Value(r);
   }},
});

}  // namespace
