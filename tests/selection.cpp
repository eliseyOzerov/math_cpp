// The golden cases of selection.ts: each TS function by name, and Selection's methods as steps of a `chain`. Paths,
// picks and steps are named by data (see tests/golden/cases/selection.ts).
#include "cases.hpp"

#include "curves/selection.hpp"

namespace {

using namespace golden;

bool given(const Value& a, size_t i) { return i < a.size() && !a[i].isNull(); }

NodeKind kindOf(const std::string& s) { return s == "smooth" ? NodeKind::smooth : s == "symmetric" ? NodeKind::symmetric : NodeKind::corner; }
std::string kindName(NodeKind k) { return k == NodeKind::smooth ? "smooth" : k == NodeKind::symmetric ? "symmetric" : "corner"; }

std::vector<Cubic> cubicsOf(const Value& v) { return list(v, cub); }
std::vector<int> intsOf(const Value& v) { std::vector<int> r; for (auto& e : v.arr()) r.push_back(int(e.num())); return r; }

Path pathOf(const Value& s) {
  std::vector<Cubic> segs = cubicsOf(s["segs"]);
  bool closed = s.has("closed") && s["closed"].boolean();
  Pt metric = s.has("metric") ? pt(s["metric"]) : Pt{1, 1};
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (s.has("kinds")) {
    kinds.emplace();
    for (auto& k : s["kinds"].arr()) kinds->push_back(k.isNull() ? std::nullopt : std::optional(kindOf(k.str())));
  }
  return Path(pt(s["start"]), segs, closed, std::nullopt, metric, kinds);
}

Value outPath(const Path& p) {
  json::Array ks;
  for (NodeKind k : p.kinds()) ks.push_back(Value(kindName(k)));
  return Value(json::Object{{"start", out(p.start())}, {"segs", out(p.segs())}, {"closed", out(p.closed())}, {"kinds", Value(ks)}});
}

struct Parsed {
  std::optional<NodePick> np;
  std::optional<Pick> pk;
};

Parsed pickOf(const Value& a, size_t i) {
  Parsed r;
  if (!given(a, i)) return r;
  const Value& s = a[i];
  if (s.isArray()) { r.np = NodePick(intsOf(s)); return r; }
  const std::string& k = s["k"].str();
  auto fn = [&r](std::function<double(Pt, int, Progress)> f) { r.np = NodePick(f); };
  if (k == "nodeAt") r.pk = nodeAt(intsOf(s["i"]));
  else if (k == "cubicAt") r.pk = cubicAt(intsOf(s["i"]));
  else if (k == "section") r.pk = section(s["from"].num(), s["to"].num());
  else if (k == "above") { double y = s["y"].num(); fn([y](Pt p, int, Progress) { return p[1] > y ? 1.0 : 0.0; }); }
  else if (k == "ramp") { double y0 = s["y0"].num(), y1 = s["y1"].num(); fn([=](Pt p, int, Progress) { return (p[1] - y0) / (y1 - y0); }); }
  else if (k == "prog") { double a0 = s["a"].num(), b0 = s["b"].num(); fn([=](Pt, int, Progress at) { return at >= a0 && at <= b0 ? 0.5 + 0.5 * at : 0.0; }); }
  else if (k == "odd") fn([](Pt, int i, Progress) { return double(i % 2); });
  else if (k == "num") { double v = s["v"].num(); fn([v](Pt, int, Progress) { return v; }); }
  else if (k == "bool") { double v = s["v"].boolean() ? 1.0 : 0.0; fn([v](Pt, int, Progress) { return v; }); }
  else throw std::runtime_error("pick " + k);
  return r;
}

Selection selOf(const Value& path, const Parsed& p) {
  Path pa = pathOf(path);
  return p.pk ? Selection::of(pa, *p.pk) : p.np ? Selection::of(pa, *p.np) : Selection::of(pa);
}
Selection selOf(const Value& path, const Value& pick) {
  Value a = json::Array{pick};
  return selOf(path, pickOf(a, 0));
}

Value outSel(const Selection& s) {
  json::Array picked, weights, paths;
  for (auto& p : s.picked) {
    picked.push_back(Value(json::Object{{"path", outPath(p.path)}, {"nodes", out(p.nodes)}, {"weights", out(p.weights)}}));
    paths.push_back(outPath(p.path));
  }
  for (auto& w : s.weights()) weights.push_back(out(w));
  auto pts = s.points();
  json::Object o{{"picked", Value(picked)}, {"weights", Value(weights)}, {"points", out(pts)}, {"metric", out(s.metric())}, {"paths", Value(paths)}};
  o["center"] = pts.empty() ? Value() : out(s.center());
  o["bounds"] = pts.empty() ? Value() : out(s.bounds());
  o["path"] = s.picked.size() == 1 ? outPath(s.path()) : Value();
  return Value(o);
}

Value pickOn(const Value& path, const Pick& p) {
  Path pa = pathOf(path);
  PickOn on = p.on(pa);
  Selection sel = Selection::of(pa, p);
  json::Object o{{"path", outPath(on.path)}, {"weights", out(sel.picked[0].weights)}};
  o["pick"] = std::holds_alternative<std::vector<int>>(on.pick) ? out(std::get<std::vector<int>>(on.pick)) : Value();
  return Value(o);
}

Wave waveOf(const Value& s) {
  Swing o;
  if (s.has("cycles")) o.cycles = s["cycles"].num();
  if (s.has("phase")) o.phase = s["phase"].num();
  if (s.has("low")) o.low = s["low"].num();
  if (s.has("high")) o.high = s["high"].num();
  return s["w"].str() == "saw" ? saw(o) : sine(o);
}
Strength strengthOf(const Value& s) {
  if (std::holds_alternative<double>(s.v)) return s.num();
  if (s.isArray()) { std::vector<double> r; for (auto& e : s.arr()) r.push_back(e.num()); return r; }
  return waveOf(s);
}

FalloffName nameOf(const std::string& s) {
  return s == "linear" ? FalloffName::linear : s == "sphere" ? FalloffName::sphere : s == "root" ? FalloffName::root
       : s == "sharp" ? FalloffName::sharp : s == "constant" ? FalloffName::constant : FalloffName::smooth;
}
FalloffSpec fallOf(const Value& s) {
  if (std::holds_alternative<std::string>(s.v)) return nameOf(s.str());
  const std::string& f = s["fn"].str();
  return Falloff(f == "sq" ? Falloff([](double x) { return 1 - x * x; }) : f == "over" ? Falloff([](double x) { return 2 - x; }) : Falloff([](double x) { return x - 0.5; }));
}

Pivot pivotOf(const Value& a, size_t i) {
  if (!given(a, i) || std::holds_alternative<std::string>(a[i].v)) return std::nullopt;
  return pt(a[i]);
}

NodeMap moveMap(const Value& s) {
  const std::string& k = s["k"].str();
  using M = std::function<Pt(Pt)>;
  if (k == "shift") { Pt d = pt(s["d"]); return [d](Pt, const Path&, int) { return M([d](Pt p) { return Pt{p[0] + d[0], p[1] + d[1]}; }); }; }
  if (k == "byIndex") { Pt d = pt(s["d"]); return [d](Pt, const Path&, int i) { return M([d, i](Pt p) { return Pt{p[0] + d[0] * i, p[1] + d[1]}; }); }; }
  if (k == "aboutNode") {
    double sc = s["s"].num();
    return [sc](Pt node, const Path&, int) { return M([sc, node](Pt p) { return Pt{node[0] + (p[0] - node[0]) * sc, node[1] + (p[1] - node[1]) * sc}; }); };
  }
  return [](Pt, const Path& path, int) { double n = double(path.segs().size()); return M([n](Pt p) { return Pt{p[0] + n, p[1]}; }); };
}

Selection step(const Selection& sel, const Value& st) {
  const std::string& name = st[0].str();
  auto at = [&](size_t i) -> const Value& { return st[i + 1]; };
  auto has = [&](size_t i) { return given(st, i + 1); };
  if (name == "soften") {
    SoftenOptions o;
    if (has(1)) {
      if (at(1).has("falloff")) o.falloff = fallOf(at(1)["falloff"]);
      if (at(1).has("along")) o.along = at(1)["along"].str() == "plane" ? Along::plane : Along::path;
    }
    return sel.soften(at(0).num(), o);
  }
  if (name == "weigh") return sel.weigh(strengthOf(at(0)));
  if (name == "invert") return sel.invert();
  if (name == "transform") return sel.transform(aff(at(0)));
  if (name == "moveEach") return sel.moveEach(moveMap(at(0)));
  if (name == "put") {
    Place pl;
    const Value& v = at(0);
    if (v.isArray()) { pl.x = v[0].num(); pl.y = v[1].num(); }
    else { if (v.has("x")) pl.x = v["x"].num(); if (v.has("y")) pl.y = v["y"].num(); }
    return sel.put(pl);
  }
  if (name == "kind") return sel.kind(kindOf(at(0).str()));
  if (name == "handles") {
    HandlesOptions o;
    const Value& v = at(0);
    if (v.has("angle")) o.angle = v["angle"].num();
    if (v.has("turn")) o.turn = v["turn"].num();
    if (v.has("length")) o.length = v["length"].num();
    if (v.has("side")) o.side = v["side"].str() == "in" ? HandleLead::in : v["side"].str() == "out" ? HandleLead::out : HandleLead::both;
    return sel.handles(o);
  }
  if (name == "inflate") {
    InflateOptions o;
    if (has(1)) {
      const Value& v = at(1);
      if (v.has("align")) o.align = pt(v["align"]);
      if (v.has("handles")) o.handles = v["handles"].num();
      if (v.has("smooth")) o.smooth = v["smooth"].num();
      if (v.has("strength")) o.strength = strengthOf(v["strength"]);
      if (v.has("along")) o.along = v["along"].str() == "path" ? InflateAlong::path : InflateAlong::ring;
    }
    return sel.inflate(at(0).num(), o);
  }
  if (name == "smooth") return has(0) ? sel.smooth(at(0).num()) : sel.smooth();
  if (name == "fair") return has(0) ? sel.fair(at(0).num()) : sel.fair();
  if (name == "rotate") return sel.rotate(at(0).num(), pivotOf(st, 2));
  if (name == "shear") {
    ShearOptions o;
    if (has(1)) {
      if (at(1).has("axis")) o.axis = at(1)["axis"].num();
      if (at(1).has("about") && !std::holds_alternative<std::string>(at(1)["about"].v)) o.about = pt(at(1)["about"]);
    }
    return sel.shear(at(0).num(), o);
  }
  if (name == "shift") return sel.shift(at(0).num(), has(1) ? at(1).num() : 0);
  if (name == "scale") {
    Pivot about = pivotOf(st, 2);
    return std::holds_alternative<double>(at(0).v) ? sel.scale(at(0).num(), about) : sel.scale(pt(at(0)), about);
  }
  if (name == "mirror") return sel.mirror(has(0) ? at(0).num() : 90, pivotOf(st, 2));
  if (name == "select") {
    Parsed p = pickOf(st, 1);
    return p.pk ? sel.select(*p.pk) : p.np ? sel.select(*p.np) : sel.select();
  }
  if (name == "and") {
    std::vector<Selection> others;
    for (size_t i = 1; i < st.size(); i++) others.push_back(selOf(st[i][0], st[i][1]));
    return sel.and_(others);
  }
  throw std::runtime_error("step " + name);
}

Selection run(Selection sel, const Value& steps) {
  for (auto& st : steps.arr()) sel = step(sel, st);
  return sel;
}

golden::Register reg("selection.json", {
  {"chain", [](auto& a) { return outSel(run(selOf(a[0], pickOf(a, 1)), a[2])); }},
  {"nodeAt", [](auto& a) { std::vector<int> i; for (size_t k = 1; k < a.size(); k++) i.push_back(integer(a[k])); return pickOn(a[0], nodeAt(i)); }},
  {"cubicAt", [](auto& a) { std::vector<int> i; for (size_t k = 1; k < a.size(); k++) i.push_back(integer(a[k])); return pickOn(a[0], cubicAt(i)); }},
  {"section", [](auto& a) { return pickOn(a[0], section(a[1].num(), a[2].num())); }},
  {"falloffAt", [](auto& a) {
     Falloff f = falloff(nameOf(a[0].str()));
     std::vector<double> r;
     for (auto& x : a[1].arr()) r.push_back(f(x.num()));
     return out(r);
   }},
  {"inflation", [](auto& a) { return out(inflation(pt(a[0]), pt(a[1]), a[2].num())); }},
  {"moveNodes", [](auto& a) {
     double dx = a[3]["dx"].num(), dy = a[3]["dy"].num();
     return out(moveNodes(cubicsOf(a[0]), flag(a[1]), intsOf(a[2]), [=](Pt p, int i) { return Pt{p[0] + dx * (i + 1), p[1] + dy * (i % 2 ? 1 : -1)}; }));
   }},
  {"smoothNodes", [](auto& a) {
     Pt metric = given(a, 4) ? pt(a[4]) : Pt{1, 1};
     auto segs = cubicsOf(a[0]);
     if (given(a, 3) && a[3].isObject()) {
       std::vector<double> by;
       for (auto& e : a[3]["byNode"].arr()) by.push_back(e.num());
       return out(smoothNodes(segs, flag(a[1]), intsOf(a[2]), std::function<double(int)>([by](int i) { return i < int(by.size()) ? by[i] : NAN; }), metric));
     }
     return out(smoothNodes(segs, flag(a[1]), intsOf(a[2]), given(a, 3) ? a[3].num() : 1.0, metric));
   }},
  {"fairNodes", [](auto& a) {
     std::optional<std::vector<double>> ws;
     if (given(a, 5)) { ws.emplace(); for (auto& e : a[5].arr()) ws->push_back(e.num()); }
     Faired r = fairNodes(cubicsOf(a[0]), flag(a[1]), intsOf(a[2]), given(a, 3) ? a[3].num() : 0.5, given(a, 4) ? pt(a[4]) : Pt{1, 1}, ws);
     return Value(json::Object{{"segs", out(r.segs)}, {"nodes", out(r.nodes)}, {"weights", out(r.weights)}});
   }},
  {"selectOf", [](auto& a) { return outSel(selOf(a[0], pickOf(a, 1))); }},
  {"pathSelect", [](auto& a) {
     Path p = pathOf(a[0]);
     Parsed k = pickOf(a, 1);
     return outSel(k.pk ? p.select(*k.pk) : k.np ? p.select(*k.np) : p.select());
   }},
  {"withKind", [](auto& a) {
     Path p = pathOf(a[0]);
     NodeKind kind = kindOf(a[1].str());
     Parsed k = pickOf(a, 2);
     if (k.pk) return outPath(p.withKind(kind, NodePick(p.select(*k.pk).picked[0].nodes)));
     return outPath(k.np ? p.withKind(kind, *k.np) : p.withKind(kind));
   }},
  {"group", [](auto& a) {
     std::vector<Selection> items;
     for (auto& it : a[0].arr()) items.push_back(it.has("sel") ? selOf(it["sel"][0], it["sel"][1]) : Selection::of(pathOf(it)));
     return outSel(run(group(items), given(a, 1) ? a[1] : Value(json::Array{})));
   }},
});

}  // namespace
