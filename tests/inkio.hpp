// Reading and writing the ink's types for the golden tests (nib.cpp, ink.cpp, outline.cpp): arguments as gen.ts wrote
// them, results in the shape the TS returned. Explicit names (outShape, outInk …) rather than `out` overloads: the
// `out` templates in cases.hpp find overloads declared after them only by argument-dependent lookup.
#pragma once

#include "cases.hpp"

#include "curves/ink.hpp"
#include "curves/nib.hpp"
#include "curves/outline.hpp"

namespace golden {

inline Radius radiusIn(const Value& v) {
  if (v.isArray()) return list<double>(v, [](const Value& e) { return e.num(); });
  return v.num();
}
inline Value outRadius(const Radius& r) {
  if (auto d = std::get_if<double>(&r)) return out(*d);
  json::Array a;
  for (double x : std::get<std::vector<double>>(r)) a.push_back(out(x));
  return a;
}
inline Shape shapeIn(const Value& v) { return {v["w"].num(), v["h"].num(), radiusIn(v["rx"]), radiusIn(v["ry"])}; }
inline json::Object shapeFields(const Shape& s) {
  return {{"w", out(s.w)}, {"h", out(s.h)}, {"rx", outRadius(s.rx)}, {"ry", outRadius(s.ry)}};
}
inline Value outShape(const Shape& s) { return shapeFields(s); }
inline Value outFrame(const Frame& f) {
  json::Object o = shapeFields(f);
  o["c"] = out(f.c);
  o["rot"] = out(f.rot);
  return o;
}
inline Value outNib(const Nib& n) {
  return json::Object{{"w", out(n.w)}, {"h", out(n.h)}, {"rx", outRadius(n.rx)}, {"ry", outRadius(n.ry)}, {"angle", out(n.angle)},
                      {"follow", out(n.follow)}, {"perCorner", out(n.perCorner)}, {"perSpan", out(n.perSpan)}};
}
inline Value outPen(const StemPen& p) {
  return json::Object{{"weight", out(p.weight)}, {"contrast", out(p.contrast)}, {"stress", out(p.stress)}};
}

/** A nib given as the TS cases give it: an object of some nib fields (the rest the nib's own) or of a pen. */
inline NibInput nibIn(const Value& v) {
  if (v.isNull()) return std::monostate{};
  if (std::holds_alternative<double>(v.v)) return v.num();
  if (v.has("weight") && !v.has("w")) {
    StemPen p{v["weight"].num()};
    if (v.has("contrast")) p.contrast = v["contrast"].num();
    if (v.has("stress")) p.stress = v["stress"].num();
    return p;
  }
  Nib n = NIB;
  if (v.has("w")) n.w = v["w"].num();
  if (v.has("h")) n.h = v["h"].num();
  if (v.has("rx")) n.rx = radiusIn(v["rx"]);
  if (v.has("ry")) n.ry = radiusIn(v["ry"]);
  if (v.has("angle")) n.angle = v["angle"].num();
  if (v.has("follow")) n.follow = v["follow"].num();
  if (v.has("perCorner")) n.perCorner = v["perCorner"].num();
  if (v.has("perSpan")) n.perSpan = v["perSpan"].num();
  return n;
}
/** The nib input at argument `i`, none where the call left it out. */
inline NibInput nibArg(const Value& a, size_t i) { return i < a.size() ? nibIn(a[i]) : NibInput{}; }

inline Compensation compIn(const Value& v) {
  Compensation c;
  if (v.has("round")) c.round = v["round"].num();
  if (v.has("diagonal")) c.diagonal = v["diagonal"].num();
  return c;
}

template <class V, class R>
Keys<V> keysIn(const Value& v, R read) {
  Keys<V> ks;
  for (auto& e : v.arr()) {
    Key<V> k{e[0].num(), read(e[1])};
    if (e.size() > 2) {
      Handles h;
      if (e[2].has("in")) h.in = pt(e[2]["in"]);
      if (e[2].has("out")) h.out = pt(e[2]["out"]);
      k.h = h;
    }
    ks.push_back(k);
  }
  return ks;
}
inline Keys<double> keysD(const Value& v) { return keysIn<double>(v, [](const Value& x) { return x.num(); }); }
inline Keys<Shape> keysS(const Value& v) { return keysIn<Shape>(v, shapeIn); }
inline std::optional<Keys<double>> optKeysD(const Value& v) { return v.isNull() ? std::nullopt : std::optional(keysD(v)); }

template <class V, class W>
Value outKeys(const Keys<V>& ks, W write) {
  json::Array a;
  for (auto& k : ks) {
    json::Array e{out(k.t), write(k.v)};
    if (k.h) {
      json::Object h;
      if (k.h->in) h["in"] = out(*k.h->in);
      if (k.h->out) h["out"] = out(*k.h->out);
      e.push_back(h);
    }
    a.push_back(e);
  }
  return a;
}
inline Value outKeysD(const Keys<double>& ks) { return outKeys(ks, [](double d) { return out(d); }); }
inline Value outKeysS(const Keys<Shape>& ks) { return outKeys(ks, outShape); }
inline Value outOptKeysD(const std::optional<Keys<double>>& ks) { return ks ? outKeysD(*ks) : Value(); }

// The ink.

inline InkEnd endIn(const Value& v) { return v.str() == "start" ? InkEnd::start : InkEnd::end; }
inline Value outEnd(InkEnd e) { return e == InkEnd::start ? "start" : "end"; }

inline Cut cutIn(const Value& v) {
  Cut c;
  const std::string& m = v["mode"].str();
  c.mode = m == "flat" ? CutMode::flat : m == "nib" ? CutMode::nib : m == "round" ? CutMode::round
         : m == "level" ? CutMode::level : m == "angle" ? CutMode::angle : CutMode::auto_;
  if (v.has("deg")) c.deg = v["deg"].num();
  if (v.has("taper")) c.taper = v["taper"].num();
  if (v.has("reach")) c.reach = v["reach"].num();
  return c;
}
inline InkStroke strokeIn(const Value& v) {
  InkStroke st;
  if (v.has("S")) st.S = pt(v["S"]);
  if (v.has("E")) st.E = pt(v["E"]);
  for (auto& c : v["segments"].arr()) st.segments.push_back(cub(c));
  if (v.has("closed")) st.closed = flag(v["closed"]);
  if (v.has("weight")) st.weight = v["weight"].num();
  if (v.has("cuts")) {
    Cuts cs;
    if (v["cuts"].has("start")) cs.start = cutIn(v["cuts"]["start"]);
    if (v["cuts"].has("end")) cs.end = cutIn(v["cuts"]["end"]);
    st.cuts = cs;
  }
  if (v.has("size")) st.size = keysD(v["size"]);
  if (v.has("rotate")) st.rotate = keysD(v["rotate"]);
  if (v.has("follow")) st.follow = keysD(v["follow"]);
  if (v.has("offset")) st.offset = keysD(v["offset"]);
  if (v.has("shapes")) st.shapes = keysS(v["shapes"]);
  return st;
}
inline InkOptions inkOptsIn(const Value& v) {
  InkOptions o;
  if (!v.isObject()) return o;
  if (v.has("caps")) o.caps = v["caps"].str() == "nib" ? Caps::nib : Caps::butt;
  if (v.has("cuts")) o.cuts = flag(v["cuts"]);
  if (v.has("levels")) o.levels = list<double>(v["levels"], [](const Value& e) { return e.num(); });
  if (v.has("perSpan")) o.perSpan = v["perSpan"].num();
  if (v.has("spacing")) o.spacing = v["spacing"].num();
  if (v.has("model")) o.model = v["model"].str() == "pointed" ? Model::pointed : Model::broad;
  return o;
}

inline Value outInk(const InkResult& r) {
  json::Array planes, stamps, frames, polys, heads;
  for (auto& p : r.planes) planes.push_back(json::Object{{"at", out(p.at)}, {"n", out(p.n)}, {"end", outEnd(p.end)}});
  for (auto& s : r.stamps) stamps.push_back(out(s));
  for (auto& f : r.frames) frames.push_back(outFrame(f));
  for (auto& p : r.polys) polys.push_back(out(p));
  for (auto& h : r.heads) heads.push_back(out(h));
  return json::Object{{"segments", out(r.segments)}, {"weight", out(r.weight)}, {"planes", planes}, {"pts", out(r.pts)},
                      {"ts", out(r.ts)}, {"widths", out(r.widths)}, {"stamps", stamps}, {"frames", frames}, {"polys", polys},
                      {"nodes", out(r.nodes)}, {"heads", heads}, {"caps", r.caps == Caps::nib ? "nib" : "butt"},
                      {"closed", out(r.closed)}};
}

// The outline.

inline OutlinePlan planIn(const Value& v) {
  OutlinePlan p;
  p.left = list<int>(v["left"], integer);
  p.right = list<int>(v["right"], integer);
  if (v["start"].isNull() || std::holds_alternative<double>(v["start"].v)) { if (!v["start"].isNull()) p.start = integer(v["start"]); }
  if (v["end"].isNull() || std::holds_alternative<double>(v["end"].v)) { if (!v["end"].isNull()) p.end = integer(v["end"]); }
  if (v.has("runs")) p.runs = Runs{v["runs"]["start"].str(), v["runs"]["end"].str()};
  return p;
}
inline Value outPlan(const OutlinePlan& p) {
  json::Object o{{"left", out(p.left)}, {"right", out(p.right)}, {"start", p.start ? out(*p.start) : Value("line")},
                 {"end", p.end ? out(*p.end) : Value("line")}};
  if (p.runs) o["runs"] = json::Object{{"start", p.runs->start}, {"end", p.runs->end}};
  return o;
}
inline Value outContour(const Contour& c) { return out(c); }
inline Value outOutline(const OutlineResult& r) {
  json::Array contours;
  for (auto& c : r.contours) contours.push_back(out(c));
  return json::Object{{"contour", out(r.contour)}, {"contours", contours}, {"plan", outPlan(r.plan)}, {"anchors", out(r.anchors)},
                      {"err", out(r.err)},
                      {"errs", json::Object{{"left", out(r.errs.left)}, {"right", out(r.errs.right)}, {"start", out(r.errs.start)},
                                            {"end", out(r.errs.end)}}}};
}

}  // namespace golden
