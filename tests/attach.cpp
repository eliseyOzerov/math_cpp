// The golden cases of attach.ts: each TS function by name. A path comes in as {start, segs, closed, metric?} and goes
// out as {start, segs, closed, kinds}; mounts, targets, pairs and aligns are objects holding such paths; a Segs is a
// number or [a, b].
#include "cases.hpp"
#include "curves/attach.hpp"

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

std::optional<double> key(const Value& v, const char* k) {
  return v.has(k) ? std::optional(v[k].num()) : std::nullopt;
}

std::optional<Segs> segsOf(const Value& v, const char* k) {
  if (!v.has(k)) return std::nullopt;
  const Value& s = v[k];
  if (s.isArray()) return Segs(integer(s[0]), integer(s[1]));
  return Segs(integer(s));
}

Mount mountOf(const Value& v) { return Mount{pathOf(v["on"]), num(v["at"]), key(v, "from"), segsOf(v, "segs")}; }

Pair pairOf(const Value& v) {
  std::optional<Progress> from = key(v, "from");
  if (v.has("point")) return {TargetPoint{pt(v["point"])}, from};
  if (v.has("cross")) {
    std::optional<Which> which;
    if (v.has("which")) {
      if (std::holds_alternative<std::string>(v["which"].v)) which = v["which"].str() == "first" ? End::first : End::last;
      else which = integer(v["which"]);
    }
    return {TargetCross{pathOf(v["cross"]), pathOf(v["with"]), which}, from};
  }
  return {TargetOn{pathOf(v["on"]), num(v["at"]), segsOf(v, "segs")}, from};
}

golden::Register reg("attach.json", {
  {"attach", [](auto& a) {
     std::optional<Mount> b;
     if (a.size() > 2 && !a[2].isNull()) b = mountOf(a[2]);
     return outPath(attach(pathOf(a[0]), mountOf(a[1]), b));
   }},
  {"target", [](auto& a) { return out(target(pairOf(a[0]).target)); }},
  {"place", [](auto& a) {
     std::vector<Pair> pairs;
     for (auto& q : a[1].arr()) pairs.push_back(pairOf(q));
     std::optional<Align> al;
     if (a.size() > 2 && !a[2].isNull()) {
       const Value& v = a[2];
       Align x{pathOf(v["on"]), num(v["at"]), segsOf(v, "segs"), AlignTo::tangent, {}, {}};
       if (v.has("to") && v["to"].str() == "normal") x.to = AlignTo::normal;
       x.angle = key(v, "angle");
       x.from = key(v, "from");
       al = x;
     }
     return outPath(place(pathOf(a[0]), pairs, al));
   }},
  {"pathAttach", [](auto& a) {
     Path p = pathOf(a[0]), on = pathOf(a[1]);
     double at = num(a[2]);
     if (a.size() < 4 || a[3].isNull()) return outPath(p.attach(on, at));
     const Value& v = a[3];
     AttachOptions o;
     o.from = key(v, "from");
     o.segs = segsOf(v, "segs");
     if (v.has("end")) {
       const Value& e = v["end"];
       AttachEnd end;
       if (std::holds_alternative<double>(e.v)) end.at = e.num();
       else {
         if (e.has("on")) end.on = pathOf(e["on"]);
         end.at = num(e["at"]);
         end.from = key(e, "from");
         end.segs = segsOf(e, "segs");
       }
       o.end = end;
     }
     return outPath(p.attach(on, at, o));
   }},
});

}  // namespace
