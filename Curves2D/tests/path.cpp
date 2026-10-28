// The golden cases of path.ts. Paths are named by a spec and written back whole (see tests/golden/cases/path.ts).
#include "cases.hpp"

#include "curves/path.hpp"

namespace {

using namespace golden;

bool isStr(const Value& v) { return std::holds_alternative<std::string>(v.v); }

NodeKind kindOf(const std::string& s) { return s == "corner" ? NodeKind::corner : s == "smooth" ? NodeKind::smooth : NodeKind::symmetric; }
const char* name(NodeKind k) { return k == NodeKind::corner ? "corner" : k == NodeKind::smooth ? "smooth" : "symmetric"; }
std::optional<Heading> heading(const Value& v) {
  if (v.isNull()) return std::nullopt;
  return v.str() == "h" ? Heading::h : Heading::v;
}

Path P(const Value& s) {
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (s.has("kinds")) {
    kinds.emplace();
    for (auto& k : s["kinds"].arr()) kinds->push_back(k.isNull() ? std::nullopt : std::optional(kindOf(k.str())));
  }
  return Path(pt(s["start"]), list(s["segs"], cub), s.has("closed") && s["closed"].boolean(),
              s.has("arrives") ? heading(s["arrives"]) : std::nullopt, s.has("metric") ? pt(s["metric"]) : Pt{1, 1}, kinds);
}

Value ser(const Path& p) {
  json::Array kinds;
  for (auto k : p.kinds()) kinds.push_back(name(k));
  json::Array segs;
  for (auto& c : p.segs()) segs.push_back(out(c));
  return json::Object{{"start", out(p.start())}, {"segs", segs}, {"closed", p.closed()},
                      {"arrives", p.arrives() ? Value(*p.arrives() == Heading::h ? "h" : "v") : Value()},
                      {"metric", out(p.metric())}, {"kinds", kinds}};
}

Which which(const Value& a, size_t i) {
  if (i >= a.size() || a[i].isNull()) return End::first;
  if (isStr(a[i])) return a[i].str() == "last" ? End::last : End::first;
  return integer(a[i]);
}

Segs segs(const Value& v) { return v.isArray() ? Segs(integer(v[0]), integer(v[1])) : Segs(integer(v)); }

std::function<Pt(Pt)> affine(const Value& v) {
  double k[6];
  for (int i = 0; i < 6; i++) k[i] = v[i].num();
  return [=](Pt p) -> Pt { return {k[0] * p[0] + k[2] * p[1] + k[4], k[1] * p[0] + k[3] * p[1] + k[5]}; };
}

Path draw(Pt from, const Value& steps) {
  Path p = curves::from(from);
  for (auto& s : steps.arr()) {
    const std::string& k = s[0].str();
    auto at = [&](size_t i) { return i < s.size() && !s[i].isNull(); };
    if (k == "line") p = p.line(pt(s[1]));
    else if (k == "curve") p = p.curve(pt(s[1]), pt(s[2]), pt(s[3]));
    else if (k == "quarter") {
      QuarterOptions o;
      if (at(2)) o.from = heading(s[2]);
      if (at(3)) o.tension = s[3].num();
      p = p.quarter(pt(s[1]), o);
    } else if (k == "arch") p = p.arch(pt(s[1]), s[2].num(), at(3) ? s[3].num() : KAPPA);
    else if (k == "join") p = p.join(P(s[1]));
    else p = p.close();
  }
  return p;
}

template <class F>
Value each(const Value& ps, F f) {
  json::Array a;
  for (auto& p : ps.arr()) a.push_back(f(p.num()));
  return a;
}

static Register reg("path.json", {
  {"path", [](const Value& a) { return ser(P(a[0])); }},
  {"of", [](const Value& a) {
     return ser(Path::of(list(a[0], cub), a.size() > 1 && a[1].boolean(), a.size() > 2 ? pt(a[2]) : Pt{1, 1}));
   }},
  {"measured", [](const Value& a) { return ser(P(a[0]).measured(pt(a[1]))); }},
  {"reshaped", [](const Value& a) { return ser(P(a[0]).reshaped(list(a[1], cub))); }},
  {"handle", [](const Value& a) {
     return ser(P(a[0]).handle(integer(a[1]), a[2].str() == "in" ? HandleSide::in : HandleSide::out, pt(a[3])));
   }},
  {"length", [](const Value& a) { return out(P(a[0]).length()); }},
  {"locate", [](const Value& a) {
     Path p = P(a[0]);
     return each(a[1], [&](double x) { auto l = p.locate(x); return Value(json::Object{{"i", out(l.i)}, {"t", out(l.t)}}); });
   }},
  {"progress", [](const Value& a) {
     Path p = P(a[0]);
     int i = integer(a[1]);
     return each(a[2], [&](double t) { return out(p.progress(i, t)); });
   }},
  {"end", [](const Value& a) { return out(P(a[0]).end()); }},
  {"draw", [](const Value& a) { return ser(draw(pt(a[0]), a[1])); }},
  {"at", [](const Value& a) { Path p = P(a[0]); return each(a[1], [&](double x) { return out(p.at(x)); }); }},
  {"crossY", [](const Value& a) { return out(P(a[0]).crossY(a[1].num())); }},
  {"crossX", [](const Value& a) { return out(P(a[0]).crossX(a[1].num())); }},
  {"atY", [](const Value& a) { return out(P(a[0]).atY(a[1].num(), which(a, 2))); }},
  {"atX", [](const Value& a) { return out(P(a[0]).atX(a[1].num(), which(a, 2))); }},
  {"tAtY", [](const Value& a) { return out(P(a[0]).tAtY(a[1].num(), which(a, 2))); }},
  {"tAtX", [](const Value& a) { return out(P(a[0]).tAtX(a[1].num(), which(a, 2))); }},
  {"chordY", [](const Value& a) { return ser(P(a[0]).chordY(a[1].num())); }},
  {"chordX", [](const Value& a) { return ser(P(a[0]).chordX(a[1].num())); }},
  {"slice", [](const Value& a) { return ser(P(a[0]).slice(a[1].num(), a[2].num())); }},
  {"aperture", [](const Value& a) { return ser(P(a[0]).aperture(a[1].num(), opt(a, 2).value_or(0))); }},
  {"divide", [](const Value& a) { return ser(P(a[0]).divide(integer(a[1]))); }},
  {"direction", [](const Value& a) { Path p = P(a[0]); return each(a[1], [&](double x) { return out(p.direction(x)); }); }},
  {"crossings", [](const Value& a) {
     json::Array r;
     for (auto& c : P(a[0]).crossings(P(a[1]))) r.push_back(json::Object{{"p", out(c.p)}, {"t", out(c.t)}, {"u", out(c.u)}});
     return Value(r);
   }},
  {"cut", [](const Value& a) { return ser(P(a[0]).cut(list(a[1], num))); }},
  {"angle", [](const Value& a) { return out(P(a[0]).angle()); }},
  {"nodeCount", [](const Value& a) { return out(P(a[0]).nodeCount()); }},
  {"node", [](const Value& a) { return out(P(a[0]).node(integer(a[1]))); }},
  {"within", [](const Value& a) { return out(P(a[0]).within(segs(a[1]), a[2].num())); }},
  {"nodeProgress", [](const Value& a) {
     Path p = P(a[0]);
     json::Array r;
     for (int i = 0; i <= p.nodeCount(); i++) r.push_back(out(p.nodeProgress(i)));
     return Value(r);
   }},
  {"nodes", [](const Value& a) { return out(P(a[0]).nodes()); }},
  {"distances", [](const Value& a) { return out(P(a[0]).distances()); }},
  {"reverse", [](const Value& a) { return ser(P(a[0]).reverse()); }},
  {"startAt", [](const Value& a) { return ser(P(a[0]).startAt(integer(a[1]))); }},
  {"map", [](const Value& a) { return ser(P(a[0]).map(affine(a[1]))); }},
  {"warp", [](const Value& a) { return ser(P(a[0]).warp(affine(a[1]))); }},
  {"line", [](const Value& a) { return ser(curves::line(pt(a[0]), pt(a[1]))); }},
  {"poly", [](const Value& a) { return ser(poly(pt(a[0]), list(a[1], pt))); }},
});

}  // namespace
