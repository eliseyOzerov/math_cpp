// The golden cases of bool.ts. Bools are named by data (see tests/golden/cases/bool.ts).
#include "cases.hpp"
#include "paths.hpp"

#include "curves/bool.hpp"

namespace {

using namespace golden;

Op opOf(const std::string& s) {
  return s == "union" ? Op::union_ : s == "intersect" ? Op::intersect : s == "subtract" ? Op::subtract : s == "exclude" ? Op::exclude : Op::cut;
}

Bool boolOf(const Value& b);

Operand operand(const Value& o) {
  if (o.has("contours")) return contours(o["contours"]);
  if (o.has("path")) return P(o["path"]);
  return std::make_shared<const Bool>(boolOf(o["bool"]));
}

std::vector<Operand> operands(const Value& v) {
  std::vector<Operand> out;
  for (auto& o : v.arr()) out.push_back(operand(o));
  return out;
}

Bool boolOf(const Value& b) { return Bool(opOf(b["op"].str()), operands(b["operands"]), b.has("eps") ? b["eps"].num() : 0.25); }

Value corner(const BoolCorner& c) {
  return json::Object{{"contour", out(c.contour)}, {"node", out(c.node)}, {"p", out(c.p)},
                      {"between", json::Array{out(c.between[0]), out(c.between[1])}}, {"in", out(c.in)},
                      {"out", out(c.out)}, {"outward", out(c.outward)}, {"convex", c.convex}};
}

std::optional<int> optInt(const Value& a, size_t i) { return opt(a, i) ? std::optional(int(*opt(a, i))) : std::nullopt; }

static Register reg("bool.json", {
  {"bool", [](const Value& a) {
     Bool x = boolOf(a[0]);
     auto cs = x.contours();
     json::Array outward;
     for (int k = 0; k < int(cs.size()); k++) {
       json::Array row;
       for (int j = 0; j < int(cs[k].size()); j++) row.push_back(out(x.outward(k, j)));
       outward.push_back(row);
     }
     return Value(json::Object{{"ok", x.ok()}, {"contours", out(cs)}, {"sources", out(x.sources())}, {"outward", outward}});
   }},
  {"corners", [](const Value& a) {
     BoolCornersOptions o;
     if (a.size() > 3 && !a[3].isNull()) o.same = a[3].boolean();
     json::Array r;
     for (auto& c : boolOf(a[0]).corners(optInt(a, 1), optInt(a, 2), o)) r.push_back(corner(c));
     return Value(r);
   }},
  {"with", [](const Value& a) { return out(boolOf(a[0]).with(operands(a[1])).contours()); }},
});

}  // namespace
