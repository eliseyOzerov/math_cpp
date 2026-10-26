// The golden cases of boolean.ts. Rules are passed by name (cases/boolean.ts has the same table), since the TS takes
// a function.
#include "cases.hpp"

#include "curves/boolean.hpp"

namespace {

using namespace golden;

const std::map<std::string, Rule> rules{
  {"union", [](auto& w) { return std::any_of(w.begin(), w.end(), [](int n) { return n != 0; }); }},
  {"intersect", [](auto& w) { return std::all_of(w.begin(), w.end(), [](int n) { return n != 0; }); }},
  {"difference", [](auto& w) { return w[0] != 0 && w[1] == 0; }},
  {"xor", [](auto& w) { return std::count_if(w.begin(), w.end(), [](int n) { return n != 0; }) == 1; }},
  {"nonzero", [](auto& w) { return w[0] != 0; }},
};
std::vector<std::vector<Contour>> groups(const Value& v) { return list(v, contours); }
std::vector<Pt> poly(const Value& v) { return list(v, pt); }
Edge edge(const Value& v) { return {pt(v[0]), pt(v[1])}; }
Value out(const Gap& g) { return json::Object{{"d", golden::out(g.d)}, {"a", golden::out(g.a)}, {"b", golden::out(g.b)}}; }
Value out(const std::optional<Gap>& g) { return g ? out(*g) : Value(); }
double eps(const Value& a) { return optKey(a, 2, "eps").value_or(0.25); }

golden::Register reg("boolean.json", {
  {"crossings", [](auto& a) {
     std::vector<SharedEnd> shared;
     if (a.size() > 2 && a[2].has("shared"))
       for (auto& s : a[2]["shared"].arr()) shared.push_back({integer(s[0]), integer(s[1])});
     return golden::out(crossings(cub(a[0]), cub(a[1]), optKey(a, 2, "tol").value_or(1e-3), shared));
   }},
  {"winding", [](auto& a) { return golden::out(winding(contours(a[0]), pt(a[1]))); }},
  {"booleanBy", [](auto& a) { return golden::out(boolean(groups(a[1]), rules.at(a[0].str()), eps(a))); }},
  {"tryBooleanBy", [](auto& a) { return golden::out(tryBoolean(groups(a[1]), rules.at(a[0].str()))); }},
  {"traceBooleanBy", [](auto& a) {
     auto t = traceBoolean(groups(a[1]), rules.at(a[0].str()));
     return t ? Value(json::Object{{"contours", golden::out(t->contours)}, {"from", golden::out(t->from)}}) : Value();
   }},
  {"union", [](auto& a) { return golden::out(unite(contours(a[0]))); }},
  {"contourArea", [](auto& a) { return golden::out(contourArea(contour(a[0]))); }},
  {"reverseContour", [](auto& a) { return golden::out(reverseContour(contour(a[0]))); }},
  {"orient", [](auto& a) { return golden::out(orient(contours(a[0]))); }},
  {"flatten", [](auto& a) { return golden::out(flatten(contours(a[0]), opt(a, 1).value_or(2), opt(a, 2).value_or(8))); }},
  {"edges", [](auto& a) { return golden::out(edges(list(a[0], poly))); }},
  {"scanline", [](auto& a) { return golden::out(scanline(list(a[0], edge), num(a[1]))); }},
  {"narrowest", [](auto& a) { return out(narrowest(poly(a[0]), num(a[1]))); }},
  {"polyGap", [](auto& a) { return out(polyGap(poly(a[0]), poly(a[1]))); }},
});

}  // namespace
