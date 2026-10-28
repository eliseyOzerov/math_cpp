// The golden cases of intersect.ts.
#include "cases.hpp"

#include "curves/intersect.hpp"

namespace {

using namespace golden;
using golden::out;

Value out(const LineCrossing& h) { return json::Object{{"t", out(h.t)}, {"s", out(h.s)}, {"p", out(h.p)}}; }
Value out(const CircleCrossing& h) { return json::Object{{"t", out(h.t)}, {"p", out(h.p)}}; }
Value out(const SegAt& a) { return json::Object{{"i", out(a.i)}, {"t", out(a.t)}}; }
Value out(const SelfCrossing& h) { return json::Object{{"a", out(h.a)}, {"b", out(h.b)}, {"p", out(h.p)}}; }
template <class T>
Value outs(const std::vector<T>& xs) { json::Array a; for (auto& x : xs) a.push_back(out(x)); return a; }
SharedEnd sharedEnd(const Value& v) { return {int(v[0].num()), int(v[1].num())}; }

golden::Register reg("intersect.json", {
  {"lineCrossings", [](auto& a) {
     bool segment = !(a.size() > 3 && a[3].has("segment")) || a[3]["segment"].boolean();
     return outs(lineCrossings(cub(a[0]), pt(a[1]), pt(a[2]), segment));
   }},
  {"circleCrossings", [](auto& a) { return outs(circleCrossings(cub(a[0]), pt(a[1]), num(a[2]), int(opt(a, 3).value_or(64)))); }},
  {"curveCrossings", [](auto& a) {
     auto shared = a.size() > 2 && a[2].has("shared") ? list(a[2]["shared"], sharedEnd) : std::vector<SharedEnd>{};
     return golden::out(curveCrossings(cub(a[0]), cub(a[1]), optKey(a, 2, "tol").value_or(1e-3), shared));
   }},
  {"selfCrossings", [](auto& a) {
     return outs(selfCrossings(list(a[0], cub), a.size() > 1 && !a[1].isNull() && a[1].boolean()));
   }},
});

}  // namespace
