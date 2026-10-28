// The golden cases of bezn.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"
#include "curves/bezn.hpp"

namespace {

using namespace golden;

std::vector<Pt> pts(const Value& v) { return list(v, pt); }
std::vector<double> nums(const Value& v) { return list<double>(v, num); }
std::vector<std::vector<double>> matrix(const Value& v) {
  std::vector<std::vector<double>> m;
  for (auto& r : v.arr()) m.push_back(nums(r));
  return m;
}

golden::Register reg("bezn.json", {
  {"bezierAt", [](auto& a) { return out(bezierAt(pts(a[0]), num(a[1]))); }},
  {"casteljauN", [](auto& a) { return out(casteljauN(pts(a[0]), num(a[1]))); }},
  {"splitN", [](auto& a) { 
     auto h = splitN(pts(a[0]), num(a[1]));
     return Value(json::Array{out(h[0]), out(h[1])});
   }},
  {"derivativeN", [](auto& a) { return out(derivativeN(pts(a[0]))); }},
  {"raise", [](auto& a) { return out(raise(pts(a[0]))); }},
  {"lower", [](auto& a) { return out(lower(pts(a[0]))); }},
  {"fitBezier", [](auto& a) {
     auto order = opt(a, 1);
     return out(order ? fitBezier(pts(a[0]), int(*order)) : fitBezier(pts(a[0])));
   }},
  {"rationalAt", [](auto& a) { return out(rationalAt(Rational{pts(a[0]), nums(a[1])}, num(a[2]))); }},
  {"toCubics", [](auto& a) {
     ToCubicsOptions o;
     if (a.size() > 1) {
       if (a[1].has("weights")) o.weights = nums(a[1]["weights"]);
       if (auto t = optKey(a, 1, "tol")) o.tol = *t;
       if (auto s = optKey(a, 1, "samples")) o.samples = int(*s);
     }
     return out(toCubics(pts(a[0]), o));
   }},
  {"bernstein", [](auto& a) { return out(bernstein(integer(a[0]), num(a[1]))); }},
  {"solve", [](auto& a) { return out(solve(matrix(a[0]), nums(a[1]))); }},
});

}  // namespace
