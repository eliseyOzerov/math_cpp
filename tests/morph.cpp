// The golden cases of morph.ts.
#include "cases.hpp"
#include "paths.hpp"

#include "curves/morph.hpp"

namespace {

using namespace golden;

static Register reg("morph.json", {
  {"alike", [](const Value& a) {
     Alike r = alike(P(a[0]), P(a[1]), opt(a, 2).value_or(1e-6));
     json::Array matched;
     for (bool b : r.matched) matched.push_back(b);
     return Value(json::Object{{"from", ser(r.from)}, {"to", ser(r.to)}, {"matched", matched}});
   }},
  {"morph", [](const Value& a) {
     Path p = P(a[0]), q = P(a[1]);
     json::Array r;
     for (double k : list(a[2], num)) r.push_back(ser(morph(p, q, k, opt(a, 3).value_or(1e-6))));
     return Value(r);
   }},
  {"between", [](const Value& a) { return ser(between(P(a[0]), P(a[1]), a[2].num(), opt(a, 3).value_or(1e-6))); }},
});

}  // namespace
