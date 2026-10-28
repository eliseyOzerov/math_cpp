// The golden cases of outline.ts: each TS function by name, with its arguments read and its result written as gen.ts
// does. The cases give a stroke, a nib and options and the wrappers ink them first (inkOutline, inkPlan,
// sharedOutlines), as the case file does.
#include "inkio.hpp"

namespace {

using namespace golden;

InkResult inked(const Value& a) { return ink(strokeIn(a[0]), nibArg(a, 1), a.size() > 2 ? inkOptsIn(a[2]) : InkOptions{}); }

std::vector<OutlinePlan> plansIn(const Value& v) { return list(v, planIn); }

golden::Register reg("outline.json", {
  {"inkOutline", [](auto& a) {
     std::optional<OutlinePlan> plan;
     double tolerance = 1;
     if (a.size() > 3 && a[3].isObject()) {
       if (a[3].has("plan")) plan = planIn(a[3]["plan"]);
       if (a[3].has("tolerance")) tolerance = a[3]["tolerance"].num();
     }
     return outOutline(outlineInk(inked(a), plan, tolerance));
   }},
  {"inkPlan", [](auto& a) { return outPlan(outlinePlan(inked(a), opt(a, 3).value_or(1))); }},
  {"sharedOutlines", [](auto& a) {
     InkStroke st = strokeIn(a[0]);
     InkOptions o = inkOptsIn(a[2]);
     std::vector<InkResult> inks;
     std::vector<OutlinePlan> ps;
     for (auto& n : a[1].arr()) { inks.push_back(ink(st, nibIn(n), o)); ps.push_back(outlinePlan(inks.back())); }
     auto plan = sharedPlan(ps);
     if (!plan) return Value();
     json::Array outlines;
     for (auto& i : inks) outlines.push_back(outOutline(outlineInk(i, plan)));
     return Value(json::Object{{"plan", outPlan(*plan)}, {"outlines", outlines}});
   }},
  {"sharedPlan", [](auto& a) {
     auto p = sharedPlan(plansIn(a[0]));
     return p ? outPlan(*p) : Value();
   }},
  {"planKey", [](auto& a) { return out(planKey(planIn(a[0]))); }},
});

}  // namespace
