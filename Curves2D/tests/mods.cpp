// The golden cases of mods.ts. Selections are named by data (see tests/golden/cases/mods.ts).
#include "cases.hpp"
#include "paths.hpp"

#include "curves/mods.hpp"

namespace {

using namespace golden;

Values valuesOf(const Value& v) {
  Values out;
  if (v.isNull()) return out;
  for (auto& [k, x] : v.obj()) out[k] = std::holds_alternative<std::string>(x.v) ? ParamValue(x.str()) : ParamValue(x.num());
  return out;
}

Selection sel(const Value& path, const Value& nodes) {
  if (nodes.isNull()) return Selection::of(P(path));
  return Selection::of(P(path), nodeAt(list(nodes, integer)));
}

Value outSel(const Selection& s) {
  json::Array paths;
  for (auto& p : s.paths()) paths.push_back(ser(p));
  return json::Object{{"paths", paths}, {"weights", out(s.weights())}};
}

std::vector<Step> stepsOf(const Value& v) {
  std::vector<Step> out;
  for (auto& s : v.arr()) out.push_back({s["mod"].str(), s.has("values") ? valuesOf(s["values"]) : Values{}});
  return out;
}

Value param(const Param& p) {
  if (auto* n = std::get_if<NumParam>(&p))
    return json::Object{{"min", n->min}, {"max", n->max}, {"step", n->step}, {"value", n->value}};
  auto& c = std::get<ChoiceParam>(p);
  json::Array options;
  for (auto& o : c.options) options.push_back(o);
  return json::Object{{"options", options}, {"value", c.value}};
}

Value valuesOut(const Values& v) {
  json::Object o;
  for (auto& [k, x] : v) o[k] = std::holds_alternative<double>(x) ? Value(std::get<double>(x)) : Value(std::get<std::string>(x));
  return o;
}

static Register reg("mods.json", {
  {"all", [](const Value&) {
     json::Array r;
     for (auto& m : mods()) {
       json::Array params;
       for (auto& [k, p] : m.params) params.push_back(json::Array{k, param(p)});
       r.push_back(json::Object{{"name", m.name}, {"group", m.group}, {"doc", m.doc}, {"params", params}});
     }
     return Value(r);
   }},
  {"modDefaults", [](const Value& a) { return valuesOut(modDefaults(mod(a[0].str()))); }},
  {"modCode", [](const Value& a) { return Value(modCode(mod(a[0].str()), a.size() > 1 ? valuesOf(a[1]) : Values{})); }},
  {"applyMod", [](const Value& a) {
     return outSel(applyMod(sel(a[0], a[1]), mod(a[2].str()), a.size() > 3 ? valuesOf(a[3]) : Values{}));
   }},
  {"chain", [](const Value& a) { return outSel(chain(sel(a[0], a[1]), stepsOf(a[2]))); }},
  {"chainCode", [](const Value& a) { return Value(chainCode(stepsOf(a[0]))); }},
});

}  // namespace
