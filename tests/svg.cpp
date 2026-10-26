// The golden cases of svg.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "cases.hpp"
#include "curves/svg.hpp"

namespace {

using namespace golden;

golden::Register reg("svg.json", {
  {"parseSvgPath", [](auto& a) { return out(parseSvgPath(a[0].str())); }},
  {"svgPath", [](auto& a) {
     auto digits = opt(a, 1);
     return out(digits ? svgPath(contours(a[0]), int(*digits)) : svgPath(contours(a[0])));
   }},
});

}  // namespace
