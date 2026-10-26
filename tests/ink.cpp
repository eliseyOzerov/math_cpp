// The golden cases of ink.ts: each TS function by name, with its arguments read and its result written as gen.ts does.
#include "inkio.hpp"

namespace {

using namespace golden;

golden::Register reg("ink.json", {
  {"ink", [](auto& a) {
     return outInk(ink(strokeIn(a[0]), nibArg(a, 1), a.size() > 2 ? inkOptsIn(a[2]) : InkOptions{}));
   }},
});

}  // namespace
