// Paths in cases, as tests/golden/cases/paths.ts names and writes them: P reads a spec, ser writes a path whole.
#pragma once

#include "cases.hpp"

#include "curves/path.hpp"

namespace golden {

inline curves::NodeKind kindOf(const std::string& s) {
  using curves::NodeKind;
  return s == "corner" ? NodeKind::corner : s == "smooth" ? NodeKind::smooth : NodeKind::symmetric;
}
inline const char* kindName(curves::NodeKind k) {
  using curves::NodeKind;
  return k == NodeKind::corner ? "corner" : k == NodeKind::smooth ? "smooth" : "symmetric";
}

inline curves::Path P(const json::Value& s) {
  using namespace curves;
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (s.has("kinds")) {
    kinds.emplace();
    for (auto& k : s["kinds"].arr()) kinds->push_back(k.isNull() ? std::nullopt : std::optional(kindOf(k.str())));
  }
  std::optional<Heading> arrives;
  if (s.has("arrives")) arrives = s["arrives"].str() == "h" ? Heading::h : Heading::v;
  return Path(pt(s["start"]), list(s["segs"], cub), s.has("closed") && s["closed"].boolean(), arrives,
              s.has("metric") ? pt(s["metric"]) : Pt{1, 1}, kinds);
}

inline json::Value ser(const curves::Path& p) {
  json::Array kinds;
  for (auto k : p.kinds()) kinds.push_back(kindName(k));
  json::Array segs;
  for (auto& c : p.segs()) segs.push_back(out(c));
  return json::Object{{"start", out(p.start())}, {"segs", segs}, {"closed", p.closed()},
                      {"arrives", p.arrives() ? json::Value(*p.arrives() == curves::Heading::h ? "h" : "v") : json::Value()},
                      {"metric", out(p.metric())}, {"kinds", kinds}};
}

}  // namespace golden
