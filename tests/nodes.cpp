// The golden cases of nodes.ts. Paths are named by a spec and written back whole (see tests/golden/cases/path.ts);
// nodes are { p, in?, out?, kind? }; callbacks come as data (see tests/golden/cases/nodes.ts).
#include "cases.hpp"

#include "curves/nodes.hpp"

namespace {

using namespace golden;

NodeKind kindOf(const std::string& s) { return s == "corner" ? NodeKind::corner : s == "smooth" ? NodeKind::smooth : NodeKind::symmetric; }
const char* name(NodeKind k) { return k == NodeKind::corner ? "corner" : k == NodeKind::smooth ? "smooth" : "symmetric"; }
std::optional<Heading> heading(const Value& v) {
  if (v.isNull()) return std::nullopt;
  return v.str() == "h" ? Heading::h : Heading::v;
}

Path P(const Value& s) {
  std::optional<std::vector<std::optional<NodeKind>>> kinds;
  if (s.has("kinds")) {
    kinds.emplace();
    for (auto& k : s["kinds"].arr()) kinds->push_back(k.isNull() ? std::nullopt : std::optional(kindOf(k.str())));
  }
  return Path(pt(s["start"]), list(s["segs"], cub), s.has("closed") && s["closed"].boolean(),
              s.has("arrives") ? heading(s["arrives"]) : std::nullopt, s.has("metric") ? pt(s["metric"]) : Pt{1, 1}, kinds);
}

Value ser(const Path& p) {
  json::Array kinds;
  for (auto k : p.kinds()) kinds.push_back(name(k));
  json::Array segs;
  for (auto& c : p.segs()) segs.push_back(out(c));
  return json::Object{{"start", out(p.start())}, {"segs", segs}, {"closed", p.closed()},
                      {"arrives", p.arrives() ? Value(*p.arrives() == Heading::h ? "h" : "v") : Value()},
                      {"metric", out(p.metric())}, {"kinds", kinds}};
}

Node node(const Value& v) {
  Node n{pt(v["p"]), std::nullopt, std::nullopt, std::nullopt};
  if (v.has("in")) n.in = pt(v["in"]);
  if (v.has("out")) n.out = pt(v["out"]);
  if (v.has("kind")) n.kind = kindOf(v["kind"].str());
  return n;
}
std::vector<Node> nodes(const Value& v) { return list(v, node); }
std::vector<std::vector<Node>> lists(const Value& v) { return list(v, nodes); }

Value outNode(const Node& n) {
  json::Object o{{"p", out(n.p)}};
  if (n.in) o["in"] = out(*n.in);
  if (n.out) o["out"] = out(*n.out);
  if (n.kind) o["kind"] = name(*n.kind);
  return o;
}
Value outNodes(const std::vector<Node>& ns) {
  json::Array a;
  for (auto& n : ns) a.push_back(outNode(n));
  return a;
}
Value outLists(const std::vector<std::vector<Node>>& ls) {
  json::Array a;
  for (auto& l : ls) a.push_back(outNodes(l));
  return a;
}

HandleSide side(const Value& v) { return v.str() == "in" ? HandleSide::in : HandleSide::out; }
const char* sideName(HandleSide s) { return s == HandleSide::in ? "in" : "out"; }
HandleRef ref(const Value& v) { return {integer(v["s"]), integer(v["i"]), side(v["side"])}; }
LockHandle lockHandle(const Value& v) {
  LockHandle h(ref(v));
  h.flip = v.has("flip") && v["flip"].boolean();
  return h;
}
HandleLock lock(const Value& v) {
  HandleLock l;
  for (auto& h : v["handles"].arr()) l.handles.push_back(lockHandle(h));
  l.angle = v.has("angle") && v["angle"].boolean();
  l.length = v.has("length") && v["length"].boolean();
  return l;
}
std::vector<HandleLock> locks(const Value& v) { return list(v, lock); }
Value outLock(const HandleLock& l) {
  json::Array hs;
  for (auto& h : l.handles) {
    json::Object o{{"s", out(h.s)}, {"i", out(h.i)}, {"side", sideName(h.side)}};
    if (h.flip) o["flip"] = true;
    hs.push_back(o);
  }
  json::Object o{{"handles", hs}};
  if (l.angle) o["angle"] = true;
  if (l.length) o["length"] = true;
  return o;
}
Value outLocks(const std::vector<HandleLock>& ls) {
  json::Array a;
  for (auto& l : ls) a.push_back(outLock(l));
  return a;
}

static Register reg("nodes.json", {
  {"nodesOf", [](const Value& a) { return outNodes(nodesOf(P(a[0]))); }},
  {"segmentOf", [](const Value& a) { return out(segmentOf(node(a[0]), node(a[1]))); }},
  {"segmentsOf", [](const Value& a) { return out(segmentsOf(nodes(a[0]), flag(a[1]))); }},
  {"pathOf", [](const Value& a) { return ser(pathOf(nodes(a[0]), flag(a[1]), a.size() > 2 && !a[2].isNull() ? pt(a[2]) : Pt{1, 1})); }},
  {"pointOn", [](const Value& a) {
     auto ns = nodes(a[0]);
     json::Array r;
     for (auto& t : a[3].arr()) r.push_back(out(pointOn(ns, flag(a[1]), integer(a[2]), t.num())));
     return Value(r);
   }},
  {"nearestOn", [](const Value& a) {
     auto ns = nodes(a[0]);
     json::Array r;
     for (auto& p : a[2].arr()) {
       auto n = nearestOn(ns, flag(a[1]), pt(p));
       r.push_back(json::Object{{"seg", out(n.seg)}, {"t", out(n.t)}, {"p", out(n.p)}, {"d", out(n.d)}});
     }
     return Value(r);
   }},
  {"crossingsOn", [](const Value& a) {
     json::Array r;
     for (auto& c : crossingsOn(nodes(a[0]), flag(a[1]), integer(a[2]), a[3].num()))
       r.push_back(json::Object{{"seg", out(c.seg)}, {"t", out(c.t)}, {"p", out(c.p)}});
     return Value(r);
   }},
  {"moldNodes", [](const Value& a) {
     return outNodes(moldNodes(nodes(a[0]), flag(a[1]), integer(a[2]), a[3].num(), pt(a[4])));
   }},
  {"nodeKind", [](const Value& a) { return Value(name(nodeKind(node(a[0])))); }},
  {"withHandle", [](const Value& a) { return outNode(withHandle(node(a[0]), side(a[1]), pt(a[2]))); }},
  {"withNodeKind", [](const Value& a) { return outNode(withNodeKind(node(a[0]), kindOf(a[1].str()))); }},
  {"insertNode", [](const Value& a) { return outNodes(insertNode(nodes(a[0]), integer(a[1]), a[2].num())); }},
  {"shiftNodes", [](const Value& a) {
     std::function<bool(int)> pick;
     if (!a[2].isNull()) {
       auto idx = list(a[2], integer);
       pick = [idx](int i) { return std::find(idx.begin(), idx.end(), i) != idx.end(); };
     }
     return outNodes(shiftNodes(nodes(a[0]), pt(a[1]), pick));
   }},
  {"simplified", [](const Value& a) { return outNodes(simplified(nodes(a[0]), flag(a[1]), a[2].num())); }},
  {"lockOf", [](const Value& a) {
     auto ls = locks(a[0]);
     const HandleLock* l = lockOf(ls, ref(a[1]));
     return l ? out(int(l - ls.data())) : Value();
   }},
  {"withLockedHandle", [](const Value& a) {
     return outLists(withLockedHandle(lists(a[0]), locks(a[1]), ref(a[2]), pt(a[3])));
   }},
  {"locked", [](const Value& a) {
     LockHow how;
     how.angle = a[2].has("angle") && a[2]["angle"].boolean();
     how.length = a[2].has("length") && a[2]["length"].boolean();
     return outLocks(locked(locks(a[0]), list(a[1], ref), how));
   }},
  {"unlocked", [](const Value& a) { return outLocks(unlocked(locks(a[0]), list(a[1], ref))); }},
  {"remapLocks", [](const Value& a) {
     std::vector<std::optional<int>> to;
     for (auto& x : a[2].arr()) to.push_back(x.isNull() ? std::nullopt : std::optional(integer(x)));
     return outLocks(remapLocks(locks(a[0]), integer(a[1]), [&](int i) -> std::optional<int> {
       return i >= 0 && i < int(to.size()) ? to[i] : std::nullopt;
     }));
   }},
  {"lockedKind", [](const Value& a) { return Value(name(lockedKind(locks(a[0]), node(a[1]), integer(a[2]), integer(a[3])))); }},
  {"lockedKinds", [](const Value& a) { return outLists(lockedKinds(lists(a[0]), locks(a[1]))); }},
  {"kindLocks", [](const Value& a) {
     auto r = kindLocks(lists(a[0]));
     return Value(json::Object{{"lists", outLists(r.lists)}, {"locks", outLocks(r.locks)}});
   }},
});

}  // namespace
