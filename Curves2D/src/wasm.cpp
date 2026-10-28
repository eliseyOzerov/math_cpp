// The C++ API for JavaScript through embind, shaped like fountain-ts/src/curves: points are [x, y], cubics lists of
// four points, contours lists of pieces (two points for a line, four for a cubic), kinds and names strings, option
// objects objects. Built only under emscripten (CMakeLists.txt); tests/wasm/smoke.ts checks it against the TS.
//
// Decisions:
// - Path and Selection are embind classes: values in C++, but JS holds handles to them, so each one made must be
//   freed with `.delete()` (or held by a FinalizationRegistry). Everything else crosses as plain JS values.
// - Optional arguments are overloads by count, as embind picks an overload by the number of arguments.
// - A pick is a list of node indices, a function (p, i, at) → weight (true is 1), or a Pick from nodeAt, cubicAt or
//   section, as in the TS.
// - Mods take their values as an object of numbers and strings, as the TS's Values.
#ifdef __EMSCRIPTEN__

#include "curves/curves.hpp"

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <stdexcept>
#include <string>
#include <vector>

using namespace curves;
using emscripten::val;

namespace {

// JS values to C++ and back.

Pt pt(const val& v) { return {v[0].as<double>(), v[1].as<double>()}; }
val js(Pt p) {
  val a = val::array();
  a.set(0, p.x);
  a.set(1, p.y);
  return a;
}

template <class T, class F>
val list(const std::vector<T>& xs, F f) {
  val a = val::array();
  for (size_t i = 0; i < xs.size(); i++) a.set(i, f(xs[i]));
  return a;
}
val nums(const std::vector<double>& xs) {
  return list(xs, [](double x) { return val(x); });
}
val pts(const std::vector<Pt>& ps) { return list(ps, js); }
std::vector<double> numsOf(const val& v) { return emscripten::vecFromJSArray<double>(v); }
std::vector<Pt> ptsOf(const val& v) {
  std::vector<Pt> ps;
  for (int i = 0, n = v["length"].as<int>(); i < n; i++) ps.push_back(pt(v[i]));
  return ps;
}

Cubic cubic(const val& v) { return {pt(v[0]), pt(v[1]), pt(v[2]), pt(v[3])}; }
val js(const Cubic& c) {
  val a = val::array();
  for (int i = 0; i < 4; i++) a.set(i, js(c[i]));
  return a;
}
val cubics(const std::vector<Cubic>& cs) {
  return list(cs, [](const Cubic& c) { return js(c); });
}
std::vector<Cubic> cubicsOf(const val& v) {
  std::vector<Cubic> cs;
  for (int i = 0, n = v["length"].as<int>(); i < n; i++) cs.push_back(cubic(v[i]));
  return cs;
}

val js(const Piece& p) {
  val a = val::array();
  for (int i = 0; i < (p.line ? 2 : 4); i++) a.set(i, js(p.p[i]));
  return a;
}
val contours(const std::vector<Contour>& cs) {
  return list(cs, [](const Contour& c) { return list(c, [](const Piece& p) { return js(p); }); });
}
std::vector<Contour> contoursOf(const val& v) {
  std::vector<Contour> cs;
  for (int i = 0, n = v["length"].as<int>(); i < n; i++) {
    Contour c;
    val pieces = v[i];
    for (int j = 0, m = pieces["length"].as<int>(); j < m; j++) {
      val p = pieces[j];
      c.push_back(p["length"].as<int>() == 2 ? Piece::of(pt(p[0]), pt(p[1])) : Piece::of(cubic(p)));
    }
    cs.push_back(std::move(c));
  }
  return cs;
}

val box(const Box& b) {
  val a = val::array();
  a.set(0, js(b[0]));
  a.set(1, js(b[1]));
  return a;
}

const char* name(NodeKind k) { return k == NodeKind::corner ? "corner" : k == NodeKind::smooth ? "smooth" : "symmetric"; }
NodeKind kindOf(const std::string& s) {
  if (s == "corner") return NodeKind::corner;
  if (s == "smooth") return NodeKind::smooth;
  if (s == "symmetric") return NodeKind::symmetric;
  throw std::invalid_argument("unknown node kind " + s);
}
HandleSide sideOf(const std::string& s) { return s == "in" ? HandleSide::in : HandleSide::out; }
HandleLead leadOf(const std::string& s) {
  return s == "in" ? HandleLead::in : s == "out" ? HandleLead::out : HandleLead::both;
}
Which which(const val& v) {
  if (v.isUndefined()) return End::first;
  if (v.isNumber()) return v.as<int>();
  return v.as<std::string>() == "last" ? End::last : End::first;
}
FalloffSpec falloffOf(const val& v) {
  if (v.typeOf().as<std::string>() == "function") return Falloff([v](double x) { return v(x).as<double>(); });
  static const std::pair<const char*, FalloffName> names[] = {
      {"smooth", FalloffName::smooth}, {"linear", FalloffName::linear},     {"sphere", FalloffName::sphere},
      {"root", FalloffName::root},     {"sharp", FalloffName::sharp},       {"constant", FalloffName::constant}};
  std::string s = v.as<std::string>();
  for (auto& [n, f] : names)
    if (s == n) return f;
  throw std::invalid_argument("unknown falloff " + s);
}
Pivot pivot(const val& v) { return v.isUndefined() || v.isString() ? Pivot{} : Pivot{pt(v)}; }
std::optional<double> optNum(const val& o, const char* key) {
  if (o.isUndefined() || o.isNull()) return {};
  val v = o[key];
  return v.isUndefined() ? std::optional<double>{} : v.as<double>();
}

// Picks: a list, a function or a Pick.

NodePick nodePick(const val& v) {
  if (v.typeOf().as<std::string>() == "function")
    return std::function<double(Pt, int, Progress)>([v](Pt p, int i, Progress at) {
      val w = v(js(p), i, at);
      return w.isNumber() ? w.as<double>() : w.as<bool>() ? 1.0 : 0.0;
    });
  return emscripten::vecFromJSArray<int>(v);
}
bool isPick(const val& v) { return v.instanceof(val::module_property("Pick")); }

Selection selectOf(const Path& p, const val& v) {
  return isPick(v) ? p.select(v.as<Pick>()) : p.select(nodePick(v));
}
Selection reselect(const Selection& s, const val& v) {
  return isPick(v) ? s.select(v.as<Pick>()) : s.select(nodePick(v));
}

// Mods' values and their list.

Values values(const val& v) {
  Values out;
  if (v.isUndefined() || v.isNull()) return out;
  val keys = val::global("Object").call<val>("keys", v);
  for (int i = 0, n = keys["length"].as<int>(); i < n; i++) {
    std::string k = keys[i].as<std::string>();
    val x = v[k];
    if (x.isNumber()) out[k] = x.as<double>();
    else out[k] = x.as<std::string>();
  }
  return out;
}
val modList() {
  val a = val::array();
  int i = 0;
  for (const Mod& m : mods()) {
    val o = val::object(), params = val::object();
    o.set("name", m.name);
    o.set("group", m.group);
    o.set("doc", m.doc);
    for (auto& [k, p] : m.params) {
      val q = val::object();
      if (auto* n = std::get_if<NumParam>(&p)) {
        q.set("min", n->min);
        q.set("max", n->max);
        q.set("step", n->step);
        q.set("value", n->value);
      } else {
        auto& c = std::get<ChoiceParam>(p);
        q.set("options", list(c.options, [](const std::string& s) { return val(s); }));
        q.set("value", c.value);
      }
      params.set(k, q);
    }
    o.set("params", params);
    a.set(i++, o);
  }
  return a;
}
std::vector<Step> steps(const val& v) {
  std::vector<Step> out;
  for (int i = 0, n = v["length"].as<int>(); i < n; i++)
    out.push_back({v[i]["mod"].as<std::string>(), values(v[i]["values"])});
  return out;
}

// Path's answers as objects.

val projection(const PathProjection& r) {
  val o = val::object();
  o.set("at", r.at);
  o.set("p", js(r.p));
  o.set("d", r.d);
  return o;
}

// The lower files' values: pieces, nodes and their locks, corners, keys, targets.

int count(const val& v) { return v["length"].as<int>(); }
bool has(const val& o, const char* key) { return !o.isUndefined() && !o.isNull() && !o[key].isUndefined(); }
bool isFunction(const val& v) { return v.typeOf().as<std::string>() == "function"; }
val obj(std::initializer_list<std::pair<const char*, val>> kvs) {
  val o = val::object();
  for (auto& [k, v] : kvs) o.set(k, v);
  return o;
}

Piece piece(const val& v) { return count(v) == 2 ? Piece::of(pt(v[0]), pt(v[1])) : Piece::of(cubic(v)); }
std::vector<Piece> piecesOf(const val& v) {
  std::vector<Piece> ps;
  for (int i = 0, n = count(v); i < n; i++) ps.push_back(piece(v[i]));
  return ps;
}
val pieces(const std::vector<Piece>& ps) {
  return list(ps, [](const Piece& p) { return js(p); });
}
std::vector<std::vector<Pt>> polysOf(const val& v) {
  std::vector<std::vector<Pt>> ps;
  for (int i = 0, n = count(v); i < n; i++) ps.push_back(ptsOf(v[i]));
  return ps;
}
val affine(const Affine& m) {
  val a = val::array();
  for (int i = 0; i < 6; i++) a.set(i, m[i]);
  return a;
}
Affine affineOf(const val& v) {
  Affine m;
  for (int i = 0; i < 6; i++) m[i] = v[i].as<double>();
  return m;
}

Node node(const val& v) {
  Node n{pt(v["p"]), {}, {}, {}};
  if (has(v, "in")) n.in = pt(v["in"]);
  if (has(v, "out")) n.out = pt(v["out"]);
  if (has(v, "kind")) n.kind = kindOf(v["kind"].as<std::string>());
  return n;
}
val js(const Node& n) {
  val o = obj({{"p", js(n.p)}});
  if (n.kind) o.set("kind", name(*n.kind));
  if (n.in) o.set("in", js(*n.in));
  if (n.out) o.set("out", js(*n.out));
  return o;
}
std::vector<Node> nodesOf(const val& v) {
  std::vector<Node> ns;
  for (int i = 0, n = count(v); i < n; i++) ns.push_back(node(v[i]));
  return ns;
}
val nodes(const std::vector<Node>& ns) {
  return list(ns, [](const Node& n) { return js(n); });
}
std::vector<std::vector<Node>> nodeListsOf(const val& v) {
  std::vector<std::vector<Node>> ls;
  for (int i = 0, n = count(v); i < n; i++) ls.push_back(nodesOf(v[i]));
  return ls;
}
val nodeLists(const std::vector<std::vector<Node>>& ls) {
  return list(ls, [](const std::vector<Node>& ns) { return nodes(ns); });
}

HandleRef ref(const val& v) { return {v["s"].as<int>(), v["i"].as<int>(), sideOf(v["side"].as<std::string>())}; }
std::vector<HandleRef> refsOf(const val& v) {
  std::vector<HandleRef> rs;
  for (int i = 0, n = count(v); i < n; i++) rs.push_back(ref(v[i]));
  return rs;
}
std::vector<HandleLock> locksOf(const val& v) {
  std::vector<HandleLock> ls;
  for (int i = 0, n = count(v); i < n; i++) {
    val l = v[i], hs = l["handles"];
    HandleLock k;
    for (int j = 0, m = count(hs); j < m; j++) {
      LockHandle h(ref(hs[j]));
      h.flip = has(hs[j], "flip") && hs[j]["flip"].as<bool>();
      k.handles.push_back(h);
    }
    k.angle = has(l, "angle") && l["angle"].as<bool>();
    k.length = has(l, "length") && l["length"].as<bool>();
    ls.push_back(std::move(k));
  }
  return ls;
}
val js(const HandleLock& l) {
  val o = obj({{"handles", list(l.handles, [](const LockHandle& h) {
                  val r = obj({{"s", val(h.s)}, {"i", val(h.i)}, {"side", val(h.side == HandleSide::in ? "in" : "out")}});
                  if (h.flip) r.set("flip", true);
                  return r;
                })}});
  if (l.angle) o.set("angle", true);
  if (l.length) o.set("length", true);
  return o;
}
val locks(const std::vector<HandleLock>& ls) {
  return list(ls, [](const HandleLock& l) { return js(l); });
}

val js(const Corner& k) {
  return obj({{"contour", val(k.contour)}, {"at", val(k.at)}, {"p", js(k.p)}, {"in", js(k.in)}, {"out", js(k.out)},
              {"turn", val(k.turn)}, {"outside", val(k.outside)}});
}
val cornerList(const std::vector<Corner>& ks) {
  return list(ks, [](const Corner& k) { return js(k); });
}
CornerOpts cornerOpts(const val& o) {
  CornerOpts c;
  if (auto m = optNum(o, "minDeg")) c.minDeg = *m;
  if (auto s = optNum(o, "share")) c.share = *s;
  return c;
}
std::function<double(const Corner&)> perCorner(const val& f) {
  return [f](const Corner& k) { return f(js(k)).as<double>(); };
}

// Keys are [t, v] or [t, v, { in?, out? }], as in the TS.
Keys<double> keysOf(const val& v) {
  Keys<double> ks;
  for (int i = 0, n = count(v); i < n; i++) {
    val k = v[i];
    Key<double> key{k[0].as<double>(), k[1].as<double>()};
    if (count(k) > 2) {
      Handles h;
      if (has(k[2], "in")) h.in = pt(k[2]["in"]);
      if (has(k[2], "out")) h.out = pt(k[2]["out"]);
      key.h = h;
    }
    ks.push_back(key);
  }
  return ks;
}
std::optional<Keys<double>> optKeys(const val& v) {
  if (v.isUndefined() || v.isNull()) return {};
  return keysOf(v);
}
val keys(const Keys<double>& ks) {
  return list(ks, [](const Key<double>& k) {
    val a = val::array();
    a.set(0, k.t);
    a.set(1, k.v);
    if (k.h) {
      val h = val::object();
      if (k.h->in) h.set("in", js(*k.h->in));
      if (k.h->out) h.set("out", js(*k.h->out));
      a.set(2, h);
    }
    return a;
  });
}
val optKeys(const std::optional<Keys<double>>& k) { return k ? keys(*k) : val::null(); }

Segs segsOf(const val& v) { return v.isNumber() ? Segs(v.as<int>()) : Segs(v[0].as<int>(), v[1].as<int>()); }
std::optional<Segs> optSegs(const val& o) {
  return has(o, "segs") ? std::optional<Segs>(segsOf(o["segs"])) : std::nullopt;
}
Target targetOf(const val& t) {
  if (has(t, "point")) return TargetPoint{pt(t["point"])};
  if (has(t, "cross"))
    return TargetCross{t["cross"].as<Path>(), t["with"].as<Path>(),
                       has(t, "which") ? std::optional<Which>(which(t["which"])) : std::nullopt};
  return TargetOn{t["on"].as<Path>(), t["at"].as<double>(), optSegs(t)};
}

// Functions across the boundary: a JS function passed in is called through `val`; a warp or wave made here goes out
// as a JS function holding a handle, freed when the function is collected.

Wave waveOf(const val& f) {
  return [f](double x) { return f(x).as<double>(); };
}
Warp warpOf(const val& f) {
  return [f](Pt p) { return pt(f(js(p))); };
}
struct WarpFn {
  Warp f;
  val call(val p) const { return js(f(pt(p))); }
};
struct WaveFn {
  Wave f;
  double call(double x) const { return f(x); }
};
val closure(val handle) {
  static val wrap = val::global("Function").new_(
      val("h"), val("r"), val("const f = (...a) => h.call(...a); r.register(f, h); return f"));
  static val registry = val::global("FinalizationRegistry").new_(val::global("Function").new_(val("h"), val("h.delete()")));
  return wrap(handle, registry);
}
val js(Warp f) { return closure(val(WarpFn{std::move(f)})); }
val js(Wave f) { return closure(val(WaveFn{std::move(f)})); }

Wobble wobbleOf(const val& o) {
  Wobble w;
  w.amplitude = o["amplitude"].as<double>();
  if (auto f = optNum(o, "frequency")) w.frequency = *f;
  if (auto k = optNum(o, "octaves")) w.octaves = *k;
  if (auto s = optNum(o, "seed")) w.seed = *s;
  return w;
}

Op opOf(const std::string& s) {
  static const std::pair<const char*, Op> ops[] = {{"union", Op::union_}, {"intersect", Op::intersect},
                                                   {"subtract", Op::subtract}, {"exclude", Op::exclude}, {"cut", Op::cut}};
  for (auto& [n, o] : ops)
    if (s == n) return o;
  throw std::invalid_argument("unknown op " + s);
}
std::vector<Operand> operandsOf(const val& v) {
  std::vector<Operand> os;
  for (int i = 0, n = count(v); i < n; i++) {
    val x = v[i];
    if (x.instanceof(val::module_property("Path"))) os.push_back(x.as<Path>());
    else if (x.instanceof(val::module_property("Bool"))) os.push_back(std::make_shared<const Bool>(x.as<Bool>()));
    else os.push_back(contoursOf(x));
  }
  return os;
}
std::optional<int> optInt(const val& v) { return v.isUndefined() || v.isNull() ? std::nullopt : std::optional<int>(v.as<int>()); }
val boolCorners(const std::vector<BoolCorner>& ks) {
  return list(ks, [](const BoolCorner& k) {
    val between = val::array();
    between.set(0, k.between[0]);
    between.set(1, k.between[1]);
    return obj({{"contour", val(k.contour)}, {"node", val(k.node)}, {"p", js(k.p)}, {"between", between},
                {"in", js(k.in)}, {"out", js(k.out)}, {"outward", js(k.outward)}, {"convex", val(k.convex)}});
  });
}

// The TS's pathOf takes cubics (path.ts) or nodes (nodes.ts); a list of objects with `p` is nodes.
bool areNodes(const val& v) { return count(v) > 0 && !v[0].isArray(); }

}  // namespace

EMSCRIPTEN_BINDINGS(curves) {
  using emscripten::class_;
  using emscripten::function;
  using emscripten::optional_override;

  class_<Pick>("Pick");
  function("nodeAt", optional_override([](val i) { return nodeAt(emscripten::vecFromJSArray<int>(i)); }));
  function("cubicAt", optional_override([](val i) { return cubicAt(emscripten::vecFromJSArray<int>(i)); }));
  function("section", &section);

  class_<Path>("Path")
      .property("start", optional_override([](const Path& p) { return js(p.start()); }))
      .property("segs", optional_override([](const Path& p) { return cubics(p.segs()); }))
      .property("closed", &Path::closed)
      .property("metric", optional_override([](const Path& p) { return js(p.metric()); }))
      .function("measured", optional_override([](const Path& p, val m) { return p.measured(pt(m)); }))
      .function("kind", optional_override([](const Path& p, int i) { return std::string(name(p.kind(i))); }))
      .property("kinds", optional_override([](const Path& p) {
                  return list(p.kinds(), [](NodeKind k) { return val(name(k)); });
                }))
      .function("withKind", optional_override([](const Path& p, std::string k) { return p.withKind(kindOf(k)); }))
      .function("withKind", optional_override([](const Path& p, std::string k, val pick) {
                  return p.withKind(kindOf(k), nodePick(pick));
                }))
      .function("handle", optional_override([](const Path& p, int i, std::string side, val to) {
                  return p.handle(i, sideOf(side), pt(to));
                }))
      .property("length", &Path::length)
      .function("locate", optional_override([](const Path& p, double at) {
                  Located l = p.locate(at);
                  val o = val::object();
                  o.set("i", l.i);
                  o.set("t", l.t);
                  return o;
                }))
      .function("progress", &Path::progress)
      .property("end", optional_override([](const Path& p) { return js(p.end()); }))
      .function("line", optional_override([](const Path& p, val to) { return p.line(pt(to)); }))
      .function("curve", optional_override([](const Path& p, val c1, val c2, val to) {
                  return p.curve(pt(c1), pt(c2), pt(to));
                }))
      .function("quarter", optional_override([](const Path& p, val to) { return p.quarter(pt(to)); }))
      .function("quarter", optional_override([](const Path& p, val to, val o) {
                  QuarterOptions q;
                  if (!o["from"].isUndefined()) q.from = o["from"].as<std::string>() == "h" ? Heading::h : Heading::v;
                  if (auto t = optNum(o, "tension")) q.tension = *t;
                  return p.quarter(pt(to), q);
                }))
      .function("arch", optional_override([](const Path& p, val to, double peak) { return p.arch(pt(to), peak); }))
      .function("arch", optional_override([](const Path& p, val to, double peak, double tension) {
                  return p.arch(pt(to), peak, tension);
                }))
      .function("join", &Path::join)
      .function("close", &Path::close)
      .function("at", optional_override([](const Path& p, double at) { return js(p.at(at)); }))
      .function("crossY", optional_override([](const Path& p, double y) { return pts(p.crossY(y)); }))
      .function("crossX", optional_override([](const Path& p, double x) { return pts(p.crossX(x)); }))
      .function("atY", optional_override([](const Path& p, double y) { return js(p.atY(y)); }))
      .function("atY", optional_override([](const Path& p, double y, val w) { return js(p.atY(y, which(w))); }))
      .function("atX", optional_override([](const Path& p, double x) { return js(p.atX(x)); }))
      .function("atX", optional_override([](const Path& p, double x, val w) { return js(p.atX(x, which(w))); }))
      .function("tAtY", optional_override([](const Path& p, double y) { return p.tAtY(y); }))
      .function("tAtY", optional_override([](const Path& p, double y, val w) { return p.tAtY(y, which(w)); }))
      .function("tAtX", optional_override([](const Path& p, double x) { return p.tAtX(x); }))
      .function("tAtX", optional_override([](const Path& p, double x, val w) { return p.tAtX(x, which(w)); }))
      .function("chordY", &Path::chordY)
      .function("chordX", &Path::chordX)
      .function("slice", &Path::slice)
      .function("aperture", optional_override([](const Path& p, double offset) { return p.aperture(offset); }))
      .function("aperture", &Path::aperture)
      .function("divide", &Path::divide)
      .function("direction", optional_override([](const Path& p, double at) { return js(p.direction(at)); }))
      .function("crossings", optional_override([](const Path& p, const Path& other) {
                  return list(p.crossings(other), [](const PathCrossing& c) {
                    val o = val::object();
                    o.set("p", js(c.p));
                    o.set("t", c.t);
                    o.set("u", c.u);
                    return o;
                  });
                }))
      .function("cut", optional_override([](const Path& p, val ps) { return p.cut(numsOf(ps)); }))
      .property("angle", &Path::angle)
      .property("nodeCount", &Path::nodeCount)
      .function("node", optional_override([](const Path& p, int i) { return js(p.node(i)); }))
      .function("nodeProgress", &Path::nodeProgress)
      .property("nodes", optional_override([](const Path& p) { return pts(p.nodes()); }))
      .property("distances", optional_override([](const Path& p) { return nums(p.distances()); }))
      .function("reverse", &Path::reverse)
      .function("startAt", &Path::startAt)
      .function("select", optional_override([](const Path& p) { return p.select(); }))
      .function("select", optional_override([](const Path& p, val pick) { return selectOf(p, pick); }))
      .function("attach", optional_override([](const Path& p, const Path& on, double at) { return p.attach(on, at); }))
      .function("project", optional_override([](const Path& p, val q) { return projection(p.project(pt(q))); }))
      .function("extrema", optional_override([](const Path& p) { return nums(p.extrema()); }))
      .function("inflections", optional_override([](const Path& p) { return nums(p.inflections()); }))
      .function("bbox", optional_override([](const Path& p) { return box(p.bbox()); }))
      .function("curvature", &Path::curvature)
      .function("normal", optional_override([](const Path& p, double at) { return js(p.normal(at)); }))
      .function("mold", optional_override([](const Path& p, double at, val to) { return p.mold(at, pt(to)); }))
      .function("offset", optional_override([](const Path& p, double d) { return p.offset(d); }))
      .function("outline", optional_override([](const Path& p, double d) { return contours(p.outline(d)); }))
      .function("outline", optional_override([](const Path& p, double d1, double d2) {
                  return contours(p.outline(d1, d2));
                }))
      .function("selectAt", optional_override([](const Path& p, double at, double reach) {
                  return p.selectAt(at, reach);
                }));

  function("from", optional_override([](val p) { return from(pt(p)); }));
  function("line", optional_override([](val a, val b) { return line(pt(a), pt(b)); }));
  function("poly", optional_override([](val p, val ps) { return poly(pt(p), ptsOf(ps)); }));
  function("pathOf", optional_override([](val segs) { return Path::of(cubicsOf(segs)); }));
  function("pathOf", optional_override([](val v, bool closed) {
             return areNodes(v) ? pathOf(nodesOf(v), closed) : Path::of(cubicsOf(v), closed);
           }));
  function("pathOf", optional_override([](val ns, bool closed, val metric) { return pathOf(nodesOf(ns), closed, pt(metric)); }));

  class_<Selection>("Selection")
      .property("weights", optional_override([](const Selection& s) {
                  return list(s.weights(), [](const std::vector<double>& w) { return nums(w); });
                }))
      .property("points", optional_override([](const Selection& s) { return pts(s.points()); }))
      .property("center", optional_override([](const Selection& s) { return js(s.center()); }))
      .property("bounds", optional_override([](const Selection& s) { return box(s.bounds()); }))
      .function("and", optional_override([](const Selection& s, const Selection& o) { return s.and_(o); }))
      .function("soften", optional_override([](const Selection& s, double reach) { return s.soften(reach); }))
      .function("soften", optional_override([](const Selection& s, double reach, val o) {
                  SoftenOptions so;
                  if (!o["falloff"].isUndefined()) so.falloff = falloffOf(o["falloff"]);
                  if (!o["along"].isUndefined()) so.along = o["along"].as<std::string>() == "plane" ? Along::plane : Along::path;
                  return s.soften(reach, so);
                }))
      .function("weigh", optional_override([](const Selection& s, val k) {
                  if (k.isNumber()) return s.weigh(k.as<double>());
                  if (k.isArray()) return s.weigh(numsOf(k));
                  return s.weigh(Wave([k](double x) { return k(x).as<double>(); }));
                }))
      .function("invert", &Selection::invert)
      .function("put", optional_override([](const Selection& s, val to) {
                  return s.put(Place{optNum(to, "x"), optNum(to, "y")});
                }))
      .function("kind", optional_override([](const Selection& s, std::string k) { return s.kind(kindOf(k)); }))
      .function("handles", optional_override([](const Selection& s, val o) {
                  HandlesOptions h;
                  h.angle = optNum(o, "angle");
                  if (auto t = optNum(o, "turn")) h.turn = *t;
                  if (auto l = optNum(o, "length")) h.length = *l;
                  if (!o["side"].isUndefined()) h.side = leadOf(o["side"].as<std::string>());
                  return s.handles(h);
                }))
      .function("inflate", optional_override([](const Selection& s, double r) { return s.inflate(r); }))
      .function("inflate", optional_override([](const Selection& s, double r, val o) {
                  InflateOptions io;
                  if (!o["align"].isUndefined()) io.align = pt(o["align"]);
                  if (auto h = optNum(o, "handles")) io.handles = *h;
                  if (auto m = optNum(o, "smooth")) io.smooth = *m;
                  if (auto k = optNum(o, "strength")) io.strength = *k;
                  if (!o["along"].isUndefined())
                    io.along = o["along"].as<std::string>() == "path" ? InflateAlong::path : InflateAlong::ring;
                  return s.inflate(r, io);
                }))
      .function("smooth", optional_override([](const Selection& s) { return s.smooth(); }))
      .function("smooth", &Selection::smooth)
      .function("fair", optional_override([](const Selection& s) { return s.fair(); }))
      .function("fair", &Selection::fair)
      .function("rotate", optional_override([](const Selection& s, double deg) { return s.rotate(deg); }))
      .function("rotate", optional_override([](const Selection& s, double deg, val about) {
                  return s.rotate(deg, pivot(about));
                }))
      .function("shear", optional_override([](const Selection& s, double deg) { return s.shear(deg); }))
      .function("shear", optional_override([](const Selection& s, double deg, val o) {
                  ShearOptions so;
                  if (auto a = optNum(o, "axis")) so.axis = *a;
                  so.about = pivot(o["about"]);
                  return s.shear(deg, so);
                }))
      .function("shift", optional_override([](const Selection& s, double d) { return s.shift(d); }))
      .function("shift", &Selection::shift)
      .function("scale", optional_override([](const Selection& s, val k) {
                  return k.isNumber() ? s.scale(k.as<double>()) : s.scale(pt(k));
                }))
      .function("scale", optional_override([](const Selection& s, val k, val about) {
                  return k.isNumber() ? s.scale(k.as<double>(), pivot(about)) : s.scale(pt(k), pivot(about));
                }))
      .function("mirror", optional_override([](const Selection& s) { return s.mirror(); }))
      .function("mirror", optional_override([](const Selection& s, double axis) { return s.mirror(axis); }))
      .function("mirror", optional_override([](const Selection& s, double axis, val about) {
                  return s.mirror(axis, pivot(about));
                }))
      .property("paths", optional_override([](const Selection& s) {
                  val a = val::array();
                  auto ps = s.paths();
                  for (size_t i = 0; i < ps.size(); i++) a.set(i, val(ps[i]));
                  return a;
                }))
      .property("path", &Selection::path)
      .function("select", optional_override([](const Selection& s) { return s.select(); }))
      .function("select", optional_override([](const Selection& s, val pick) { return reselect(s, pick); }));

  function("group", optional_override([](val of) {
             std::vector<Selection> ss;
             for (int i = 0, n = of["length"].as<int>(); i < n; i++) {
               val x = of[i];
               ss.push_back(x.instanceof(val::module_property("Path")) ? x.as<Path>().select() : x.as<Selection>());
             }
             return group(ss);
           }));

  function("mods", &modList);
  function("modDefaults", optional_override([](std::string m) {
             val o = val::object();
             for (auto& [k, v] : modDefaults(mod(m))) std::visit([&](auto& x) { o.set(k, x); }, v);
             return o;
           }));
  function("applyMod", optional_override([](const Selection& s, std::string m) { return applyMod(s, mod(m)); }));
  function("applyMod", optional_override([](const Selection& s, std::string m, val v) {
             return applyMod(s, mod(m), values(v));
           }));
  function("modCode", optional_override([](std::string m) { return modCode(mod(m)); }));
  function("modCode", optional_override([](std::string m, val v) { return modCode(mod(m), values(v)); }));
  function("chain", optional_override([](const Selection& s, val st) { return chain(s, steps(st)); }));
  function("chainCode", optional_override([](val st) { return chainCode(steps(st)); }));

  function("parseSvgPath", optional_override([](std::string d) { return contours(parseSvgPath(d)); }));
  function("svgPath", optional_override([](val cs) { return svgPath(contoursOf(cs)); }));
  function("svgPath", optional_override([](val cs, int digits) { return svgPath(contoursOf(cs), digits); }));
}

// The lower files' free functions, the ones fountain-ts reaches for outside src/curves.
EMSCRIPTEN_BINDINGS(curves_free) {
  using emscripten::class_;
  using emscripten::function;
  using emscripten::optional_override;

  class_<WarpFn>("WarpFn").function("call", &WarpFn::call);
  class_<WaveFn>("WaveFn").function("call", &WaveFn::call);

  // geometry
  function("add", optional_override([](val a, val b) { return js(pt(a) + pt(b)); }));
  function("sub", optional_override([](val a, val b) { return js(pt(a) - pt(b)); }));
  function("mul", optional_override([](val a, double k) { return js(pt(a) * k); }));
  function("dot", optional_override([](val a, val b) { return dot(pt(a), pt(b)); }));
  function("cross", optional_override([](val a, val b) { return cross(pt(a), pt(b)); }));
  function("len", optional_override([](val v) { return len(pt(v)); }));
  function("dist", optional_override([](val a, val b) { return dist(pt(a), pt(b)); }));
  function("lerp", optional_override([](val a, val b, double t) { return js(lerp(pt(a), pt(b), t)); }));
  function("norm", optional_override([](val v) { return js(norm(pt(v))); }));
  function("rad", optional_override([](double d) { return rad(d); }));
  function("deg", optional_override([](double r) { return deg(r); }));
  function("clamp", optional_override([](double v, double a, double b) { return clamp(v, a, b); }));
  function("halfTurn", &halfTurn);
  function("turn", optional_override([](val a, val b) { return turn(pt(a), pt(b)); }));
  function("rotate", optional_override([](val v, double t) { return js(rotate(pt(v), t)); }));
  function("bez", optional_override([](val c, double t) { return js(bez(cubic(c), t)); }));
  function("bezD", optional_override([](val c, double t) { return js(bezD(cubic(c), t)); }));
  function("split", optional_override([](val c, double t) {
             auto [l, r] = split(cubic(c), t);
             val a = val::array();
             a.set(0, js(l));
             a.set(1, js(r));
             return a;
           }));
  function("nearSegment", optional_override([](val p, val a, val b, double tol) {
             return nearSegment(pt(p), pt(a), pt(b), tol);
           }));
  function("nearSegment", optional_override([](val p, val a, val b, double tol, double over) {
             return nearSegment(pt(p), pt(a), pt(b), tol, over);
           }));
  function("isLine", optional_override([](val c) { return isLine(cubic(c)); }));
  function("lineCubic", optional_override([](val a, val b) { return js(lineCubic(pt(a), pt(b))); }));
  static constexpr auto near = [](const Nearest& n) { return obj({{"i", val(n.i)}, {"t", val(n.t)}, {"p", js(n.p)}, {"d", val(n.d)}}); };
  function("nearest", optional_override([](val segs, val p) { return near(nearest(cubicsOf(segs), pt(p))); }));
  function("nearest", optional_override([](val segs, val p, int steps) {
             return near(nearest(cubicsOf(segs), pt(p), steps));
           }));
  function("arcLength", optional_override([](val c) { return arcLength(cubic(c)); }));
  function("arcLength", optional_override([](val c, double t) { return arcLength(cubic(c), t); }));
  function("box", optional_override([](val qs) { return box(curves::box(piecesOf(qs))); }));
  function("box", optional_override([](val qs, int samples) { return box(curves::box(piecesOf(qs), samples)); }));
  function("arcCubics", optional_override([](val a, val b, double al) { return cubics(arcCubics(pt(a), pt(b), al)); }));
  function("apply", optional_override([](val m, val p) { return js(curves::apply(affineOf(m), pt(p))); }));
  function("rotation", optional_override([](double d) { return affine(rotation(d)); }));
  function("rotation", optional_override([](double d, val p) { return affine(rotation(d, pt(p))); }));
  function("scaling", optional_override([](double kx) { return affine(scaling(kx, kx)); }));
  function("scaling", optional_override([](double kx, double ky) { return affine(scaling(kx, ky)); }));
  function("scaling", optional_override([](double kx, double ky, val p) { return affine(scaling(kx, ky, pt(p))); }));
  function("shift", optional_override([](double d) { return affine(shift(d)); }));
  function("shift", optional_override([](double d, double axis) { return affine(shift(d, axis)); }));

  // bezier, bezn
  function("raise", optional_override([](val ps) { return pts(raise(ptsOf(ps))); }));

  // fit
  function("fitCubic", optional_override([](val ps, val ta, val tb) {
             CubicFit f = fitCubic(ptsOf(ps), pt(ta), pt(tb));
             return obj({{"c", js(f.c)}, {"err", val(f.err)}});
           }));
  function("fitCubics", optional_override([](val ps, double tol) { return cubics(fitCubics(ptsOf(ps), tol)); }));
  function("fitCubics", optional_override([](val ps, double tol, val o) {
             FitOptions f;
             if (auto c = optNum(o, "corner")) f.corner = *c;
             return cubics(fitCubics(ptsOf(ps), tol, f));
           }));
  function("cubicToQuads", optional_override([](val c, double tol) { return pts(cubicToQuads(cubic(c), tol)); }));
  function("hull", optional_override([](val ps) { return pts(hull(ptsOf(ps))); }));
  function("hull", optional_override([](val ps, bool sorted) { return pts(hull(ptsOf(ps), sorted)); }));
  function("mergeHulls", optional_override([](val a, val b) { return pts(mergeHulls(ptsOf(a), ptsOf(b))); }));
  function("clipPolygon", optional_override([](val poly, val plane) {
             return pts(clipPolygon(ptsOf(poly), Plane{pt(plane["at"]), pt(plane["n"])}));
           }));

  // boolean
  function("contourArea", optional_override([](val c) { return contourArea(piecesOf(c)); }));
  function("orient", optional_override([](val cs) { return contours(orient(contoursOf(cs))); }));
  function("flatten", optional_override([](val cs) { return list(flatten(contoursOf(cs)), pts); }));
  function("flatten", optional_override([](val cs, double step) { return list(flatten(contoursOf(cs), step), pts); }));
  function("flatten", optional_override([](val cs, double step, double min) {
             return list(flatten(contoursOf(cs), step, min), pts);
           }));
  function("edges", optional_override([](val polys) {
             return list(edges(polysOf(polys)), [](const Edge& e) { return pts({e[0], e[1]}); });
           }));
  function("scanline", optional_override([](val es, double y) {
             std::vector<Edge> e;
             for (int i = 0, n = count(es); i < n; i++) e.push_back({pt(es[i][0]), pt(es[i][1])});
             return list(scanline(e, y), [](const std::array<double, 2>& r) { return nums({r[0], r[1]}); });
           }));

  // corners
  function("pits", optional_override([](val cs) { return cornerList(pits(contoursOf(cs))); }));
  function("pits", optional_override([](val cs, double maxDeg) { return cornerList(pits(contoursOf(cs), maxDeg)); }));
  function("peaks", optional_override([](val cs) { return cornerList(peaks(contoursOf(cs))); }));
  function("peaks", optional_override([](val cs, double minDeg) { return cornerList(peaks(contoursOf(cs), minDeg)); }));
  static constexpr auto round = [](val cs, val r, val o) {
    auto c = contoursOf(cs);
    return contours(r.isNumber() ? roundCorners(c, r.as<double>(), cornerOpts(o)) : roundCorners(c, perCorner(r), cornerOpts(o)));
  };
  function("roundCorners", optional_override([](val cs, val r) { return round(cs, r, val::undefined()); }));
  function("roundCorners", optional_override([](val cs, val r, val o) { return round(cs, r, o); }));
  function("shapeCorners", optional_override([](val cs, val size, val shape) {
             return contours(shapeCorners(contoursOf(cs), perCorner(size), [shape](const Corner& k) {
               val s = shape(js(k));
               if (s.isNumber()) return CornerShape(s.as<double>());
               if (has(s, "profile")) return CornerShape(s["angle"].as<double>(), piecesOf(s["profile"]));
               return CornerShape(s["angle"].as<double>());
             }));
           }));
  function("capped", optional_override([](val cs, val end, val cap) {
             EndFrame e{pt(end["p"]), pt(end["t"]), end["r"].as<double>()};
             Cap c{piecesOf(cap["shape"]), optNum(cap, "align")};
             return contours(capped(contoursOf(cs), e, c));
           }));
  function("capsTriangle", optional_override([]() { return pieces(caps::triangle()); }));
  function("capsTriangle", optional_override([](double h) { return pieces(caps::triangle(h)); }));
  function("capsRect", optional_override([]() { return pieces(caps::rect()); }));
  function("capsRect", optional_override([](double h) { return pieces(caps::rect(h)); }));
  function("capsArc", optional_override([]() { return pieces(caps::arc()); }));
  function("capsArc", optional_override([](double h) { return pieces(caps::arc(h)); }));

  // nodes (plain Nodes: fields beyond p, in, out and kind aren't carried)
  function("nodesOf", optional_override([](const Path& p) { return nodes(nodesOf(p)); }));
  function("segmentsOf", optional_override([](val ns, bool closed) { return cubics(segmentsOf(nodesOf(ns), closed)); }));
  function("pointOn", optional_override([](val ns, bool closed, int seg, double t) {
             return js(pointOn(nodesOf(ns), closed, seg, t));
           }));
  function("nearestOn", optional_override([](val ns, bool closed, val p) {
             NodesNearest n = nearestOn(nodesOf(ns), closed, pt(p));
             return obj({{"seg", val(n.seg)}, {"t", val(n.t)}, {"p", js(n.p)}, {"d", val(n.d)}});
           }));
  function("crossingsOn", optional_override([](val ns, bool closed, int j, double v) {
             return list(crossingsOn(nodesOf(ns), closed, j, v), [](const NodesCrossing& c) {
               return obj({{"seg", val(c.seg)}, {"t", val(c.t)}, {"p", js(c.p)}});
             });
           }));
  function("moldNodes", optional_override([](val ns, bool closed, int seg, double t, val to) {
             return nodes(moldNodes(nodesOf(ns), closed, seg, t, pt(to)));
           }));
  function("nodeKind", optional_override([](val n) { return std::string(name(nodeKind(node(n)))); }));
  function("withHandle", optional_override([](val n, std::string side, val to) {
             return js(withHandle(node(n), sideOf(side), pt(to)));
           }));
  function("withNodeKind", optional_override([](val n, std::string k) { return js(withNodeKind(node(n), kindOf(k))); }));
  function("insertNode", optional_override([](val ns, int seg, double t) { return nodes(insertNode(nodesOf(ns), seg, t)); }));
  function("simplified", optional_override([](val ns, bool closed, double tol) {
             return nodes(simplified(nodesOf(ns), closed, tol));
           }));
  function("lockOf", optional_override([](val ls, val r) {
             auto all = locksOf(ls);
             const HandleLock* l = lockOf(all, ref(r));
             return l ? js(*l) : val::undefined();
           }));
  function("withLockedHandle", optional_override([](val lists, val ls, val r, val to) {
             return nodeLists(withLockedHandle(nodeListsOf(lists), locksOf(ls), ref(r), pt(to)));
           }));
  function("locked", optional_override([](val ls, val refs, val how) {
             LockHow h{has(how, "angle") && how["angle"].as<bool>(), has(how, "length") && how["length"].as<bool>()};
             return locks(locked(locksOf(ls), refsOf(refs), h));
           }));
  function("unlocked", optional_override([](val ls, val refs) { return locks(unlocked(locksOf(ls), refsOf(refs))); }));
  function("remapLocks", optional_override([](val ls, int s, val to) {
             return locks(remapLocks(locksOf(ls), s, [to](int i) { return optInt(to(i)); }));
           }));
  function("lockedKind", optional_override([](val ls, val n, int s, int i) {
             return std::string(name(lockedKind(locksOf(ls), node(n), s, i)));
           }));
  function("lockedKinds", optional_override([](val lists, val ls) {
             return nodeLists(lockedKinds(nodeListsOf(lists), locksOf(ls)));
           }));
  function("kindLocks", optional_override([](val lists) {
             KindLocks k = kindLocks(nodeListsOf(lists));
             return obj({{"lists", nodeLists(k.lists)}, {"locks", locks(k.locks)}});
           }));

  // profile (number keys)
  function("smoothstep", &smoothstep);
  function("easeKeys", optional_override([](double t0, double a, double t1, double b) { return keys(easeKeys(t0, a, t1, b)); }));
  function("keysAt", optional_override([](val k, double t) { return keysAt(optKeys(k), t); }));
  function("keysAt", optional_override([](val k, double t, double dflt) { return keysAt(optKeys(k), t, dflt); }));
  static constexpr auto keysIn = [](val v, int samples) -> val {
    if (v.isUndefined() || v.isNull()) return v;
    KeysInput in = v.isNumber()          ? KeysInput(v.as<double>())
                   : isFunction(v)       ? KeysInput(waveOf(v))
                   : count(v) == 0 || !v[0].isNumber() ? KeysInput(keysOf(v))
                                         : KeysInput(numsOf(v));
    return optKeys(keysOf(std::optional<KeysInput>(in), samples));
  };
  function("keysOf", optional_override([](val v) { return keysIn(v, 65); }));
  function("keysOf", optional_override([](val v, int samples) { return keysIn(v, samples); }));
  function("keysTimes", optional_override([](val a, val b) -> val {
             auto k = keysTimes(optKeys(a), optKeys(b));
             // Both left out gives back what was passed, as the TS does.
             if (!k && (a.isUndefined() || a.isNull()) && (b.isUndefined() || b.isNull())) return b.isNull() || a.isNull() ? val::null() : val::undefined();
             return optKeys(k);
           }));
  function("sorted", optional_override([](val k) { return optKeys(sorted(optKeys(k))); }));
  function("spread", optional_override([](val vs) { return keys(spread(numsOf(vs))); }));

  // shape
  function("soft", optional_override([](val ps) { return soft(ptsOf(ps)); }));
  static constexpr auto softness = [](val o) {
    Softness s;
    if (auto r = optNum(o, "radius")) s.radius = *r;
    if (has(o, "bulge")) s.bulge = o["bulge"].isNumber() ? decltype(s.bulge)(o["bulge"].as<double>()) : decltype(s.bulge)(numsOf(o["bulge"]));
    return s;
  };
  function("soft", optional_override([](val ps, val o) { return soft(ptsOf(ps), softness(o)); }));
  function("soft", optional_override([](val ps, val o, val m) { return soft(ptsOf(ps), softness(o), pt(m)); }));
  static constexpr auto ovalOpts = [](val o) {
    OvalOptions v;
    if (auto t = optNum(o, "tension")) v.tension = *t;
    if (has(o, "start")) {
      std::string s = o["start"].as<std::string>();
      v.start = s == "right" ? Side::right : s == "top" ? Side::top : s == "left" ? Side::left : Side::bottom;
    }
    return v;
  };
  function("oval", optional_override([](double x0, double y0, double x1, double y1) { return oval(x0, y0, x1, y1); }));
  function("oval", optional_override([](double x0, double y0, double x1, double y1, val o) {
             return oval(x0, y0, x1, y1, ovalOpts(o));
           }));
  static constexpr auto lobeOpts = [](val o) {
    LobeOptions l;
    if (auto r = optNum(o, "reach")) l.reach = *r;
    if (auto n = optNum(o, "nodes")) l.nodes = *n;
    if (auto r = optNum(o, "radius")) l.radius = *r;
    if (auto b = optNum(o, "bulge")) l.bulge = *b;
    l.axis = optNum(o, "axis");
    if (has(o, "reachFrom") && o["reachFrom"].as<std::string>() == "start") l.reachFrom = ReachFrom::start;
    return l;
  };
  function("lobe", optional_override([](val a, val b) { return lobe(pt(a), pt(b)); }));
  function("lobe", optional_override([](val a, val b, val o) { return lobe(pt(a), pt(b), lobeOpts(o)); }));
  function("lobe", optional_override([](val a, val b, val o, val m) {
             return lobe(pt(a), pt(b), lobeOpts(o), pt(m));
           }));

  // warp: warps and waves made here come back as JS functions
  function("meshGrid", optional_override([](val lo, val hi) { return pts(meshGrid(pt(lo), pt(hi))); }));
  function("mesh", optional_override([](val lo, val hi, val grid) { return js(mesh(pt(lo), pt(hi), ptsOf(grid))); }));
  function("spine", optional_override([](val dx, double y0, double y1) {
             std::variant<double, Wave> d = dx.isNumber() ? std::variant<double, Wave>(dx.as<double>()) : waveOf(dx);
             return js(spine(d, y0, y1));
           }));
  function("pinned", optional_override([](val f, val ys, double reach) { return js(pinned(warpOf(f), numsOf(ys), reach)); }));
  static constexpr auto warpOpts = [](val o) {
    WarpOptions w;
    if (auto p = optNum(o, "pieces")) w.pieces = int(*p);
    if (auto s = optNum(o, "step")) w.step = *s;
    if (auto e = optNum(o, "eps")) w.eps = *e;
    return w;
  };
  function("warpContours", optional_override([](val cs, val f) { return contours(warpContours(contoursOf(cs), warpOf(f))); }));
  function("warpContours", optional_override([](val cs, val f, val o) {
             return contours(warpContours(contoursOf(cs), warpOf(f), warpOpts(o)));
           }));
  function("warpNode", optional_override([](val n, val f) { return js(warpNode(node(n), warpOf(f))); }));
  function("warpNode", optional_override([](val n, val f, double eps) { return js(warpNode(node(n), warpOf(f), eps)); }));
  function("wavelength", optional_override([](val w) { return wavelength(wobbleOf(w)); }));
  function("wobble", optional_override([](val w) { return js(wobble(wobbleOf(w))); }));
  function("wobbleStep", optional_override([](val w) { return wobbleStep(wobbleOf(w)); }));
  function("wobbleWave", optional_override([](val w, double length) { return js(wobbleWave(wobbleOf(w), length)); }));
  function("wobbleWave", optional_override([](val w, double length, bool closed) {
             return js(wobbleWave(wobbleOf(w), length, closed));
           }));

  // attach
  static constexpr auto pairsOf = [](val ps) {
    std::vector<Pair> out;
    for (int i = 0, n = count(ps); i < n; i++)
      out.push_back({targetOf(ps[i]), has(ps[i], "from") ? std::optional<double>(ps[i]["from"].as<double>()) : std::nullopt});
    return out;
  };
  function("place", optional_override([](const Path& p, val pairs) { return place(p, pairsOf(pairs)); }));
  function("place", optional_override([](const Path& p, val pairs, val a) {
             if (a.isUndefined() || a.isNull()) return place(p, pairsOf(pairs));
             AlignTo to = has(a, "to") && a["to"].as<std::string>() == "normal" ? AlignTo::normal : AlignTo::tangent;
             Align al{a["on"].as<Path>(), a["at"].as<double>(), optSegs(a), to, optNum(a, "angle"), optNum(a, "from")};
             return place(p, pairsOf(pairs), al);
           }));

  // bool
  class_<Bool>("Bool")
      .constructor(optional_override([](std::string op, val operands) { return Bool(opOf(op), operandsOf(operands)); }))
      .constructor(optional_override([](std::string op, val operands, double eps) {
        return Bool(opOf(op), operandsOf(operands), eps);
      }))
      .property("op", optional_override([](const Bool& b) {
                  static const char* names[] = {"union", "intersect", "subtract", "exclude", "cut"};
                  return std::string(names[int(b.op())]);
                }))
      .property("eps", &Bool::eps)
      .function("with", optional_override([](const Bool& b, val operands) { return b.with(operandsOf(operands)); }))
      .property("ok", &Bool::ok)
      .property("contours", optional_override([](const Bool& b) { return contours(b.contours()); }))
      .property("sources", optional_override([](const Bool& b) {
                  return list(b.sources(), [](const std::vector<int>& s) {
                    return list(s, [](int i) { return val(i); });
                  });
                }))
      .function("corners", optional_override([](const Bool& b) { return boolCorners(b.corners()); }))
      .function("corners", optional_override([](const Bool& b, val a) { return boolCorners(b.corners(optInt(a))); }))
      .function("corners", optional_override([](const Bool& b, val a, val c) {
                  return boolCorners(b.corners(optInt(a), optInt(c)));
                }))
      .function("corners", optional_override([](const Bool& b, val a, val c, val o) {
                  return boolCorners(b.corners(optInt(a), optInt(c), {has(o, "same") && o["same"].as<bool>()}));
                }))
      .function("outward", optional_override([](const Bool& b, int c, int n) { return js(b.outward(c, n)); }));
}

// The rest of src/curves: the functions the TS uses only inside its own files, bound so that every export of the TS
// is reachable. Where two TS files export one name (analysis's `project(path, p)` and bezier's `project(c, p)`, …),
// one binding takes both, a Path first argument picking the path's.

namespace {

bool isPath(const val& v) { return v.instanceof(val::module_property("Path")); }
Box boxOf(const val& v) { return {pt(v[0]), pt(v[1])}; }
std::optional<double> numOr(const val& v) {
  return v.isUndefined() || v.isNull() ? std::nullopt : std::optional<double>(v.as<double>());
}
bool flag(const val& o, const char* key, bool dflt) { return has(o, key) ? o[key].as<bool>() : dflt; }
val ints(const std::vector<int>& xs) {
  return list(xs, [](int i) { return val(i); });
}
std::vector<int> intsOf(const val& v) { return emscripten::vecFromJSArray<int>(v); }
std::vector<std::vector<double>> rowsOf(const val& v) {
  std::vector<std::vector<double>> rows;
  for (int i = 0, n = count(v); i < n; i++) rows.push_back(numsOf(v[i]));
  return rows;
}
std::vector<std::vector<Contour>> groupsOf(const val& v) {
  std::vector<std::vector<Contour>> gs;
  for (int i = 0, n = count(v); i < n; i++) gs.push_back(contoursOf(v[i]));
  return gs;
}
Reach reachOf(const val& v) { return v.isNumber() ? Reach(v.as<double>()) : Reach(waveOf(v)); }
Strength strengthOf(const val& v) {
  if (v.isNumber()) return v.as<double>();
  if (isFunction(v)) return waveOf(v);
  return numsOf(v);
}
Swing swingOf(const val& o) {
  Swing s;
  if (auto x = optNum(o, "cycles")) s.cycles = *x;
  if (auto x = optNum(o, "phase")) s.phase = *x;
  if (auto x = optNum(o, "low")) s.low = *x;
  if (auto x = optNum(o, "high")) s.high = *x;
  return s;
}
SquareSwing squareOf(const val& o) {
  SquareSwing s;
  static_cast<Swing&>(s) = swingOf(o);
  if (auto x = optNum(o, "duty")) s.duty = *x;
  return s;
}
Rule ruleOf(const val& f) {
  return [f](const std::vector<int>& w) { return f(ints(w)).as<bool>(); };
}
double epsOf(const val& o, double dflt) { return optNum(o, "eps").value_or(dflt); }
Mount mountOf(const val& m) {
  return {m["on"].as<Path>(), m["at"].as<double>(), optNum(m, "from"), optSegs(m)};
}
Softness softnessOf(const val& o) {
  Softness s;
  if (auto r = optNum(o, "radius")) s.radius = *r;
  if (has(o, "bulge"))
    s.bulge = o["bulge"].isNumber() ? decltype(s.bulge)(o["bulge"].as<double>()) : decltype(s.bulge)(numsOf(o["bulge"]));
  return s;
}
WarpOptions warpOptsOf(const val& o) {
  WarpOptions w;
  if (auto p = optNum(o, "pieces")) w.pieces = int(*p);
  if (auto s = optNum(o, "step")) w.step = *s;
  if (auto e = optNum(o, "eps")) w.eps = *e;
  return w;
}
CatmullRomOptions catmullOf(const val& o) {
  CatmullRomOptions c;
  if (auto x = optNum(o, "alpha")) c.alpha = *x;
  if (auto x = optNum(o, "tension")) c.tension = *x;
  c.closed = flag(o, "closed", false);
  if (has(o, "metric")) c.metric = pt(o["metric"]);
  return c;
}
BSplineOptions bsplineOf(const val& o) {
  BSplineOptions b;
  if (auto x = optNum(o, "degree")) b.degree = int(*x);
  if (has(o, "knots")) b.knots = numsOf(o["knots"]);
  if (has(o, "weights")) b.weights = numsOf(o["weights"]);
  b.closed = flag(o, "closed", false);
  if (auto x = optNum(o, "tol")) b.tol = *x;
  if (has(o, "metric")) b.metric = pt(o["metric"]);
  return b;
}
SelectAtOptions selectAtOf(const val& o) {
  SelectAtOptions s;
  if (has(o, "falloff")) s.falloff = falloffOf(o["falloff"]);
  if (has(o, "along")) s.along = o["along"].as<std::string>() == "plane" ? Along::plane : Along::path;
  s.cut = flag(o, "cut", true);
  return s;
}
Way wayOf(const std::string& s) {
  return s == "left" ? Way::left : s == "right" ? Way::right : s == "top" ? Way::top : Way::bottom;
}

val osculating(const Osculating& o) {
  val r = val::object();
  if (o.center) r.set("center", js(*o.center));
  r.set("r", o.r);
  r.set("k", o.k);
  return r;
}
val arc(const Arc& a) {
  val o = val::object();
  if (a.center) o.set("center", js(*a.center));
  o.set("r", a.r);
  o.set("start", a.start);
  o.set("end", a.end);
  o.set("ccw", a.ccw);
  o.set("from", js(a.from));
  o.set("to", js(a.to));
  o.set("t0", a.t0);
  o.set("t1", a.t1);
  return o;
}
val gap(const Gap& g) { return obj({{"d", val(g.d)}, {"a", js(g.a)}, {"b", js(g.b)}}); }
val ptsList(const std::vector<std::vector<Pt>>& ls) { return list(ls, pts); }

// A function of two numbers made here (noise2) goes out as a JS function, like warps and waves.
struct Fn2 {
  std::function<double(double, double)> f;
  double call(double x, double y) const { return f(x, y); }
};

}  // namespace

EMSCRIPTEN_BINDINGS(curves_rest) {
  using emscripten::class_;
  using emscripten::function;
  using emscripten::optional_override;

  class_<Fn2>("Fn2").function("call", &Fn2::call);

  // geometry
  emscripten::constant("KAPPA", KAPPA);
  emscripten::constant("identity", affine(identity));
  function("neg", optional_override([](val v) { return js(-pt(v)); }));
  function("fromAngle", optional_override([](double a) { return js(fromAngle(a)); }));
  function("mean", optional_override([](val ps) { return js(mean(ptsOf(ps))); }));
  function("bezDD", optional_override([](val c, double t) { return js(bezDD(cubic(c), t)); }));
  static constexpr auto meetOf = [](val p, val d, val q, val e, val o) {
    auto m = meet(pt(p), pt(d), pt(q), pt(e), optNum(o, "flat").value_or(1e-9), optNum(o, "inside"));
    return m ? nums({(*m)[0], (*m)[1]}) : val::undefined();
  };
  function("meet", optional_override([](val p, val d, val q, val e) { return meetOf(p, d, q, e, val::undefined()); }));
  function("meet", optional_override([](val p, val d, val q, val e, val o) { return meetOf(p, d, q, e, o); }));
  function("asCubic", optional_override([](val q) { return js(asCubic(piece(q))); }));
  function("reverse", optional_override([](val q) { return js(curves::reverse(piece(q))); }));
  function("endTangents", optional_override([](val q) {
             auto [a, b] = endTangents(piece(q));
             return pts({a, b});
           }));
  function("trim", optional_override([](val c, double t0, double t1) { return js(trim(cubic(c), t0, t1)); }));
  function("roots", optional_override([](val c, int coord, double v) { return nums(roots(cubic(c), coord, v)); }));
  function("roots", optional_override([](val c, int coord, double v, int steps) {
             return nums(roots(cubic(c), coord, v, steps));
           }));
  function("tAtLength", optional_override([](val c, double s) { return tAtLength(cubic(c), s); }));
  function("tAtLength", optional_override([](val c, double s, val total) { return tAtLength(cubic(c), s, numOr(total)); }));
  function("pieceLength", optional_override([](val q) { return pieceLength(piece(q)); }));
  function("stitch", optional_override([](val c) {
             Contour k = piecesOf(c);
             return pieces(stitch(k));
           }));
  function("arcCubic", optional_override([](val a, val ta, val b, val tb) {
             return js(arcCubic(pt(a), pt(ta), pt(b), pt(tb)));
           }));
  function("cutAt", optional_override([](val c, val shares) { return cubics(cutAt(cubic(c), numsOf(shares))); }));
  function("cutAt", optional_override([](val c, val shares, val m) {
             return cubics(cutAt(cubic(c), numsOf(shares), pt(m)));
           }));
  function("splitEven", optional_override([](val c, int n) { return cubics(splitEven(cubic(c), n)); }));
  function("runLengths", optional_override([](val ps) { return nums(runLengths(ptsOf(ps))); }));
  function("pointAlong", optional_override([](val ps, val run, double s) {
             return js(pointAlong(ptsOf(ps), numsOf(run), s));
           }));
  function("hullLength", optional_override([](val c) { return hullLength(cubic(c)); }));
  function("area", optional_override([](val ps) { return area(ptsOf(ps)); }));
  function("bounds", optional_override([](val ps) { return box(bounds(ptsOf(ps))); }));
  function("overlaps", optional_override([](val a, val b) { return overlaps(boxOf(a), boxOf(b)); }));
  function("overlaps", optional_override([](val a, val b, double e) { return overlaps(boxOf(a), boxOf(b), e); }));
  function("center", optional_override([](val ps) { return js(center(ptsOf(ps))); }));
  function("within", optional_override([](val b, val at) { return js(within(boxOf(b), pt(at))); }));
  function("toMetric", optional_override([](val p, val m) { return js(toMetric(pt(p), pt(m))); }));
  function("fromMetric", optional_override([](val p, val m) { return js(fromMetric(pt(p), pt(m))); }));
  function("then_", optional_override([](val m, val n) { return affine(curves::then(affineOf(m), affineOf(n))); }));
  function("shear", optional_override([](double d) { return affine(shear(d)); }));
  function("shear", optional_override([](double d, double axis) { return affine(shear(d, axis)); }));
  function("shear", optional_override([](double d, double axis, val p) { return affine(shear(d, axis, pt(p))); }));
  function("inMetric", optional_override([](val m, val metric) { return affine(inMetric(affineOf(m), pt(metric))); }));
  function("partly", optional_override([](val m, double w) { return affine(partly(affineOf(m), w)); }));

  // bezier
  function("cubicRoots", optional_override([](double a, double b, double c, double d) { return nums(cubicRoots(a, b, c, d)); }));
  function("inUnit", optional_override([](val ts) { return nums(inUnit(numsOf(ts))); }));
  function("inUnit", optional_override([](val ts, double eps) { return nums(inUnit(numsOf(ts), eps)); }));
  function("coordRoots", optional_override([](double p0, double p1, double p2, double p3) {
             return nums(coordRoots(p0, p1, p2, p3));
           }));
  function("coordRoots", optional_override([](double p0, double p1, double p2, double p3, double v) {
             return nums(coordRoots(p0, p1, p2, p3, v));
           }));
  function("cubicCoordRoots", optional_override([](val c, int coord, double v) { return nums(cubicCoordRoots(cubic(c), coord, v)); }));
  function("align", optional_override([](val c) { return js(align(cubic(c))); }));
  function("align", optional_override([](val c, val a) { return js(align(cubic(c), pt(a), cubic(c)[3])); }));
  function("align", optional_override([](val c, val a, val b) { return js(align(cubic(c), pt(a), pt(b))); }));
  function("tightBox", optional_override([](val c) {
             auto b = tightBox(cubic(c));
             return pts({b.begin(), b.end()});
           }));
  static constexpr auto canonicalOf = [](const Canonical& k) {
    return obj({{"kind", val(std::string(name(k.kind)))}, {"at", js(k.at)}});
  };
  function("canonical", optional_override([](val c) { return canonicalOf(canonical(cubic(c))); }));
  function("canonical", optional_override([](val c, double eps) { return canonicalOf(canonical(cubic(c), eps)); }));
  function("casteljau", optional_override([](val c, double t) {
             auto ps = casteljau(cubic(c), t);
             return pts({ps.begin(), ps.end()});
           }));
  function("lut", optional_override([](val c) { return pts(lut(cubic(c))); }));
  function("lut", optional_override([](val c, int steps) { return pts(lut(cubic(c), steps)); }));
  function("tangent", optional_override([](val c, double t) { return js(tangent(cubic(c), t)); }));
  function("quadThrough", optional_override([](val s, val b, val e) { return js(quadThrough(pt(s), pt(b), pt(e))); }));
  function("quadThrough", optional_override([](val s, val b, val e, val t) {
             return js(quadThrough(pt(s), pt(b), pt(e), numOr(t)));
           }));
  static constexpr auto throughOf = [](val o) {
    ThroughOptions t;
    t.t = optNum(o, "t");
    t.d1 = optNum(o, "d1");
    if (has(o, "dir")) t.dir = pt(o["dir"]);
    return t;
  };
  function("cubicThrough", optional_override([](val s, val b, val e) { return js(cubicThrough(pt(s), pt(b), pt(e))); }));
  function("cubicThrough", optional_override([](val s, val b, val e, val o) {
             return js(cubicThrough(pt(s), pt(b), pt(e), throughOf(o)));
           }));

  // bezier and analysis: one name for a cubic and a path
  function("project", optional_override([](val x, val p) {
             if (isPath(x)) return projection(project(x.as<Path>(), pt(p)));
             auto r = project(cubic(x), pt(p));
             return obj({{"t", val(r.t)}, {"p", js(r.p)}, {"d", val(r.d)}});
           }));
  function("extrema", optional_override([](val x) {
             if (isPath(x)) return nums(extrema(x.as<Path>()));
             auto e = extrema(cubic(x));
             return obj({{"x", nums(e.x)}, {"y", nums(e.y)}, {"values", nums(e.values)}});
           }));
  function("inflections", optional_override([](val x) {
             return nums(isPath(x) ? inflections(x.as<Path>()) : inflections(cubic(x)));
           }));
  function("bbox", optional_override([](val x) { return box(isPath(x) ? bbox(x.as<Path>()) : bbox(cubic(x))); }));
  function("curvature", optional_override([](val x, double t) {
             return isPath(x) ? curvature(x.as<Path>(), t) : curvature(cubic(x), t);
           }));
  function("normal", optional_override([](val x, double t) {
             return js(isPath(x) ? normal(x.as<Path>(), t) : normal(cubic(x), t));
           }));
  function("osculating", optional_override([](val x, double t) {
             return osculating(isPath(x) ? curves::osculating(x.as<Path>(), t) : curves::osculating(cubic(x), t));
           }));
  function("mold", optional_override([](val x, double t, val to) {
             return isPath(x) ? val(mold(x.as<Path>(), t, pt(to))) : js(mold(cubic(x), t, pt(to)));
           }));

  // analysis
  function("side", optional_override([](const Path& p, std::string way) { return side(p, wayOf(way)); }));
  static constexpr auto turnList = [](const std::vector<Turn>& ts) {
    return list(ts, [](const Turn& t) {
      return obj({{"at", val(t.at)}, {"p", js(t.p)}, {"turn", val(t.turn)}, {"tip", js(t.tip)}});
    });
  };
  function("turns", optional_override([](const Path& p) { return turnList(turns(p)); }));
  function("turns", optional_override([](const Path& p, double minDeg) { return turnList(turns(p, minDeg)); }));
  function("nearness", optional_override([](const Path& p, val ats, double reach) {
             return js(Wave(nearness(p, numsOf(ats), reach)));
           }));
  function("nearness", optional_override([](const Path& p, val ats, double reach, val f) {
             return js(Wave(nearness(p, numsOf(ats), reach, falloffOf(f))));
           }));
  function("selectAt", optional_override([](const Path& p, double at, double reach) { return selectAt(p, at, reach); }));
  function("selectAt", optional_override([](const Path& p, double at, double reach, val o) {
             return selectAt(p, at, reach, selectAtOf(o));
           }));

  // offset, and analysis's offset, outline and arcs of a path
  function("simple", optional_override([](val c) { return simple(cubic(c)); }));
  function("reduce", optional_override([](val c) {
             return list(reduce(cubic(c)), [](const Reduced& r) {
               return obj({{"c", js(r.c)}, {"t0", val(r.t0)}, {"t1", val(r.t1)}});
             });
           }));
  function("scale", optional_override([](val c, val d) { return js(scale(cubic(c), reachOf(d))); }));
  function("offset", optional_override([](val x, val d) {
             return isPath(x) ? val(offset(x.as<Path>(), reachOf(d))) : cubics(offset(cubic(x), reachOf(d)));
           }));
  function("offsetAt", optional_override([](val c, double t, double d) { return js(offsetAt(cubic(c), t, d)); }));
  function("outline", optional_override([](val x, double d1) {
             return isPath(x) ? contours(outline(x.as<Path>(), d1)) : pieces(outline(cubic(x), d1));
           }));
  function("outline", optional_override([](val x, double d1, val d2) {
             if (isPath(x)) return contours(outline(x.as<Path>(), d1, numOr(d2).value_or(d1)));
             return pieces(outline(cubic(x), d1, numOr(d2)));
           }));
  function("outline", optional_override([](val c, double d1, val d2, val d3) {
             return pieces(outline(cubic(c), d1, numOr(d2), numOr(d3)));
           }));
  function("outline", optional_override([](val c, double d1, val d2, val d3, val d4) {
             return pieces(outline(cubic(c), d1, numOr(d2), numOr(d3), numOr(d4)));
           }));
  function("outlineShapes", optional_override([](val c, double d1) { return contours(outlineShapes(cubic(c), d1)); }));
  function("outlineShapes", optional_override([](val c, double d1, val d2) {
             return contours(outlineShapes(cubic(c), d1, numOr(d2)));
           }));
  function("outlineShapes", optional_override([](val c, double d1, val d2, val d3) {
             return contours(outlineShapes(cubic(c), d1, numOr(d2), numOr(d3)));
           }));
  function("outlineShapes", optional_override([](val c, double d1, val d2, val d3, val d4) {
             return contours(outlineShapes(cubic(c), d1, numOr(d2), numOr(d3), numOr(d4)));
           }));
  static constexpr auto arcsOf = [](val x, double threshold) {
    if (!isPath(x)) return list(arcs(cubic(x), threshold), arc);
    return list(arcs(x.as<Path>(), threshold), [](const PathArc& a) {
      val o = arc(a);
      o.set("at0", a.at0);
      o.set("at1", a.at1);
      return o;
    });
  };
  function("arcs", optional_override([](val x) { return arcsOf(x, 0.5); }));
  function("arcs", optional_override([](val x, double threshold) { return arcsOf(x, threshold); }));

  // intersect, and analysis's crossings of a path
  static constexpr auto lineCrossingsOf = [](val x, val a, val b, val o) {
    bool segment = flag(o, "segment", true);
    if (isPath(x))
      return list(lineCrossings(x.as<Path>(), pt(a), pt(b), segment), [](const PathLineCrossing& c) {
        return obj({{"at", val(c.at)}, {"s", val(c.s)}, {"p", js(c.p)}});
      });
    return list(lineCrossings(cubic(x), pt(a), pt(b), segment), [](const LineCrossing& c) {
      return obj({{"t", val(c.t)}, {"s", val(c.s)}, {"p", js(c.p)}});
    });
  };
  function("lineCrossings", optional_override([](val x, val a, val b) { return lineCrossingsOf(x, a, b, val::undefined()); }));
  function("lineCrossings", optional_override([](val x, val a, val b, val o) { return lineCrossingsOf(x, a, b, o); }));
  static constexpr auto circleCrossingsOf = [](val x, val o, double r, int samples) {
    if (isPath(x))
      return list(circleCrossings(x.as<Path>(), pt(o), r), [](const PathCircleCrossing& c) {
        return obj({{"at", val(c.at)}, {"p", js(c.p)}});
      });
    return list(circleCrossings(cubic(x), pt(o), r, samples), [](const CircleCrossing& c) {
      return obj({{"t", val(c.t)}, {"p", js(c.p)}});
    });
  };
  function("circleCrossings", optional_override([](val x, val o, double r) { return circleCrossingsOf(x, o, r, 64); }));
  function("circleCrossings", optional_override([](val c, val o, double r, int samples) {
             return circleCrossingsOf(c, o, r, samples);
           }));
  static constexpr auto selfCrossingsOf = [](val x, bool closed) {
    if (isPath(x))
      return list(selfCrossings(x.as<Path>()), [](const PathSelfCrossing& c) {
        return obj({{"a", val(c.a)}, {"b", val(c.b)}, {"p", js(c.p)}});
      });
    return list(selfCrossings(cubicsOf(x), closed), [](const SelfCrossing& c) {
      return obj({{"a", obj({{"i", val(c.a.i)}, {"t", val(c.a.t)}})},
                  {"b", obj({{"i", val(c.b.i)}, {"t", val(c.b.t)}})},
                  {"p", js(c.p)}});
    });
  };
  function("selfCrossings", optional_override([](val x) { return selfCrossingsOf(x, false); }));
  function("selfCrossings", optional_override([](val segs, bool closed) { return selfCrossingsOf(segs, closed); }));

  // bezn
  function("bezierAt", optional_override([](val ps, double t) { return js(bezierAt(ptsOf(ps), t)); }));
  function("casteljauN", optional_override([](val ps, double t) { return ptsList(casteljauN(ptsOf(ps), t)); }));
  function("splitN", optional_override([](val ps, double t) {
             auto [a, b] = splitN(ptsOf(ps), t);
             return ptsList({a, b});
           }));
  function("derivativeN", optional_override([](val ps) { return pts(derivativeN(ptsOf(ps))); }));
  function("lower", optional_override([](val ps) { return pts(lower(ptsOf(ps))); }));
  function("fitBezier", optional_override([](val ps) { return pts(fitBezier(ptsOf(ps))); }));
  function("fitBezier", optional_override([](val ps, int order) { return pts(fitBezier(ptsOf(ps), order)); }));
  function("rationalAt", optional_override([](val ps, val ws, double t) { return js(rationalAt(ptsOf(ps), numsOf(ws), t)); }));
  static constexpr auto toCubicsOf = [](val ps, val o) {
    ToCubicsOptions t;
    if (has(o, "weights")) t.weights = numsOf(o["weights"]);
    if (auto x = optNum(o, "tol")) t.tol = *x;
    if (auto x = optNum(o, "samples")) t.samples = int(*x);
    return cubics(toCubics(ptsOf(ps), t));
  };
  function("toCubics", optional_override([](val ps) { return toCubicsOf(ps, val::undefined()); }));
  function("toCubics", optional_override([](val ps, val o) { return toCubicsOf(ps, o); }));
  function("bernstein", optional_override([](int n, double t) { return nums(bernstein(n, t)); }));
  function("solve", optional_override([](val a, val b) { return nums(solve(rowsOf(a), numsOf(b))); }));

  // boolean
  static constexpr auto crossingsOf = [](val a, val b, val o) {
    std::vector<SharedEnd> shared;
    if (has(o, "shared"))
      for (int i = 0, n = count(o["shared"]); i < n; i++) shared.push_back({o["shared"][i][0].as<int>(), o["shared"][i][1].as<int>()});
    auto r = crossings(cubic(a), cubic(b), optNum(o, "tol").value_or(1e-3), shared);
    if (!r) return val::null();
    return list(*r, [](const Crossing& c) { return nums({c[0], c[1]}); });
  };
  function("crossings", optional_override([](val a, val b) { return crossingsOf(a, b, val::undefined()); }));
  function("crossings", optional_override([](val a, val b, val o) { return crossingsOf(a, b, o); }));
  function("curveCrossings", optional_override([](val a, val b) { return crossingsOf(a, b, val::undefined()); }));
  function("curveCrossings", optional_override([](val a, val b, val o) { return crossingsOf(a, b, o); }));
  function("winding", optional_override([](val cs, val p) { return winding(contoursOf(cs), pt(p)); }));
  function("boolean", optional_override([](val in, val inside) { return contours(boolean(groupsOf(in), ruleOf(inside))); }));
  function("boolean", optional_override([](val in, val inside, val o) {
             return contours(boolean(groupsOf(in), ruleOf(inside), epsOf(o, 0.25)));
           }));
  static constexpr auto tryOf = [](val in, val inside, val o) {
    auto r = tryBoolean(groupsOf(in), ruleOf(inside), epsOf(o, 0.25));
    return r ? contours(*r) : val::null();
  };
  function("tryBoolean", optional_override([](val in, val inside) { return tryOf(in, inside, val::undefined()); }));
  function("tryBoolean", optional_override([](val in, val inside, val o) { return tryOf(in, inside, o); }));
  static constexpr auto traceOf = [](val in, val inside, val o) {
    auto r = traceBoolean(groupsOf(in), ruleOf(inside), epsOf(o, 0.25));
    if (!r) return val::null();
    return obj({{"contours", contours(r->contours)}, {"from", list(r->from, ints)}});
  };
  function("traceBoolean", optional_override([](val in, val inside) { return traceOf(in, inside, val::undefined()); }));
  function("traceBoolean", optional_override([](val in, val inside, val o) { return traceOf(in, inside, o); }));
  function("union", optional_override([](val cs) { return contours(unite(contoursOf(cs))); }));
  function("reverseContour", optional_override([](val c) { return pieces(reverseContour(piecesOf(c))); }));
  function("narrowest", optional_override([](val poly, double apart) {
             auto g = narrowest(ptsOf(poly), apart);
             return g ? gap(*g) : val::undefined();
           }));
  function("polyGap", optional_override([](val p, val q) { return gap(polyGap(ptsOf(p), ptsOf(q))); }));

  // corners
  function("corners", optional_override([](val cs) { return cornerList(corners(contoursOf(cs))); }));
  function("corners", optional_override([](val cs, double minDeg) { return cornerList(corners(contoursOf(cs), minDeg)); }));
  static constexpr auto replaceOf = [](val cs, val reach, val make, val o) {
    return contours(replaceCorners(
        contoursOf(cs), [reach](const Corner& k) { return reach(js(k)).as<double>(); },
        [make](Pt a, Pt ta, Pt b, Pt tb, const Corner& k) { return piecesOf(make(js(a), js(ta), js(b), js(tb), js(k))); },
        cornerOpts(o)));
  };
  function("replaceCorners", optional_override([](val cs, val reach, val make) {
             return replaceOf(cs, reach, make, val::undefined());
           }));
  function("replaceCorners", optional_override([](val cs, val reach, val make, val o) { return replaceOf(cs, reach, make, o); }));

  // nodes
  function("segmentOf", optional_override([](val a, val b) { return js(segmentOf(node(a), node(b))); }));
  function("shiftNodes", optional_override([](val ns, val d) { return nodes(shiftNodes(nodesOf(ns), pt(d))); }));
  function("shiftNodes", optional_override([](val ns, val d, val pick) {
             return nodes(shiftNodes(nodesOf(ns), pt(d), [pick](int i) { return pick(i).as<bool>(); }));
           }));

  // selection
  {
    static const std::pair<const char*, FalloffName> names[] = {
        {"smooth", FalloffName::smooth}, {"linear", FalloffName::linear}, {"sphere", FalloffName::sphere},
        {"root", FalloffName::root},     {"sharp", FalloffName::sharp},   {"constant", FalloffName::constant}};
    val fs = val::object();
    for (auto& [n, f] : names) fs.set(n, js(Wave(falloff(f))));
    emscripten::constant("falloffs", fs);
  }
  function("moveNodes", optional_override([](val segs, bool closed, val ns, val f) {
             return cubics(moveNodes(cubicsOf(segs), closed, intsOf(ns), [f](Pt p, int i) { return pt(f(js(p), i)); }));
           }));
  static constexpr auto smoothOf = [](val segs, bool closed, val ns, val amount, Pt metric) {
    if (isFunction(amount))
      return cubics(smoothNodes(cubicsOf(segs), closed, intsOf(ns),
                                std::function<double(int)>([amount](int i) { return amount(i).as<double>(); }), metric));
    return cubics(smoothNodes(cubicsOf(segs), closed, intsOf(ns), numOr(amount).value_or(1), metric));
  };
  function("smoothNodes", optional_override([](val segs, bool closed, val ns) {
             return smoothOf(segs, closed, ns, val::undefined(), {1, 1});
           }));
  function("smoothNodes", optional_override([](val segs, bool closed, val ns, val amount) {
             return smoothOf(segs, closed, ns, amount, {1, 1});
           }));
  function("smoothNodes", optional_override([](val segs, bool closed, val ns, val amount, val m) {
             return smoothOf(segs, closed, ns, amount, pt(m));
           }));
  function("inflation", optional_override([](val p, val from, double r) { return js(inflation(pt(p), pt(from), r)); }));
  static constexpr auto fairOf = [](val segs, bool closed, val ns, val reach, val m, val ws) {
    auto f = fairNodes(cubicsOf(segs), closed, intsOf(ns), numOr(reach).value_or(0.5),
                       m.isUndefined() ? Pt{1, 1} : pt(m),
                       ws.isUndefined() ? std::nullopt : std::optional<std::vector<double>>(numsOf(ws)));
    return obj({{"segs", cubics(f.segs)}, {"nodes", ints(f.nodes)}, {"weights", nums(f.weights)}});
  };
  function("fairNodes", optional_override([](val segs, bool closed, val ns) {
             return fairOf(segs, closed, ns, val::undefined(), val::undefined(), val::undefined());
           }));
  function("fairNodes", optional_override([](val segs, bool closed, val ns, val reach) {
             return fairOf(segs, closed, ns, reach, val::undefined(), val::undefined());
           }));
  function("fairNodes", optional_override([](val segs, bool closed, val ns, val reach, val m) {
             return fairOf(segs, closed, ns, reach, m, val::undefined());
           }));
  function("fairNodes", optional_override([](val segs, bool closed, val ns, val reach, val m, val ws) {
             return fairOf(segs, closed, ns, reach, m, ws);
           }));

  // profile
  function("constant", optional_override([](double v) { return js(curves::constant(v)); }));
  function("gradient", optional_override([](val vs) { return js(gradient(numsOf(vs))); }));
  function("steps", optional_override([](val vs) { return js(curves::steps(numsOf(vs))); }));
  function("sine", optional_override([]() { return js(sine()); }));
  function("sine", optional_override([](val o) { return js(sine(swingOf(o))); }));
  function("cosine", optional_override([]() { return js(cosine()); }));
  function("cosine", optional_override([](val o) { return js(cosine(swingOf(o))); }));
  function("square", optional_override([]() { return js(square()); }));
  function("square", optional_override([](val o) { return js(square(squareOf(o))); }));
  function("triangle", optional_override([]() { return js(triangle()); }));
  function("triangle", optional_override([](val o) { return js(triangle(swingOf(o))); }));
  function("saw", optional_override([]() { return js(saw()); }));
  function("saw", optional_override([](val o) { return js(saw(swingOf(o))); }));
  function("wave", optional_override([](val s) { return js(wave(strengthOf(s))); }));
  // times and plus take any number of strengths in the TS; here up to four.
  function("times", optional_override([]() { return js(times({})); }));
  function("times", optional_override([](val a) { return js(times({strengthOf(a)})); }));
  function("times", optional_override([](val a, val b) { return js(times({strengthOf(a), strengthOf(b)})); }));
  function("times", optional_override([](val a, val b, val c) {
             return js(times({strengthOf(a), strengthOf(b), strengthOf(c)}));
           }));
  function("times", optional_override([](val a, val b, val c, val d) {
             return js(times({strengthOf(a), strengthOf(b), strengthOf(c), strengthOf(d)}));
           }));
  function("plus", optional_override([]() { return js(plus({})); }));
  function("plus", optional_override([](val a) { return js(plus({strengthOf(a)})); }));
  function("plus", optional_override([](val a, val b) { return js(plus({strengthOf(a), strengthOf(b)})); }));
  function("plus", optional_override([](val a, val b, val c) {
             return js(plus({strengthOf(a), strengthOf(b), strengthOf(c)}));
           }));
  function("plus", optional_override([](val a, val b, val c, val d) {
             return js(plus({strengthOf(a), strengthOf(b), strengthOf(c), strengthOf(d)}));
           }));
  function("bulge", optional_override([](double mid) { return keys(bulge(mid)); }));
  function("bulge", optional_override([](double mid, double ends) { return keys(bulge(mid, ends)); }));

  // shape
  function("bowed", optional_override([](val a, val b) { return js(bowed(pt(a), pt(b))); }));
  function("bowed", optional_override([](val a, val b, double bulge) { return js(bowed(pt(a), pt(b), bulge)); }));
  function("fitBulges", optional_override([](val ps, val bs) { return nums(fitBulges(ptsOf(ps), numsOf(bs))); }));
  function("softPolyline", optional_override([](val ps) { return cubics(softPolyline(ptsOf(ps))); }));
  function("softPolyline", optional_override([](val ps, val o) { return cubics(softPolyline(ptsOf(ps), softnessOf(o))); }));

  // spline
  function("catmullRom", optional_override([](val ps) { return cubics(catmullRom(ptsOf(ps))); }));
  function("catmullRom", optional_override([](val ps, val o) { return cubics(catmullRom(ptsOf(ps), catmullOf(o))); }));
  function("catmullRomPath", optional_override([](val ps) { return catmullRomPath(ptsOf(ps)); }));
  function("catmullRomPath", optional_override([](val ps, val o) { return catmullRomPath(ptsOf(ps), catmullOf(o)); }));
  function("deBoor", optional_override([](val ps, double u) { return js(deBoor(ptsOf(ps), u)); }));
  function("deBoor", optional_override([](val ps, double u, val o) { return js(deBoor(ptsOf(ps), u, bsplineOf(o))); }));
  function("bspline", optional_override([](val ps) { return cubics(bspline(ptsOf(ps))); }));
  function("bspline", optional_override([](val ps, val o) { return cubics(bspline(ptsOf(ps), bsplineOf(o))); }));
  function("bsplinePath", optional_override([](val ps) { return bsplinePath(ptsOf(ps)); }));
  function("bsplinePath", optional_override([](val ps, val o) { return bsplinePath(ptsOf(ps), bsplineOf(o)); }));

  // morph
  static constexpr auto alikeOf = [](const Path& a, const Path& b, double eps) {
    Alike r = alike(a, b, eps);
    return obj({{"from", val(r.from)}, {"to", val(r.to)}, {"matched", list(r.matched, [](bool m) { return val(m); })}});
  };
  function("alike", optional_override([](const Path& a, const Path& b) { return alikeOf(a, b, 1e-6); }));
  function("alike", optional_override([](const Path& a, const Path& b, val o) { return alikeOf(a, b, epsOf(o, 1e-6)); }));
  function("morph", optional_override([](const Path& a, const Path& b, double k) { return morph(a, b, k); }));
  function("morph", optional_override([](const Path& a, const Path& b, double k, val o) {
             return morph(a, b, k, epsOf(o, 1e-6));
           }));
  function("between", optional_override([](const Path& a, const Path& b, double k) { return between(a, b, k); }));
  function("between", optional_override([](const Path& a, const Path& b, double k, double eps) {
             return between(a, b, k, eps);
           }));

  // warp
  function("jacobian", optional_override([](val f, val p) {
             auto [a, b] = jacobian(warpOf(f), pt(p));
             return pts({a, b});
           }));
  function("jacobian", optional_override([](val f, val p, double eps) {
             auto [a, b] = jacobian(warpOf(f), pt(p), eps);
             return pts({a, b});
           }));
  function("warpCubic", optional_override([](val c, val f) { return cubics(warpCubic(cubic(c), warpOf(f))); }));
  function("warpCubic", optional_override([](val c, val f, val o) {
             return cubics(warpCubic(cubic(c), warpOf(f), warpOptsOf(o)));
           }));
  function("warpCubics", optional_override([](val cs, val f) { return cubics(warpCubics(cubicsOf(cs), warpOf(f))); }));
  function("warpCubics", optional_override([](val cs, val f, val o) {
             return cubics(warpCubics(cubicsOf(cs), warpOf(f), warpOptsOf(o)));
           }));
  function("noise2", optional_override([](double seed) { return closure(val(Fn2{noise2(seed)})); }));

  // attach
  function("attach", optional_override([](const Path& p, val a) { return attach(p, mountOf(a)); }));
  function("attach", optional_override([](const Path& p, val a, val b) {
             return b.isUndefined() ? attach(p, mountOf(a)) : attach(p, mountOf(a), mountOf(b));
           }));
  function("target", optional_override([](val t) { return js(target(targetOf(t))); }));

  // mods
  function("expr", optional_override([](std::string code, val value) { return obj({{"expr", val(code)}, {"value", value}}); }));
  function("mod", optional_override([](std::string name) {
             mod(name);
             val ms = modList();
             for (int i = 0, n = count(ms); i < n; i++)
               if (ms[i]["name"].as<std::string>() == name) return ms[i];
             return val::undefined();
           }));
}

#endif
