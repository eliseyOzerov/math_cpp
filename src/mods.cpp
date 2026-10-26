#include "curves/mods.hpp"

#include "js.hpp"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#pragma STDC FP_CONTRACT OFF

namespace curves {

Arg Arg::of(double n) {
  Arg a;
  a.kind = Kind::number;
  a.number = n;
  return a;
}
Arg Arg::of(std::string s) {
  Arg a;
  a.kind = Kind::string;
  a.string = std::move(s);
  return a;
}
Arg Arg::list(std::vector<Arg> items) {
  Arg a;
  a.kind = Kind::list;
  a.items = std::move(items);
  return a;
}
Arg Arg::object(std::vector<std::pair<std::string, Arg>> entries) {
  Arg a;
  a.kind = Kind::object;
  for (auto& [k, v] : entries) a.keys.push_back(k), a.items.push_back(std::move(v));
  return a;
}
Arg Arg::of(Wave w) {
  Arg a;
  a.kind = Kind::wave;
  a.wave = std::move(w);
  return a;
}
const Arg& Arg::operator[](const std::string& key) const {
  static const Arg none;
  for (size_t i = 0; i < keys.size(); i++)
    if (keys[i] == key) return items[i];
  return none;
}

Arg expr(std::string code, Arg value) {
  Arg a;
  a.kind = Arg::Kind::expr;
  a.string = std::move(code);
  a.items.push_back(std::move(value));
  return a;
}

namespace {

/** An argument as passed: an expression's value, inside objects too. */
Arg value(const Arg& a) {
  if (a.kind == Arg::Kind::expr) return a.items[0];
  if (a.kind != Arg::Kind::object) return a;
  Arg o = a;
  for (auto& v : o.items) v = value(v);
  return o;
}

/** An argument as written: JSON, with expressions as their code and keys bare. */
std::string written(const Arg& a) {
  using K = Arg::Kind;
  switch (a.kind) {
    case K::expr: return a.string;
    case K::list: {
      std::string s = "[";
      for (size_t i = 0; i < a.items.size(); i++) s += (i ? ", " : "") + (a.items[i].isUndefined() ? "" : written(a.items[i]));
      return s + "]";
    }
    case K::object: {
      std::string s = "{ ";
      for (size_t i = 0; i < a.items.size(); i++) s += (i ? ", " : "") + a.keys[i] + ": " + written(a.items[i]);
      return s + " }";
    }
    case K::number: return js::string(std::isfinite(a.number) && std::fabs(a.number) < 1e21 ? std::strtod(js::toFixed(a.number, 4).c_str(), nullptr) : a.number);
    case K::string: return "'" + a.string + "'";
    case K::wave: return "{}";
    case K::undefined: return "undefined";
  }
  return "undefined";
}

Param num(double min, double max, double step, double value) { return NumParam{min, max, step, value}; }
Param choice(std::vector<std::string> options) {
  std::string first = options[0];
  return ChoiceParam{std::move(options), std::move(first)};
}

using Params = std::vector<std::pair<std::string, Param>>;

ParamValue paramDefault(const Param& p) {
  return std::holds_alternative<NumParam>(p) ? ParamValue(std::get<NumParam>(p).value) : ParamValue(std::get<ChoiceParam>(p).value);
}

Arg argOf(const ParamValue& v) {
  return std::holds_alternative<double>(v) ? Arg::of(std::get<double>(v)) : Arg::of(std::get<std::string>(v));
}

/** The options object with the entries left at their defaults dropped; undefined when nothing is left. */
Arg opts(const Params& m, const Values& v, const std::vector<std::string>& keys) {
  std::vector<std::pair<std::string, Arg>> out;
  for (auto& k : keys) {
    auto it = v.find(k);
    if (it == v.end()) continue;
    for (auto& [n, p] : m)
      if (n == k && it->second != paramDefault(p)) out.emplace_back(k, argOf(it->second));
  }
  return out.empty() ? Arg() : Arg::object(std::move(out));
}

double N(const Values& v, const std::string& k) {
  auto it = v.find(k);
  return it == v.end() ? NAN : std::holds_alternative<double>(it->second) ? std::get<double>(it->second) : std::strtod(std::get<std::string>(it->second).c_str(), nullptr);
}
std::string S(const Values& v, const std::string& k) {
  auto it = v.find(k);
  return it == v.end() ? "undefined" : std::holds_alternative<std::string>(it->second) ? std::get<std::string>(it->second) : js::string(std::get<double>(it->second));
}

Wave waveOf(const std::string& name, double cycles) {
  Swing s;
  s.cycles = cycles;
  if (name == "sine") return sine(s);
  if (name == "cosine") return cosine(s);
  if (name == "triangle") return triangle(s);
  if (name == "saw") return saw(s);
  SquareSwing q;
  q.cycles = cycles;
  return square(q);
}

/** The wave `name` with `cycles` cycles as an argument, or undefined for an even strength. */
Arg waveArg(const std::string& name, double cycles) {
  return name == "even" ? Arg() : expr(name + "({ cycles: " + js::string(cycles) + " })", Arg::of(waveOf(name, cycles)));
}

const Params& P(const std::string& name) {
  static const std::map<std::string, Params> ps = {
    {"soften", {{"reach", num(0, 1.5, 0.01, 0.4)}, {"falloff", choice({"smooth", "linear", "sphere", "root", "sharp", "constant"})}, {"along", choice({"path", "plane"})}}},
    {"weigh", {{"from", num(0, 1, 0.01, 1)}, {"to", num(0, 1, 0.01, 0)}}},
    {"invert", {}},
    {"shift", {{"dist", num(-0.5, 0.5, 0.01, 0.15)}, {"axis", num(-180, 180, 1, 90)}}},
    {"put", {{"x", num(-0.5, 1.5, 0.01, 0.5)}, {"y", num(-0.5, 1.5, 0.01, 0.5)}, {"along", choice({"y", "x", "both"})}}},
    {"rotate", {{"deg", num(-180, 180, 1, 30)}}},
    {"scale", {{"x", num(-2, 3, 0.01, 1.5)}, {"y", num(-2, 3, 0.01, 1.5)}}},
    {"shear", {{"deg", num(-60, 60, 1, 20)}, {"axis", num(0, 180, 1, 0)}}},
    {"mirror", {{"axis", num(0, 180, 1, 90)}}},
    {"kind", {{"kind", choice({"smooth", "symmetric", "corner"})}}},
    {"inflate", {{"r", num(-0.4, 0.4, 0.01, 0.1)}, {"handles", num(0, 1, 0.01, 1)}, {"smooth", num(0, 1, 0.01, 0)}, {"wave", choice({"even", "sine", "cosine", "square", "triangle", "saw"})}, {"cycles", num(1, 8, 1, 3)}, {"along", choice({"ring", "path"})}}},
    {"handles", {{"aim", choice({"turn", "angle"})}, {"deg", num(-180, 180, 1, 0)}, {"length", num(0, 3, 0.01, 1)}, {"side", choice({"both", "in", "out"})}}},
    {"smooth", {{"amount", num(0, 1, 0.01, 1)}}},
    {"fair", {{"reach", num(0, 1, 0.01, 0.5)}}},
  };
  return ps.at(name);
}

// Reading passed arguments into the typed calls: a number, or the default when undefined.
double numOr(const Arg& a, double d) { return a.isUndefined() ? d : a.number; }
const Arg& at(const std::vector<Arg>& a, size_t i) {
  static const Arg none;
  return i < a.size() ? a[i] : none;
}
std::optional<double> optNum(const Arg& a) { return a.isUndefined() ? std::nullopt : std::optional(a.number); }

FalloffName falloffName(const std::string& s) {
  return s == "smooth" ? FalloffName::smooth : s == "linear" ? FalloffName::linear : s == "sphere" ? FalloffName::sphere
       : s == "root" ? FalloffName::root : s == "sharp" ? FalloffName::sharp : FalloffName::constant;
}
NodeKind nodeKind(const std::string& s) { return s == "corner" ? NodeKind::corner : s == "smooth" ? NodeKind::smooth : NodeKind::symmetric; }
HandleLead lead(const std::string& s) { return s == "in" ? HandleLead::in : s == "out" ? HandleLead::out : HandleLead::both; }

std::vector<Mod> made() {
  using V = const Values&;
  using A = const std::vector<Arg>&;
  using Sel = const Selection&;
  return {
    {"soften", "select", "spread the weights to the nodes within reach, by a falloff (proportional editing)", P("soften"),
     [](V v) { return std::vector{argOf(v.at("reach")), opts(P("soften"), v, {"falloff", "along"})}; },
     [](Sel s, A a) {
       SoftenOptions o;
       const Arg& f = at(a, 1)["falloff"];
       const Arg& al = at(a, 1)["along"];
       if (!f.isUndefined()) o.falloff = falloffName(f.string);
       if (!al.isUndefined()) o.along = al.string == "plane" ? Along::plane : Along::path;
       return s.soften(at(a, 0).number, o);
     }},
    {"weigh", "select", "taper the weights along the path, from its start to its end", P("weigh"),
     [](V v) { return std::vector{v.at("from") == v.at("to") ? argOf(v.at("from")) : Arg::list({argOf(v.at("from")), argOf(v.at("to"))})}; },
     [](Sel s, A a) {
       const Arg& w = at(a, 0);
       if (w.kind == Arg::Kind::list) return s.weigh(std::vector<double>{w.items[0].number, w.items[1].number});
       return s.weigh(w.number);
     }},
    {"invert", "select", "weight every node the other way round", P("invert"),
     [](V) { return std::vector<Arg>{}; },
     [](Sel s, A) { return s.invert(); }},
    {"shift", "move", "move along an axis (0 across, 90 up)", P("shift"),
     [](V v) { return std::vector{argOf(v.at("dist")), N(v, "axis") == 0 ? Arg() : argOf(v.at("axis"))}; },
     [](Sel s, A a) { return s.shift(at(a, 0).number, numOr(at(a, 1), 0)); }},
    {"put", "move", "move the middle to a place in the box (0…1 across and up), along y, x or both", P("put"),
     [](V v) {
       std::string along = S(v, "along"), x = S(v, "x"), y = S(v, "y");
       Arg a = along == "y" ? expr("y(" + y + ")", Arg::object({{"y", argOf(v.at("y"))}}))
             : along == "x" ? expr("x(" + x + ")", Arg::object({{"x", argOf(v.at("x"))}}))
                            : expr("p(" + x + ", " + y + ")", Arg::list({argOf(v.at("x")), argOf(v.at("y"))}));
       return std::vector{a};
     },
     [](Sel s, A a) {
       const Arg& to = at(a, 0);
       Place p;
       if (to.kind == Arg::Kind::list) p = {to.items[0].number, to.items[1].number};
       else p = {optNum(to["x"]), optNum(to["y"])};
       return s.put(p);
     }},
    {"rotate", "move", "turn about the selection's middle", P("rotate"),
     [](V v) { return std::vector{argOf(v.at("deg"))}; },
     [](Sel s, A a) { return s.rotate(at(a, 0).number); }},
    {"scale", "move", "scale about the selection's middle, across and up", P("scale"),
     [](V v) { return std::vector{v.at("x") == v.at("y") ? argOf(v.at("x")) : Arg::list({argOf(v.at("x")), argOf(v.at("y"))})}; },
     [](Sel s, A a) {
       const Arg& k = at(a, 0);
       if (k.kind == Arg::Kind::list) return s.scale(Pt{k.items[0].number, k.items[1].number});
       return s.scale(k.number);
     }},
    {"shear", "move", "lean the lines across an axis", P("shear"),
     [](V v) { return std::vector{argOf(v.at("deg")), opts(P("shear"), v, {"axis"})}; },
     [](Sel s, A a) {
       ShearOptions o;
       o.axis = numOr(at(a, 1)["axis"], 0);
       return s.shear(at(a, 0).number, o);
     }},
    {"mirror", "move", "mirror in the line at an angle through the middle", P("mirror"),
     [](V v) { return std::vector{N(v, "axis") == 90 ? Arg() : argOf(v.at("axis"))}; },
     [](Sel s, A a) { return s.mirror(numOr(at(a, 0), 90)); }},
    {"inflate", "shape", "push each node out from the middle by the same distance, under a wave", P("inflate"),
     [](V v) {
       // The wave and its cycles go in as one option, `strength`.
       Arg o = opts(P("inflate"), v, {"handles", "smooth", "along"}), w = waveArg(S(v, "wave"), N(v, "cycles"));
       if (!w.isUndefined()) {
         if (o.isUndefined()) o = Arg::object({});
         o.keys.push_back("strength"), o.items.push_back(w);
       }
       return std::vector{argOf(v.at("r")), o};
     },
     [](Sel s, A a) {
       const Arg& o = at(a, 1);
       InflateOptions io;
       io.handles = numOr(o["handles"], 1);
       io.smooth = numOr(o["smooth"], 0);
       if (!o["along"].isUndefined()) io.along = o["along"].string == "path" ? InflateAlong::path : InflateAlong::ring;
       if (!o["strength"].isUndefined()) io.strength = o["strength"].wave;
       return s.inflate(at(a, 0).number, io);
     }},
    {"kind", "shape", "set the nodes smooth (handles on one line), symmetric or corner", P("kind"),
     [](V v) { return std::vector{argOf(v.at("kind"))}; },
     [](Sel s, A a) { return s.kind(nodeKind(at(a, 0).string)); }},
    {"handles", "shape", "turn the handles by an angle or aim them at one, and scale their length", P("handles"),
     [](V v) {
       std::vector<std::pair<std::string, Arg>> o;
       if (S(v, "aim") == "angle") o.emplace_back("angle", argOf(v.at("deg")));
       else if (js::truthy(N(v, "deg"))) o.emplace_back("turn", argOf(v.at("deg")));
       if (N(v, "length") != 1) o.emplace_back("length", argOf(v.at("length")));
       if (S(v, "side") != "both") o.emplace_back("side", argOf(v.at("side")));
       return std::vector{Arg::object(std::move(o))};
     },
     [](Sel s, A a) {
       const Arg& o = at(a, 0);
       HandlesOptions h;
       h.angle = optNum(o["angle"]);
       h.turn = numOr(o["turn"], 0);
       h.length = numOr(o["length"], 1);
       if (!o["side"].isUndefined()) h.side = lead(o["side"].string);
       return s.handles(h);
     }},
    {"smooth", "shape", "turn and size the handles to run smoothly through the nodes", P("smooth"),
     [](V v) { return std::vector{N(v, "amount") == 1 ? Arg() : argOf(v.at("amount"))}; },
     [](Sel s, A a) { return s.smooth(numOr(at(a, 0), 1)); }},
    {"fair", "shape", "continuous radii where a line meets a curve, eased over a reach", P("fair"),
     [](V v) { return std::vector{N(v, "reach") == 0.5 ? Arg() : argOf(v.at("reach"))}; },
     [](Sel s, A a) { return s.fair(numOr(at(a, 0), 0.5)); }},
  };
}

/** The defaults of `m` with `v` over them. */
Values merged(const Mod& m, const Values& v) {
  Values all = modDefaults(m);
  for (auto& [k, x] : v) all[k] = x;
  return all;
}

}  // namespace

const std::vector<Mod>& mods() {
  static const std::vector<Mod> all = made();
  return all;
}

const Mod& mod(const std::string& name) {
  for (auto& m : mods())
    if (m.name == name) return m;
  throw std::invalid_argument("no modifier " + name);
}

Values modDefaults(const Mod& m) {
  Values v;
  for (auto& [k, p] : m.params) v[k] = paramDefault(p);
  return v;
}

Selection applyMod(const Selection& s, const Mod& m, const Values& v) {
  std::vector<Arg> passed;
  for (auto& a : m.args(merged(m, v))) passed.push_back(value(a));
  return m.call(s, passed);
}

std::string modCode(const Mod& m, const Values& v) {
  auto a = m.args(merged(m, v));
  while (!a.empty() && a.back().isUndefined()) a.pop_back();
  std::string s = "." + m.name + "(";
  for (size_t i = 0; i < a.size(); i++) s += (i ? ", " : "") + written(a[i]);
  return s + ")";
}

Selection chain(const Selection& s, const std::vector<Step>& steps) {
  Selection q = s;
  for (auto& st : steps) q = applyMod(q, mod(st.mod), st.values);
  return q;
}

std::string chainCode(const std::vector<Step>& steps) {
  std::string s;
  for (auto& st : steps) s += modCode(mod(st.mod), st.values);
  return s;
}

}  // namespace curves
