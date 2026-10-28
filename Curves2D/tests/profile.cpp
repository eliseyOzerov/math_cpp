// The golden cases of profile.ts: each TS function by name, with its arguments read and its result written as gen.ts
// does. Waves are functions, so the cases wrap them (sineAt, timesAt …) to sample them at a list of x.
#include "cases.hpp"

#include "curves/profile.hpp"

namespace {

using namespace golden;

std::vector<double> numbers(const Value& v) { return list<double>(v, [](const Value& e) { return e.num(); }); }
bool isNumber(const Value& v) { return std::holds_alternative<double>(v.v); }

// An options object `o` (a Value) into a Swing (and a duty).
SquareSwing swingOf(const Value& o) {
  SquareSwing s;
  if (!o.isObject()) return s;
  if (o.has("cycles")) s.cycles = o["cycles"].num();
  if (o.has("phase")) s.phase = o["phase"].num();
  if (o.has("low")) s.low = o["low"].num();
  if (o.has("high")) s.high = o["high"].num();
  if (o.has("duty")) s.duty = o["duty"].num();
  return s;
}
Swing plainSwing(const Value& o) { SquareSwing s = swingOf(o); return s; }

Wave specWave(const Value& s);
Strength strengthOf(const Value& s) {
  if (isNumber(s)) return s.num();
  if (s.isArray()) return numbers(s);
  return specWave(s);
}
Wave specWave(const Value& s) {
  const std::string& w = s["w"].str();
  if (w == "sine") return sine(plainSwing(s));
  if (w == "cosine") return cosine(plainSwing(s));
  if (w == "square") return square(swingOf(s));
  if (w == "triangle") return triangle(plainSwing(s));
  if (w == "saw") return saw(plainSwing(s));
  if (w == "gradient") return gradient(numbers(s["values"]));
  return steps(numbers(s["values"]));
}
std::vector<Strength> strengths(const Value& v) {
  std::vector<Strength> out;
  for (auto& e : v.arr()) out.push_back(strengthOf(e));
  return out;
}
Value sample(const Wave& w, const Value& xs) {
  json::Array a;
  for (auto& x : xs.arr()) a.push_back(out(w(x.num())));
  return a;
}

template <class V, class R>
Keys<V> keysIn(const Value& v, R read) {
  Keys<V> ks;
  for (auto& e : v.arr()) {
    Key<V> k{e[0].num(), read(e[1])};
    if (e.size() > 2) {
      Handles h;
      if (e[2].has("in")) h.in = pt(e[2]["in"]);
      if (e[2].has("out")) h.out = pt(e[2]["out"]);
      k.h = h;
    }
    ks.push_back(k);
  }
  return ks;
}
Keys<double> keysD(const Value& v) { return keysIn<double>(v, [](const Value& x) { return x.num(); }); }
Keys<std::vector<double>> keysV(const Value& v) { return keysIn<std::vector<double>>(v, numbers); }
std::optional<Keys<double>> optKeys(const Value& v) { return v.isNull() ? std::nullopt : std::optional(keysD(v)); }

Value outKeys(const Keys<double>& ks) {
  json::Array a;
  for (auto& k : ks) {
    json::Array e{out(k.t), out(k.v)};
    if (k.h) {
      json::Object h;
      if (k.h->in) h["in"] = out(*k.h->in);
      if (k.h->out) h["out"] = out(*k.h->out);
      e.push_back(h);
    }
    a.push_back(e);
  }
  return a;
}
Value outOptKeys(const std::optional<Keys<double>>& ks) { return ks ? outKeys(*ks) : Value(); }

std::vector<double> lerpVec(const std::vector<double>& a, const std::vector<double>& b, double u) {
  std::vector<double> r;
  for (size_t i = 0; i < a.size(); i++) r.push_back(a[i] + (b[i] - a[i]) * u);
  return r;
}

golden::Register reg("profile.json", {
  {"constantAt", [](auto& a) { return sample(constant(num(a[0])), a[1]); }},
  {"gradientAt", [](auto& a) { return sample(gradient(numbers(a[0])), a[1]); }},
  {"stepsAt", [](auto& a) { return sample(steps(numbers(a[0])), a[1]); }},
  {"sineAt", [](auto& a) { return sample(sine(plainSwing(a[0])), a[1]); }},
  {"cosineAt", [](auto& a) { return sample(cosine(plainSwing(a[0])), a[1]); }},
  {"squareAt", [](auto& a) { return sample(square(swingOf(a[0])), a[1]); }},
  {"triangleAt", [](auto& a) { return sample(triangle(plainSwing(a[0])), a[1]); }},
  {"sawAt", [](auto& a) { return sample(saw(plainSwing(a[0])), a[1]); }},
  {"waveAt", [](auto& a) { return sample(wave(strengthOf(a[0])), a[1]); }},
  {"timesAt", [](auto& a) { return sample(times(strengths(a[0])), a[1]); }},
  {"plusAt", [](auto& a) { return sample(plus(strengths(a[0])), a[1]); }},
  {"smoothstep", [](auto& a) { return out(smoothstep(num(a[0]))); }},
  {"keysAt", [](auto& a) {
     double d = opt(a, 2).value_or(0);
     return out(a[0].isNull() ? keysAt<double>(std::optional<Keys<double>>{}, num(a[1]), d) : keysAt(keysD(a[0]), num(a[1]), d));
   }},
  {"keysAtVec", [](auto& a) {
     return out(keysAt<std::vector<double>>(keysV(a[0]), num(a[1]), numbers(a[2]), lerpVec));
   }},
  {"keysAtMixed", [](auto& a) {
     return out(keysAt<std::vector<double>>(keysV(a[0]), num(a[1]), numbers(a[2]), lerpVec));
   }},
  {"sorted", [](auto& a) { return outOptKeys(sorted<double>(optKeys(a[0]))); }},
  {"spread", [](auto& a) { return outKeys(spread<double>(numbers(a[0]))); }},
  {"easeKeys", [](auto& a) { return outKeys(easeKeys(num(a[0]), num(a[1]), num(a[2]), num(a[3]))); }},
  {"bulge", [](auto& a) { return outKeys(a.size() > 1 ? bulge(num(a[0]), num(a[1])) : bulge(num(a[0]))); }},
  {"keysOf", [](auto& a) {
     const Value& v = a[0];
     std::optional<KeysInput> in;
     if (v.isNull()) {}
     else if (isNumber(v)) in = v.num();
     else if (v.size() == 0 || isNumber(v[0])) in = numbers(v);
     else in = keysD(v);
     return outOptKeys(keysOf(in));
   }},
  {"keysOfSpec", [](auto& a) {
     KeysInput in = [&]() -> KeysInput {
       Strength s = strengthOf(a[0]);
       if (auto f = std::get_if<Wave>(&s)) return *f;
       if (auto n = std::get_if<double>(&s)) return *n;
       return std::get<std::vector<double>>(s);
     }();
     return outOptKeys(a.size() > 1 && !a[1].isNull() ? keysOf(in, integer(a[1])) : keysOf(in));
   }},
  {"keysTimes", [](auto& a) { return outOptKeys(keysTimes(optKeys(a[0]), optKeys(a[1]))); }},
});

}  // namespace
