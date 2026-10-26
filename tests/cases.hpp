// What every golden test file shares: arguments read back as gen.ts wrote them, results written in the shape the TS
// returned, and the registry each tests/<file>.cpp adds its functions to (one per TS file, run against
// tests/golden/<file>.json by main.cpp).
#pragma once

#include "json.hpp"

#include "curves/curves.hpp"

#include <functional>
#include <map>
#include <optional>
#include <string>

namespace golden {

using namespace curves;
using json::Value;

// Arguments in, as gen.ts wrote them.
inline double num(const Value& v) { return v.num(); }
inline int integer(const Value& v) { return int(v.num()); }
inline bool flag(const Value& v) { return v.boolean(); }
inline Pt pt(const Value& v) { return {v[0].num(), v[1].num()}; }
inline Cubic cub(const Value& v) { return {pt(v[0]), pt(v[1]), pt(v[2]), pt(v[3])}; }
inline Piece piece(const Value& v) { return v.size() == 2 ? Piece::of(pt(v[0]), pt(v[1])) : Piece::of(cub(v)); }
inline Affine aff(const Value& v) { Affine m; for (int i = 0; i < 6; i++) m[i] = v[i].num(); return m; }
inline Box box2(const Value& v) { return {pt(v[0]), pt(v[1])}; }
template <class T>
std::vector<T> list(const Value& v, T (*f)(const Value&)) {
  std::vector<T> out;
  for (auto& e : v.arr()) out.push_back(f(e));
  return out;
}
inline Contour contour(const Value& v) { return list(v, piece); }
inline std::vector<Contour> contours(const Value& v) { return list(v, contour); }
/** The argument at `i`, or nothing where the TS call left it out. */
inline std::optional<double> opt(const Value& args, size_t i) {
  return i < args.size() && !args[i].isNull() ? std::optional(args[i].num()) : std::nullopt;
}
/** Option `k` of the object at argument `i`, or nothing. */
inline std::optional<double> optKey(const Value& args, size_t i, const std::string& k) {
  return i < args.size() && args[i].has(k) ? std::optional(args[i][k].num()) : std::nullopt;
}

// Results out, shaped as the TS returned them.
inline Value out(double d) { return json::number(d); }
inline Value out(int i) { return double(i); }
inline Value out(size_t i) { return double(i); }
inline Value out(bool b) { return b; }
inline Value out(const std::string& s) { return s; }
inline Value out(Pt p) { return json::Array{out(p.x), out(p.y)}; }
inline Value out(const Piece& q) {
  json::Array a;
  for (int k = 0; k < (q.line ? 2 : 4); k++) a.push_back(out(q.p[k]));
  return a;
}
template <class T, size_t N>
Value out(const std::array<T, N>& xs) { json::Array a; for (auto& x : xs) a.push_back(out(x)); return a; }
template <class T>
Value out(const std::vector<T>& xs) { json::Array a; for (auto& x : xs) a.push_back(out(x)); return a; }
template <class T>
Value out(const std::optional<T>& x) { return x ? out(*x) : Value(); }

using Fn = std::function<Value(const Value&)>;
using Fns = std::map<std::string, Fn>;

/** Every test file's functions, by the golden file they check (geometry.json …). */
inline std::map<std::string, Fns>& registry() { static std::map<std::string, Fns> r; return r; }
/** Adds a file's functions at start-up: `static golden::Register reg("fit.json", {...});`. */
struct Register {
  Register(const std::string& file, Fns fns) { registry()[file] = std::move(fns); }
};

}  // namespace golden
