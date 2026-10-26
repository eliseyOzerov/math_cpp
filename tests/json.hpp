// Just enough JSON for the golden tests: a value type, a parser for the files gen.ts writes, and a printer for
// failures. Not for use outside the tests.
#pragma once

#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace json {

struct Value;
using Array = std::vector<Value>;
using Object = std::map<std::string, Value>;

/** A JSON value; non-finite numbers travel as the strings "NaN", "Infinity" and "-Infinity", as gen.ts writes them. */
struct Value {
  std::variant<std::nullptr_t, bool, double, std::string, std::shared_ptr<Array>, std::shared_ptr<Object>> v;

  Value() : v(nullptr) {}
  Value(std::nullptr_t) : v(nullptr) {}
  Value(bool b) : v(b) {}
  Value(double d) : v(d) {}
  Value(int d) : v(double(d)) {}
  Value(std::string s) : v(std::move(s)) {}
  Value(const char* s) : v(std::string(s)) {}
  Value(Array a) : v(std::make_shared<Array>(std::move(a))) {}
  Value(Object o) : v(std::make_shared<Object>(std::move(o))) {}

  bool isNull() const { return std::holds_alternative<std::nullptr_t>(v); }
  bool isArray() const { return std::holds_alternative<std::shared_ptr<Array>>(v); }
  bool isObject() const { return std::holds_alternative<std::shared_ptr<Object>>(v); }
  const Array& arr() const { return *std::get<std::shared_ptr<Array>>(v); }
  const Object& obj() const { return *std::get<std::shared_ptr<Object>>(v); }
  const Value& operator[](size_t i) const { return arr().at(i); }
  const Value& operator[](const std::string& k) const { return obj().at(k); }
  bool has(const std::string& k) const { return isObject() && obj().count(k) && !obj().at(k).isNull(); }
  size_t size() const { return arr().size(); }
  bool boolean() const { return std::get<bool>(v); }
  const std::string& str() const { return std::get<std::string>(v); }
  /** The number, reading the non-finite strings back. */
  double num() const {
    if (auto s = std::get_if<std::string>(&v)) {
      if (*s == "NaN") return NAN;
      if (*s == "Infinity") return INFINITY;
      if (*s == "-Infinity") return -INFINITY;
      throw std::runtime_error("not a number: " + *s);
    }
    return std::get<double>(v);
  }
};

/** A number as gen.ts would have written it. */
inline Value number(double d) {
  if (std::isnan(d)) return "NaN";
  if (std::isinf(d)) return d > 0 ? "Infinity" : "-Infinity";
  return d;
}

class Parser {
  const std::string& s;
  size_t i = 0;
  void ws() { while (i < s.size() && std::isspace((unsigned char)s[i])) i++; }
  char peek() { ws(); return i < s.size() ? s[i] : '\0'; }
  void expect(char c) {
    if (peek() != c) throw std::runtime_error(std::string("json: expected ") + c + " at " + std::to_string(i));
    i++;
  }
  std::string string() {
    expect('"');
    std::string out;
    while (s[i] != '"') {
      if (s[i] == '\\') { i++; out += s[i] == 'n' ? '\n' : s[i]; } else out += s[i];
      i++;
    }
    i++;
    return out;
  }

 public:
  explicit Parser(const std::string& text) : s(text) {}
  Value value() {
    char c = peek();
    if (c == '{') {
      i++;
      Object o;
      if (peek() == '}') { i++; return o; }
      for (;;) {
        std::string k = string();
        expect(':');
        o[k] = value();
        if (peek() == ',') { i++; continue; }
        expect('}');
        return o;
      }
    }
    if (c == '[') {
      i++;
      Array a;
      if (peek() == ']') { i++; return a; }
      for (;;) {
        a.push_back(value());
        if (peek() == ',') { i++; continue; }
        expect(']');
        return a;
      }
    }
    if (c == '"') return string();
    if (s.compare(i, 4, "true") == 0) { i += 4; return true; }
    if (s.compare(i, 5, "false") == 0) { i += 5; return false; }
    if (s.compare(i, 4, "null") == 0) { i += 4; return nullptr; }
    size_t used = 0;
    double d = std::stod(s.substr(i, 32), &used);
    i += used;
    return d;
  }
};

inline Value parse(const std::string& text) { return Parser(text).value(); }

inline std::string print(const Value& x) {
  if (x.isNull()) return "null";
  if (auto b = std::get_if<bool>(&x.v)) return *b ? "true" : "false";
  if (auto d = std::get_if<double>(&x.v)) { char buf[32]; std::snprintf(buf, sizeof buf, "%.17g", *d); return buf; }
  if (auto s = std::get_if<std::string>(&x.v)) return '"' + *s + '"';
  std::string out;
  if (x.isArray()) {
    for (auto& e : x.arr()) out += (out.empty() ? "" : ",") + print(e);
    return '[' + out + ']';
  }
  for (auto& [k, e] : x.obj()) out += (out.empty() ? "\"" : ",\"") + k + "\":" + print(e);
  return '{' + out + '}';
}

/** Whether `got` matches `want`: the same shape, strings and booleans equal, numbers within `tol` relative to their
 *  size (absolute below 1). Keys `want` has as null may be missing from `got`. */
inline bool close(const Value& want, const Value& got, double tol) {
  if (want.v.index() != got.v.index()) return false;
  if (auto a = std::get_if<double>(&want.v)) {
    double b = std::get<double>(got.v);
    return std::abs(*a - b) <= tol * std::max({1.0, std::abs(*a), std::abs(b)});
  }
  if (want.isArray()) {
    if (want.size() != got.size()) return false;
    for (size_t k = 0; k < want.size(); k++) if (!close(want[k], got[k], tol)) return false;
    return true;
  }
  if (want.isObject()) {
    for (auto& [k, e] : want.obj()) {
      bool there = got.obj().count(k);
      if (e.isNull() ? there && !got[k].isNull() : !there || !close(e, got[k], tol)) return false;
    }
    return true;
  }
  return want.v == got.v;
}

}  // namespace json
