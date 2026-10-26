// JavaScript's arithmetic where it differs from C++'s, for porting the TS faithfully. Internal to src/.
#pragma once

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace curves::js {

/** `x ? … : …` on a number: false for 0 and NaN. */
inline bool truthy(double x) { return x != 0 && !std::isnan(x); }
/** `x || d` on a number. */
inline double orElse(double x, double d) { return truthy(x) ? x : d; }
/** `Math.min(a, b)`: NaN if either is. */
inline double min(double a, double b) { return std::isnan(a) || std::isnan(b) ? NAN : a < b ? a : b; }
/** `Math.max(a, b)`: NaN if either is. */
inline double max(double a, double b) { return std::isnan(a) || std::isnan(b) ? NAN : a > b ? a : b; }
/** `Math.max(...xs)`: -Infinity for none, NaN if any is. */
template <class R>
double maxOf(const R& xs) {
  double m = -std::numeric_limits<double>::infinity();
  for (double x : xs) m = max(m, x);
  return m;
}
/** `Math.min(...xs)`: Infinity for none, NaN if any is. */
template <class R>
double minOf(const R& xs) {
  double m = std::numeric_limits<double>::infinity();
  for (double x : xs) m = min(m, x);
  return m;
}
/** `Math.round`: halves go up, unlike std::round. */
inline double round(double x) { return std::floor(x + 0.5); }
/** `a % b`: the sign of `a`, as std::fmod. */
inline double mod(double a, double b) { return std::fmod(a, b); }

/** `Number.prototype.toFixed(digits)` of a finite double as text: n/10^digits closest to x, the larger n on a tie. */
inline std::string toFixed(double x, int digits) {
  bool neg = x < 0;
  double v = std::abs(x);
  char buf[1400];
  // A tie is a value that ends exactly in a 5 one place past `digits`: nudge it up so printf's even rounding agrees.
  std::snprintf(buf, sizeof buf, "%.*f", digits + 60, v);
  std::string full = buf;
  size_t dot = full.find('.');
  if (dot != std::string::npos) {
    size_t at = dot + 1 + digits;
    if (full[at] == '5' && full.find_first_not_of('0', at + 1) == std::string::npos) v = std::nextafter(v, INFINITY);
  }
  std::snprintf(buf, sizeof buf, "%.*f", digits, v);
  return (neg ? "-" : "") + std::string(buf);
}

/** A number as JS prints it: the shortest digits that read back, fixed up to 1e21 and down to 1e-6, else exponent. */
inline std::string string(double x) {
  if (std::isnan(x)) return "NaN";
  if (x == 0) return "0";
  if (std::isinf(x)) return x < 0 ? "-Infinity" : "Infinity";
  char buf[64];
  auto r = std::to_chars(buf, buf + sizeof buf, std::abs(x), std::chars_format::scientific);
  std::string s(buf, r.ptr);
  size_t e = s.find('e');
  int exp10 = std::atoi(s.c_str() + e + 1);
  std::string digits;
  for (size_t i = 0; i < e; i++) if (s[i] != '.') digits += s[i];
  int k = int(digits.size()), n = exp10 + 1;
  std::string out;
  if (k <= n && n <= 21) out = digits + std::string(n - k, '0');
  else if (0 < n && n <= 21) out = digits.substr(0, n) + "." + digits.substr(n);
  else if (-6 < n && n <= 0) out = "0." + std::string(-n, '0') + digits;
  else {
    int ex = n - 1;
    out = digits.substr(0, 1) + (k > 1 ? "." + digits.substr(1) : "") + "e" + (ex < 0 ? "-" : "+") + std::to_string(std::abs(ex));
  }
  return (x < 0 ? "-" : "") + out;
}

}  // namespace curves::js
