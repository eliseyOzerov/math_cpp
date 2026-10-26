// See include/curves/profile.hpp. Ported from fountain-ts/src/curves/profile.ts line for line.
#include "curves/profile.hpp"

#include "js.hpp"

// No fused multiply-adds: JS rounds every operation, and the golden values depend on it.
#pragma STDC FP_CONTRACT OFF

#include <cmath>
#include <set>

namespace curves {

Wave constant(double v) { return [v](double) { return v; }; }

Wave gradient(std::vector<double> values) {
  if (!values.size()) throw std::invalid_argument("a gradient needs a value");
  if (values.size() == 1) return constant(values[0]);
  int n = int(values.size()) - 1;
  return [values = std::move(values), n](double x) {
    double u = js::min(js::max(x, 0), 1) * n;
    if (std::isnan(u)) return double(NAN);
    double i = js::min(std::floor(u), n - 1), t = u - i;
    size_t k = size_t(i);
    return values[k] + (values[k + 1] - values[k]) * t;
  };
}

Wave steps(std::vector<double> values) {
  if (!values.size()) throw std::invalid_argument("steps need a value");
  return [values = std::move(values)](double x) {
    double u = js::min(js::max(x, 0), 1) * double(values.size());
    if (std::isnan(u)) return double(NAN);
    return values[size_t(js::min(std::floor(u), double(values.size()) - 1))];
  };
}

namespace {
/** A repeating wave from its shape over one cycle (`f` of 0…1, giving −1…1). */
Wave periodic(std::function<double(double)> f, Swing s) {
  return [f = std::move(f), s](double x) {
    double u = s.cycles * x + s.phase, v = f(u - std::floor(u));
    return s.low + (v + 1) / 2 * (s.high - s.low);
  };
}
}  // namespace

Wave sine(Swing s) { return periodic([](double u) { return std::sin(2 * M_PI * u); }, s); }
Wave cosine(Swing s) { return periodic([](double u) { return std::cos(2 * M_PI * u); }, s); }
Wave square(SquareSwing s) { double duty = s.duty; return periodic([duty](double u) { return u < duty ? 1.0 : -1.0; }, s); }
Wave triangle(Swing s) { return periodic([](double u) { return 1 - 4 * std::abs(u - 0.5); }, s); }
Wave saw(Swing s) { return periodic([](double u) { return 2 * u - 1; }, s); }

Wave wave(const Strength& s) {
  if (auto f = std::get_if<Wave>(&s)) return *f;
  if (auto v = std::get_if<double>(&s)) return constant(*v);
  return gradient(std::get<std::vector<double>>(s));
}

Wave times(const std::vector<Strength>& ws) {
  std::vector<Wave> fs;
  for (auto& w : ws) fs.push_back(wave(w));
  return [fs = std::move(fs)](double x) { double v = 1; for (auto& f : fs) v = v * f(x); return v; };
}

Wave plus(const std::vector<Strength>& ws) {
  std::vector<Wave> fs;
  for (auto& w : ws) fs.push_back(wave(w));
  return [fs = std::move(fs)](double x) { double v = 0; for (auto& f : fs) v = v + f(x); return v; };
}

double smoothstep(double u) { return u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u); }

Keys<double> easeKeys(double t0, double a, double t1, double b) {
  double h = (t1 - t0) / 2;
  return {Key<double>{t0, a, Handles{std::nullopt, Pt{h, 0}}}, Key<double>{t1, b, Handles{Pt{-h, 0}, std::nullopt}}};
}

Keys<double> bulge(double mid, double ends) {
  double d = (mid - ends) * 4 / 3;
  return {Key<double>{0, ends, Handles{std::nullopt, Pt{1.0 / 3, d}}}, Key<double>{1, ends, Handles{Pt{-1.0 / 3, d}, std::nullopt}}};
}

std::optional<Keys<double>> keysOf(const std::optional<KeysInput>& v, int samples) {
  if (!v) return std::nullopt;
  if (auto n = std::get_if<double>(&*v)) return Keys<double>{Key<double>{0, *n}};
  if (auto f = std::get_if<Wave>(&*v)) {
    Keys<double> out;
    for (int i = 0; i < samples; i++) out.push_back(Key<double>{double(i) / (samples - 1), (*f)(double(i) / (samples - 1))});
    return out;
  }
  if (auto k = std::get_if<Keys<double>>(&*v)) return *k;
  auto& xs = std::get<std::vector<double>>(*v);
  if (xs.size() == 0) return Keys<double>{};
  return spread<double>(xs);
}

std::optional<Keys<double>> keysTimes(const std::optional<Keys<double>>& a, const std::optional<Keys<double>>& b) {
  if (!a || !b) return a ? a : b;
  std::set<double> set;
  for (int i = 0; i < 129; i++) set.insert(double(i) / 128);
  for (auto& q : *a) set.insert(q.t);
  for (auto& q : *b) set.insert(q.t);
  Keys<double> out;
  for (double t : set) if (t >= 0 && t <= 1) out.push_back(Key<double>{t, keysAt(*a, t, 1.0) * keysAt(*b, t, 1.0)});
  return out;
}

}  // namespace curves
