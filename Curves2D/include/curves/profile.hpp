// Profiles: a value along a share x (0…1, round a ring or along a path), to vary a move or a pen from place to place.
// Two kinds. A wave is a function of x: one value is the same everywhere, a list of values a gradient or steps, and
// sine, cosine, square, triangle and saw waves repeat; waves multiply and add, so they combine into any profile.
// Keys are data: (t, value) points with optional bezier handles, like animation keyframes, so they save as JSON and
// a panel can drag them, and the value can be any type (the nib morphs between shapes this way).
//
// Why functions of x: a strength is generic, not tied to inflate or to rings, so any move that varies from node to
// node can take one, and x can be a share round a ring, along a path or anything else. A wave below 0 is fine:
// under inflate, a negative strength shrinks.
//
// Known issue (as in the TS): the two kinds overlap. A wave given where keys are wanted is sampled (keysOf), and
// keys multiply by resampling (keysTimes) where waves multiply exactly; keys could simply be a wave closed over its
// points. Merging them is its own change.
//
// Ported from fountain-ts/src/curves/profile.ts.
//
// Decisions:
// - Wave is a std::function<double(double)>; Strength a std::variant<double, std::vector<double>, Wave>.
// - Keys<V> is a vector of Key<V> {t, v, optional Handles}, a template over the value type. A TS "no keys" (null or
//   undefined) is std::nullopt where a function returns or takes one, and an empty Keys elsewhere.
// - keysAt's numeric-with-handles path (the cubic, halved 40 times) exists only for V = double (`if constexpr`); for
//   any other V the keys are blended by `mix`, which must then be given. For V = double `mix` defaults to lerp.
// - Swing's options are a struct with the TS defaults; square's extra `duty` makes SquareSwing.
// - keysOf takes KeysInput, a variant of Keys<double>, a number, a list of numbers or a Wave (an empty list is
//   empty keys, as in the TS).
#pragma once

#include "curves/geometry.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <type_traits>
#include <optional>
#include <variant>
#include <vector>

namespace curves {

/** A strength for each share `x` of the way (0…1). */
using Wave = std::function<double(double)>;

/** A strength as given: one value everywhere, a list of values as a gradient between them, or a wave. */
using Strength = std::variant<double, std::vector<double>, Wave>;

/** The same `v` everywhere. */
Wave constant(double v);

/** A smooth run through `values`, spread evenly from x = 0 to x = 1 (one value is the same everywhere). For a ring,
 *  end on the first value so it meets itself. Throws with no values. */
Wave gradient(std::vector<double> values);

/** Hard jumps between `values`: x split into as many equal parts, each part one value. Throws with no values. */
Wave steps(std::vector<double> values);

/** How a repeating wave is laid out: `cycles` over x from 0 to 1, starting `phase` of a cycle in, swinging between
 *  `low` and `high`. */
struct Swing {
  double cycles = 1, phase = 0, low = -1, high = 1;
};
/** A square wave's layout: a swing and its duty. */
struct SquareSwing : Swing {
  double duty = 0.5;
};

/** A sine wave: at its middle at the start, rising. */
Wave sine(Swing s = {});
/** A cosine wave: at its high at the start. */
Wave cosine(Swing s = {});
/** A square wave: high for the first `duty` of each cycle, then low. */
Wave square(SquareSwing s = {});
/** A triangle wave: from its low up to its high halfway through each cycle and back. */
Wave triangle(Swing s = {});
/** A saw wave: from its low up to its high over each cycle, then straight back down. */
Wave saw(Swing s = {});

/** A strength as a wave. */
Wave wave(const Strength& s);

/** The strengths multiplied: a wave shaped by another (a sine fading out under a gradient, say). */
Wave times(const std::vector<Strength>& ws);

/** The strengths added. */
Wave plus(const std::vector<Strength>& ws);

/** The smooth step from 0 at u = 0 to 1 at u = 1, level at both ends and held there outside them. */
double smoothstep(double u);

/** A key's bezier handles, each (dt, dv) from it. */
struct Handles {
  std::optional<Pt> in, out;
};
/** A key: a value at a share t, and optionally handles. */
template <class V = double>
struct Key {
  double t;
  V v;
  std::optional<Handles> h = std::nullopt;
};
/** Keys: (t, value) points, or with handles, over a share 0…1 (along a stroke, 0 start … 1 end). */
template <class V = double>
using Keys = std::vector<Key<V>>;

/** The keys' value at t, level past the first and last; `dflt` with none. Between two keys the value runs straight
 *  unless either has a handle on that side, and then as the cubic through the handles (a missing one sits on its
 *  key); values that aren't numbers are blended by `mix`. Handles are held inside their span along t, so there is
 *  one value at each t. */
template <class V = double>
V keysAt(const Keys<V>& keys, double t, V dflt = V{}, std::function<V(const V&, const V&, double)> mix = {}) {
  if (!keys.size()) return dflt;
  if (!mix) {
    if constexpr (std::is_same_v<V, double>) mix = [](const double& a, const double& b, double u) { CURVES_NO_FMA return a + (b - a) * u; };
    else throw std::invalid_argument("keysAt needs a mix for values that aren't numbers");
  }
  if (t <= keys[0].t) return keys[0].v;
  for (size_t i = 1; i < keys.size(); i++) if (t <= keys[i].t) {
    double t0 = keys[i - 1].t, t1 = keys[i].t;
    const V& a = keys[i - 1].v; const V& b = keys[i].v;
    const std::optional<Handles>& h0 = keys[i - 1].h; const std::optional<Handles>& h1 = keys[i].h;
    if (!(t1 > t0)) return b;
    std::optional<Pt> out = h0 ? h0->out : std::nullopt, inn = h1 ? h1->in : std::nullopt;
    if constexpr (std::is_same_v<V, double>) {
      if (!out && !inn) return mix(a, b, (t - t0) / (t1 - t0));
      double span = t1 - t0, v1 = b;
      Cubic c = {Pt{t0, a}, Pt{t0 + (out ? clamp((*out)[0], 0, span) : 0), a + (out ? (*out)[1] : 0)},
                 Pt{t1 + (inn ? clamp((*inn)[0], -span, 0) : 0), v1 + (inn ? (*inn)[1] : 0)}, Pt{t1, v1}};
      // The cubic's t rises from t0 to t1, so the parameter at t is found by halving.
      double lo = 0, hi = 1;
      for (int k = 0; k < 40; k++) { double m = (lo + hi) / 2; if (bez(c, m)[0] < t) lo = m; else hi = m; }
      return bez(c, (lo + hi) / 2)[1];
    } else {
      return mix(a, b, (t - t0) / (t1 - t0));
    }
  }
  return keys.back().v;
}
/** keysAt for keys that may be none (nullopt: `dflt`). */
template <class V = double>
V keysAt(const std::optional<Keys<V>>& keys, double t, V dflt = V{}, std::function<V(const V&, const V&, double)> mix = {}) {
  return keysAt<V>(keys ? *keys : Keys<V>{}, t, std::move(dflt), std::move(mix));
}

/** Keys in order along t, or none for none. */
template <class V = double>
std::optional<Keys<V>> sorted(const std::optional<Keys<V>>& k) {
  if (!k || !k->size()) return std::nullopt;
  Keys<V> s = *k;
  std::stable_sort(s.begin(), s.end(), [](const Key<V>& a, const Key<V>& b) { return a.t - b.t < 0; });
  return s;
}

/** Values spread evenly over 0…1 as keys; one value is a single key. */
template <class V = double>
Keys<V> spread(const std::vector<V>& vs) {
  if (vs.size() == 1) return {Key<V>{0, vs[0]}};
  Keys<V> out;
  for (size_t i = 0; i < vs.size(); i++) out.push_back(Key<V>{double(i) / double(vs.size() - 1), vs[i]});
  return out;
}

/** Two keys easing from `a` at share `t0` to `b` at `t1`: flat handles, so the value leaves and meets its levels
 *  smoothly. */
Keys<double> easeKeys(double t0, double a, double t1, double b);

/** One cubic from `ends` at 0 to `mid` at the middle and back to `ends` at 1: a waist below `ends` (a nib narrowing
 *  along a stroke) or a swell above. A cubic whose ends sit at `ends` with handles a third along reaches
 *  ends + 3/4·(handle height − ends) at its middle, so the handles go (mid − ends)·4/3 from the ends. */
Keys<double> bulge(double mid, double ends = 1);

/** Keys given loosely: keys as they are, or a strength (a number, values spread evenly, or a wave) sampled into
 *  them, `samples` points for a wave (enough that a wave's shape survives the straight runs between them; keys with
 *  handles are the way to draw a smooth one exactly). */
using KeysInput = std::variant<Keys<double>, double, std::vector<double>, Wave>;
std::optional<Keys<double>> keysOf(const std::optional<KeysInput>& v, int samples = 65);

/** Two keys multiplied, either left out: read at 129 places and at each one's own keys, so a step in either stays
 *  sharp. */
std::optional<Keys<double>> keysTimes(const std::optional<Keys<double>>& a, const std::optional<Keys<double>>& b);

}  // namespace curves
