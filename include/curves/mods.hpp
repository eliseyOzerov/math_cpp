// Modifiers: every operation on a selection (selection.hpp) described as data, its parameters with their ranges and
// how it is called, so a panel, a playground and a rule can all take any of them the same way. A modifier takes a
// selection and gives one: the ones that shape the selection itself (soften, weigh, invert) and the ones that move its
// nodes chain alike, `path.select(…).soften(0.3).inflate(0.1)`.
//
// Decisions:
// - One shape for all (name, parameters, args), not a class per operation: the parameters are what a panel draws
//   sliders from, and `args` turns the slider values into the method's arguments, so the call and the line of code a
//   panel shows are made from the same list and can't drift apart.
// - Ranges are in the metric's units, which in a glyph are box heights (a unit box is 1 high), so the same sliders
//   suit any glyph.
// - A wave as an argument is written as code (`sine({ cycles: 3 })`) and passed as the function, through `expr`.
//
// Ported from fountain-ts/src/curves/mods.ts.
//
// Decisions of the port:
// - The TS's untyped arguments are an Arg: undefined, a number, a string, a list, an object (keys in order), an
//   expression (its code and the Arg it passes) or a wave (only ever an expression's value).
// - The TS calls the Selection method by name with the arguments; here each Mod has `call`, which reads the passed
//   arguments into that method's typed call. `args` stays the one list both the call and the code are made from.
// - A modifier's parameters are a list of (name, Param) in the TS's order; Values is a map.
// - Numbers are written as JavaScript writes them (`String(+n.toFixed(4))` in arguments, `String(n)` in code).
#pragma once

#include "curves/profile.hpp"
#include "curves/selection.hpp"

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace curves {

/** A number on a slider; `value` is its default. */
struct NumParam {
  double min, max, step, value;
};
/** A choice among options; `value` is its default. */
struct ChoiceParam {
  std::vector<std::string> options;
  std::string value;
};
/** A parameter: a number on a slider, or a choice among options. */
using Param = std::variant<NumParam, ChoiceParam>;

/** A parameter's value: a number or a choice. */
using ParamValue = std::variant<double, std::string>;
/** A modifier's parameter values by name. */
using Values = std::map<std::string, ParamValue>;

/** An argument of a modifier's call: what the TS passes untyped. */
struct Arg {
  enum class Kind { undefined, number, string, list, object, expr, wave };
  Kind kind = Kind::undefined;
  double number = 0;
  /** A string's text, or an expression's code. */
  std::string string;
  /** A list's items, an object's values (by `keys`), or an expression's value (one). */
  std::vector<Arg> items;
  std::vector<std::string> keys;
  Wave wave;

  static Arg of(double n);
  static Arg of(std::string s);
  static Arg list(std::vector<Arg> items);
  static Arg object(std::vector<std::pair<std::string, Arg>> entries);
  static Arg of(Wave w);
  bool isUndefined() const { return kind == Kind::undefined; }
  /** The entry `key` of an object; undefined when it has none. */
  const Arg& operator[](const std::string& key) const;
};

/** An argument written as `code` in a modifier's line and passed to the call as `value`. */
Arg expr(std::string code, Arg value);

/** A modifier: the Selection method `name`, grouped under `group`, its parameters, its arguments made from their
 *  values (an option left at its default is left undefined, so the line says only what changed), and the method
 *  called with them as passed. */
struct Mod {
  std::string name, group, doc;
  std::vector<std::pair<std::string, Param>> params;
  std::function<std::vector<Arg>(const Values&)> args;
  std::function<Selection(const Selection&, const std::vector<Arg>&)> call;
};

/** Every modifier, in the order a panel lists them. */
const std::vector<Mod>& mods();

/** The modifier called `name`. Throws when there is none. */
const Mod& mod(const std::string& name);

/** The defaults of a modifier's parameters. */
Values modDefaults(const Mod& m);

/** The selection with modifier `m` applied with values `v` (its defaults for any left out). */
Selection applyMod(const Selection& s, const Mod& m, const Values& v = {});

/** The call as a line of code, `.inflate(0.1, { handles: 0.5 })`, trailing arguments left at their defaults
 *  dropped. */
std::string modCode(const Mod& m, const Values& v = {});

/** A chain of modifiers applied in order: what a stack in a panel is. */
struct Step {
  std::string mod;
  Values values;
};

/** The selection with every step applied in turn. */
Selection chain(const Selection& s, const std::vector<Step>& steps);

/** The steps as code, one call after another. */
std::string chainCode(const std::vector<Step>& steps);

}  // namespace curves
