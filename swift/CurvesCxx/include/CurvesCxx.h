// The C++ API (curves/*.hpp) as the module CurvesCxx, for Swift's C++ interop, with what Swift can't reach in it on
// its own. The Swift module Curves (swift/Curves) re-exports this one and adds the default arguments Swift drops; a
// Swift client imports Curves. See README.md, "From Swift".
//
// What Swift can't do with the headers alone, and what this file gives it instead:
// - Spell std::vector<T> (the allocator parameter): every list type has a name here (PtList, ContourList…). Swift
//   array literals still convert to vector parameters, and a vector converts to a Swift Array with Array(v).
// - Build a std::variant: cxx:: has a maker per alternative (cxx::strength(0.5)); Curves wraps them as inits
//   (curves.Strength(0.5)).
// - Make a std::span: every function that takes spans has an overload here taking vectors.
// - Pass a closure where a std::function is unnamed, takes references or returns a function: each callback has a
//   name here (curves.Warp { p in … }), and the ones that take references have overloads taking them by value
//   (CornerSize, WindingRule, NodeWarp…).
// - Call a function template with a defaulted template argument: keysAt, sorted and spread for doubles are in cxx::.
// Nothing here changes what a function does: each one forwards to the header it names.
#pragma once

#include "curves/curves.hpp"

namespace curves {

// Lists.
using PtList = std::vector<Pt>;
using PtLists = std::vector<std::vector<Pt>>;
using DoubleList = std::vector<double>;
using IntList = std::vector<int>;
using CubicList = std::vector<Cubic>;
using ContourList = std::vector<Contour>;
using ContourGroups = std::vector<std::vector<Contour>>;
using NodeList = std::vector<Node>;
using NodeLists = std::vector<std::vector<Node>>;
using PathList = std::vector<Path>;
using SelectionList = std::vector<Selection>;
using StrengthList = std::vector<Strength>;
using HandleLockList = std::vector<HandleLock>;
using HandleRefList = std::vector<HandleRef>;
using SharedEndList = std::vector<SharedEnd>;
using EdgeList = std::vector<Edge>;
using CornerList = std::vector<Corner>;
using BoolCornerList = std::vector<BoolCorner>;
using OperandList = std::vector<Operand>;
using PairList = std::vector<Pair>;
using StepList = std::vector<Step>;
using ArgList = std::vector<Arg>;
using KeyD = Key<double>;
using KeyList = Keys<double>;

// Optionals (a Swift client builds one with .init(v) and reads it with .value).
using OptDouble = std::optional<double>;
using OptInt = std::optional<int>;
using OptPt = std::optional<Pt>;
using OptDoubles = std::optional<std::vector<double>>;
using OptMount = std::optional<Mount>;
using OptAlign = std::optional<Align>;
using OptKeys = std::optional<Keys<double>>;

// Callbacks the headers leave unnamed.
using IntTest = std::function<bool(int)>;
using IntRemap = std::function<std::optional<int>(int)>;
using NodeWeight = std::function<double(Pt, int, Progress)>;
using NodeAmount = std::function<double(int)>;
using NodeMove = std::function<Pt(Pt, int)>;
using Noise = std::function<double(double, double)>;

// Callbacks with their arguments by value, for the ones that take references.
using CornerSize = std::function<double(Corner)>;
using CornerMaker = std::function<std::vector<Piece>(Pt, Pt, Pt, Pt, Corner)>;
using CornerShaper = std::function<CornerShape(Corner)>;
using WindingRule = std::function<bool(std::vector<int>)>;
using NodeWarp = std::function<Warp(Pt, Path, int)>;
using PickOnPath = std::function<PickOn(Path)>;

namespace cxx {

// Variants, one maker per alternative. Wave, Falloff and Reach's function are one type, function<double(double)>.
inline Strength strength(double v) { return v; }
inline Strength strength(std::vector<double> v) { return v; }
inline Strength strength(Wave w) { return w; }
inline Reach reach(double v) { return v; }
inline Reach reach(Wave f) { return f; }
inline NodePick nodePick(std::vector<int> nodes) { return nodes; }
inline NodePick nodePick(NodeWeight f) { return f; }
inline FalloffSpec falloffSpec(FalloffName n) { return n; }
inline FalloffSpec falloffSpec(Falloff f) { return f; }
inline KeysInput keysInput(Keys<double> k) { return k; }
inline KeysInput keysInput(double v) { return v; }
inline KeysInput keysInput(std::vector<double> v) { return v; }
inline KeysInput keysInput(Wave w) { return w; }
inline Which which(End e) { return e; }
inline Which which(int i) { return i; }
inline ParamValue paramValue(double v) { return v; }
inline ParamValue paramValue(std::string s) { return s; }
inline Target target(TargetOn t) { return t; }
inline Target target(TargetPoint t) { return t; }
inline Target target(TargetCross t) { return t; }
inline Operand operand(std::vector<Contour> cs) { return cs; }
inline Operand operand(Path p) { return p; }
inline Operand operand(const Bool& b) { return std::make_shared<const Bool>(b); }
inline std::variant<double, std::vector<double>> bulge(double v) { return v; }
inline std::variant<double, std::vector<double>> bulge(std::vector<double> v) { return v; }

// Reading a ParamValue.
inline bool isNumber(const ParamValue& v) { return std::holds_alternative<double>(v); }
inline double number(const ParamValue& v) { return std::get<double>(v); }
inline std::string text(const ParamValue& v) { return std::get<std::string>(v); }

// Values: a mod's values by parameter name.
inline void set(Values& v, const std::string& name, double x) { v[name] = x; }
inline void set(Values& v, const std::string& name, const std::string& x) { v[name] = x; }

// Keys of doubles (the templates' default).
inline double keysAt(const Keys<double>& keys, double t, double dflt = 0) { return curves::keysAt<double>(keys, t, dflt); }
inline double keysAt(const std::optional<Keys<double>>& keys, double t, double dflt = 0) {
  return curves::keysAt<double>(keys, t, dflt);
}
inline std::optional<Keys<double>> sorted(const std::optional<Keys<double>>& k) { return curves::sorted<double>(k); }
inline Keys<double> spread(const std::vector<double>& vs) { return curves::spread<double>(vs); }

// Selection's and Pick's callbacks by value.
inline Selection moveEach(const Selection& s, NodeWarp f) {
  return s.moveEach([f = std::move(f)](Pt p, const Path& path, int i) { return f(p, path, i); });
}
inline Pick pick(PickOnPath f) {
  return {[f = std::move(f)](const Path& p) { return f(p); }};
}

}  // namespace cxx

// The functions that take spans, taking vectors.
inline Pt mean(const std::vector<Pt>& ps) { return mean(std::span<const Pt>(ps)); }
inline Nearest nearest(const std::vector<Cubic>& segs, Pt p, int steps = 32) {
  return nearest(std::span<const Cubic>(segs), p, steps);
}
inline Box box(const Contour& qs, int samples = 16) { return box(std::span<const Piece>(qs), samples); }
inline std::vector<Cubic> cutAt(const Cubic& c, const std::vector<double>& shares, Pt metric = {1, 1}) {
  return cutAt(c, std::span<const double>(shares), metric);
}
inline std::vector<double> runLengths(const std::vector<Pt>& ps) { return runLengths(std::span<const Pt>(ps)); }
inline Pt pointAlong(const std::vector<Pt>& ps, const std::vector<double>& run, double s) {
  return pointAlong(std::span<const Pt>(ps), std::span<const double>(run), s);
}
inline double area(const std::vector<Pt>& ps) { return area(std::span<const Pt>(ps)); }
inline Box bounds(const std::vector<Pt>& ps) { return bounds(std::span<const Pt>(ps)); }
inline Pt center(const std::vector<Pt>& ps) { return center(std::span<const Pt>(ps)); }
inline std::vector<double> inUnit(const std::vector<double>& ts, double eps = 1e-9) {
  return inUnit(std::span<const double>(ts), eps);
}

inline std::vector<Corner> corners(const std::vector<Contour>& cs, double minDeg = 1) {
  return corners(std::span<const Contour>(cs), minDeg);
}
inline std::vector<Corner> pits(const std::vector<Contour>& cs, double maxDeg = 170) {
  return pits(std::span<const Contour>(cs), maxDeg);
}
inline std::vector<Corner> peaks(const std::vector<Contour>& cs, double minDeg = 10) {
  return peaks(std::span<const Contour>(cs), minDeg);
}
inline std::vector<Contour> replaceCorners(const std::vector<Contour>& cs, CornerSize reach, CornerMaker make,
                                           CornerOpts opts = {}) {
  return replaceCorners(
      std::span<const Contour>(cs), [&](const Corner& k) { return reach(k); },
      [&](Pt a, Pt ta, Pt b, Pt tb, const Corner& k) { return make(a, ta, b, tb, k); }, opts);
}
inline std::vector<Contour> roundCorners(const std::vector<Contour>& cs, double radius, CornerOpts opts = {}) {
  return roundCorners(std::span<const Contour>(cs), radius, opts);
}
inline std::vector<Contour> roundCorners(const std::vector<Contour>& cs, CornerSize radius, CornerOpts opts = {}) {
  return roundCorners(std::span<const Contour>(cs), [&](const Corner& k) { return radius(k); }, opts);
}
inline std::vector<Contour> shapeCorners(const std::vector<Contour>& cs, CornerSize size, CornerShaper shape) {
  return shapeCorners(
      std::span<const Contour>(cs), [&](const Corner& k) { return size(k); },
      [&](const Corner& k) { return shape(k); });
}
inline std::vector<Contour> capped(const std::vector<Contour>& cs, const EndFrame& end, const Cap& cap) {
  return capped(std::span<const Contour>(cs), end, cap);
}

inline std::optional<std::vector<Crossing>> crossings(const Cubic& a, const Cubic& b, double tol,
                                                     const std::vector<SharedEnd>& shared) {
  return crossings(a, b, tol, std::span<const SharedEnd>(shared));
}
inline std::optional<std::vector<Crossing>> curveCrossings(const Cubic& a, const Cubic& b, double tol,
                                                          const std::vector<SharedEnd>& shared) {
  return curveCrossings(a, b, tol, std::span<const SharedEnd>(shared));
}
inline std::vector<SelfCrossing> selfCrossings(const std::vector<Cubic>& segs, bool closed = false) {
  return selfCrossings(std::span<const Cubic>(segs), closed);
}
inline int winding(const std::vector<Contour>& cs, Pt p) { return winding(std::span<const Contour>(cs), p); }
inline std::vector<Contour> boolean(const std::vector<std::vector<Contour>>& input, WindingRule inside,
                                    double eps = 0.25) {
  return boolean(std::span<const std::vector<Contour>>(input), [&](const std::vector<int>& w) { return inside(w); },
                 eps);
}
inline std::optional<std::vector<Contour>> tryBoolean(const std::vector<std::vector<Contour>>& input,
                                                      WindingRule inside, double eps = 0.25) {
  return tryBoolean(std::span<const std::vector<Contour>>(input),
                    [&](const std::vector<int>& w) { return inside(w); }, eps);
}
inline std::optional<Traced> traceBoolean(const std::vector<std::vector<Contour>>& input, WindingRule inside,
                                          double eps = 0.25) {
  return traceBoolean(std::span<const std::vector<Contour>>(input),
                      [&](const std::vector<int>& w) { return inside(w); }, eps);
}
inline std::vector<Contour> unite(const std::vector<Contour>& cs) { return unite(std::span<const Contour>(cs)); }
inline std::vector<Contour> orient(const std::vector<Contour>& cs) { return orient(std::span<const Contour>(cs)); }
inline std::vector<std::vector<Pt>> flatten(const std::vector<Contour>& cs, double step = 2, double min = 8) {
  return flatten(std::span<const Contour>(cs), step, min);
}
inline std::vector<Edge> edges(const std::vector<std::vector<Pt>>& polys) {
  return edges(std::span<const std::vector<Pt>>(polys));
}
inline std::vector<std::array<double, 2>> scanline(const std::vector<Edge>& es, double y) {
  return scanline(std::span<const Edge>(es), y);
}
inline std::optional<Gap> narrowest(const std::vector<Pt>& poly, double apart) {
  return narrowest(std::span<const Pt>(poly), apart);
}
inline Gap polyGap(const std::vector<Pt>& p, const std::vector<Pt>& q) {
  return polyGap(std::span<const Pt>(p), std::span<const Pt>(q));
}

}  // namespace curves
