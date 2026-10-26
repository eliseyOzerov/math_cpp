#include "curves/bool.hpp"

#include <cmath>

#pragma STDC FP_CONTRACT OFF

namespace curves {

namespace {

bool anyNonZero(const std::vector<int>& w, size_t from) {
  for (size_t i = from; i < w.size(); i++)
    if (w[i] != 0) return true;
  return false;
}

Rule rule(Op op) {
  switch (op) {
    case Op::union_: return [](const std::vector<int>& w) { return anyNonZero(w, 0); };
    case Op::intersect:
      return [](const std::vector<int>& w) {
        for (int n : w)
          if (n == 0) return false;
        return true;
      };
    case Op::subtract: return [](const std::vector<int>& w) { return w.at(0) != 0 && !anyNonZero(w, 1); };
    case Op::exclude:
      return [](const std::vector<int>& w) {
        int k = 0;
        for (int n : w) k += n != 0;
        return k % 2 == 1;
      };
    case Op::cut: return [](const std::vector<int>& w) { return w.at(0) != 0; };
  }
  return {};
}

/** An outline from an operand: a Bool's own, a closed path's segments, or the contours as given. */
std::vector<Contour> outline(const Operand& o) {
  if (auto* b = std::get_if<std::shared_ptr<const Bool>>(&o)) return (*b)->contours();
  if (auto* p = std::get_if<Path>(&o)) {
    Contour c;
    for (auto& s : p->segs()) c.push_back(Piece::of(s));
    return {c};
  }
  return std::get<std::vector<Contour>>(o);
}

/** The corner between piece `prev` arriving and `next` leaving. The filled side is on the right of the way the
 *  outline runs, so out is the left normal; at a corner, the normals of the two pieces are added to halve it. */
BoolCorner corner(int contour, int node, const Piece& prev, const Piece& next, std::array<int, 2> between) {
  Pt din = norm(endTangents(prev)[1]), dout = norm(endTangents(next)[0]);
  auto left = [](Pt d) -> Pt { return {-d[1], d[0]}; };
  Pt sum = left(din) + left(dout);
  Pt outward = curves::hypot(sum[0], sum[1]) > 1e-9 ? norm(sum) : din * -1;
  return {contour, node, next.p[0], between, din, dout, outward, cross(din, dout) < 0};
}

}  // namespace

Bool::Bool(Op op, std::vector<Operand> operands, double eps) : op_(op), operands_(std::move(operands)), eps_(eps) {}

Bool Bool::with(std::vector<Operand> operands) const { return Bool(op_, std::move(operands), eps_); }

const std::optional<Traced>& Bool::traced() const {
  if (traced_) return *traced_;
  std::vector<std::vector<Contour>> groups;
  for (auto& o : operands_) groups.push_back(outline(o));
  if (op_ != Op::cut) return traced_.emplace(traceBoolean(groups, rule(op_), eps_));
  // The first operand's parts inside the others, then outside them.
  auto a = traceBoolean(groups, [](const std::vector<int>& w) { return w.at(0) != 0 && anyNonZero(w, 1); }, eps_);
  auto b = traceBoolean(groups, rule(Op::subtract), eps_);
  if (!a || !b) return traced_.emplace(std::nullopt);
  Traced t = *a;
  t.contours.insert(t.contours.end(), b->contours.begin(), b->contours.end());
  t.from.insert(t.from.end(), b->from.begin(), b->from.end());
  return traced_.emplace(t);
}

bool Bool::ok() const { return traced().has_value(); }

std::vector<Contour> Bool::contours() const {
  if (auto& t = traced()) return t->contours;
  std::vector<Contour> out;
  for (auto& o : operands_)
    for (auto& c : outline(o)) out.push_back(c);
  return out;
}

std::vector<std::vector<int>> Bool::sources() const {
  if (auto& t = traced()) return t->from;
  std::vector<std::vector<int>> out;
  for (auto& c : contours()) out.push_back(std::vector<int>(c.size(), -1));
  return out;
}

std::vector<BoolCorner> Bool::corners(std::optional<int> a, std::optional<int> b, const BoolCornersOptions& o) const {
  std::vector<BoolCorner> out;
  auto cs = contours();
  auto src = sources();
  for (int k = 0; k < int(cs.size()); k++) {
    const Contour& c = cs[k];
    const int n = int(c.size());
    for (int j = 0; j < n; j++) {
      int i = (j + n - 1) % n;
      std::array<int, 2> between{src[k][i], src[k][j]};
      if (!o.same && between[0] == between[1]) continue;
      auto [x, y] = between;
      if (a && (!b ? x != *a && y != *a : !(x == *a && y == *b) && !(x == *b && y == *a))) continue;
      out.push_back(corner(k, j, c[i], c[j], between));
    }
  }
  return out;
}

Pt Bool::outward(int contour, int node) const {
  auto cs = contours();
  const Contour& c = cs.at(contour);
  const int n = int(c.size());
  int i = (node + n - 1) % n;
  return corner(contour, node, c[i], c.at(node), {-1, -1}).outward;
}

}  // namespace curves
