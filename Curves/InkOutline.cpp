// The ink outline's plan, its result and its key (InkOutline.h). Rails are in InkOutline+Rails.cpp, fitting in
// InkOutline+Fitting.cpp.
#include "InkOutline.h"

#include <algorithm>

#include "InkOutline+Internal.h"

namespace editor {

using namespace inkoutline;

namespace {

/// A plan with the fit it was last checked with. The fit is none when the last round still split a piece.
struct Planned {
    InkOutlinePlan plan;
    std::optional<Fit> fit;
};

/// The plan an ink needs alone: one piece per span (two per rounded end), then a piece more wherever the worst error is
/// over the tolerance, for up to `mostPieces` rounds.
Planned planned(const InkResult& ink, float tolerance) {
    RailPair parts = outlineParts(ink);
    const InkOutlineSettings settings;
    size_t spanCount = ink.nodes.size() > 0 ? ink.nodes.size() - 1 : 0;
    auto endPieces = [&](InkEnd end) -> std::optional<int> { return ink.closed || parts.trimmed(end) ? std::nullopt : std::optional<int>(2); };
    InkOutlinePlan plan;
    plan.left.assign(spanCount, 1);
    plan.right.assign(spanCount, 1);
    plan.start = endPieces(InkEnd::start);
    plan.end = endPieces(InkEnd::end);
    for (int round = 0; round < settings.mostPieces; round++) {
        Fit fit = fitOutline(ink, plan, parts);
        const InkOutlineErrors& errors = fit.errors;
        plan.runs = fit.runs;
        bool more = false;
        for (size_t span = 0; span < errors.left.size(); span++) {
            if (errors.left[span] > tolerance && plan.left[span] < settings.mostPieces) {
                plan.left[span]++;
                more = true;
            }
        }
        for (size_t span = 0; span < errors.right.size(); span++) {
            if (errors.right[span] > tolerance && plan.right[span] < settings.mostPieces) {
                plan.right[span]++;
                more = true;
            }
        }
        for (InkEnd end : {InkEnd::start, InkEnd::end}) {
            std::optional<int>& pieces = end == InkEnd::start ? plan.start : plan.end;
            float error = end == InkEnd::start ? errors.start : errors.end;
            if (pieces && error > tolerance && *pieces < settings.mostPieces) {
                pieces = *pieces + 1;
                more = true;
            }
        }
        if (!more) return {plan, fit};
    }
    return {plan, std::nullopt};
}

const std::optional<int>& endOf(const InkOutlinePlan& plan, InkEnd end) { return end == InkEnd::start ? plan.start : plan.end; }

/// An end's runs as a plan's key writes them: empty for a line end.
std::string runsKey(const InkOutlinePlan& plan, InkEnd end) {
    return plan.runs && endOf(plan, end) ? ":" + (end == InkEnd::start ? plan.runs->start : plan.runs->end) : "";
}

/// A fitted piece list as a closed path.
Path2D pathOf(const std::vector<Piece>& pieces) {
    Path2D path(pieces.front().cubic.start);
    for (const Piece& piece : pieces) {
        if (piece.isLine) path.addLine(piece.cubic.end);
        else path.addCubic(piece.cubic.startHandle, piece.cubic.endHandle, piece.cubic.end);
    }
    path.close();
    return path;
}

} // namespace

InkOutlinePlan InkOutline::plan(const InkResult& ink, float tolerance) { return planned(ink, tolerance).plan; }

std::optional<InkOutlinePlan> InkOutline::sharedPlan(const std::vector<InkOutlinePlan>& plans) {
    if (plans.empty()) return std::nullopt;
    const InkOutlinePlan& first = plans[0];
    auto differs = [&](const InkOutlinePlan& other) {
        if (other.left.size() != first.left.size() || other.right.size() != first.right.size()) return true;
        for (InkEnd end : {InkEnd::start, InkEnd::end}) {
            if (endOf(other, end).has_value() != endOf(first, end).has_value() || runsKey(other, end) != runsKey(first, end)) return true;
        }
        return false;
    };
    for (size_t index = 1; index < plans.size(); index++) {
        if (differs(plans[index])) return std::nullopt;
    }
    // A line end stays a line; a rounded one takes the larger count.
    auto most = [](const std::optional<int>& first, const std::optional<int>& second) -> std::optional<int> {
        return !first ? first : std::optional<int>(std::max(*first, second ? *second : 0));
    };
    InkOutlinePlan merged = first;
    for (size_t index = 1; index < plans.size(); index++) {
        for (size_t span = 0; span < merged.left.size(); span++) merged.left[span] = std::max(merged.left[span], plans[index].left[span]);
        for (size_t span = 0; span < merged.right.size(); span++) merged.right[span] = std::max(merged.right[span], plans[index].right[span]);
        merged.start = most(merged.start, plans[index].start);
        merged.end = most(merged.end, plans[index].end);
    }
    return merged;
}

InkOutlineResult InkOutline::outline(const InkResult& ink, const std::optional<InkOutlinePlan>& given, float tolerance) {
    std::optional<Planned> own;
    if (!given) own = planned(ink, tolerance);
    InkOutlinePlan plan = given ? *given : own->plan;
    Fit fit = own && own->fit ? *own->fit : fitOutline(ink, plan, outlineParts(ink));
    float error = std::max(fit.errors.start, fit.errors.end);
    for (float value : fit.errors.left) error = std::max(error, value);
    for (float value : fit.errors.right) error = std::max(error, value);
    InkOutlineResult result;
    result.contour = pathOf(fit.contour);
    int anchors = 0;
    for (const std::vector<Piece>& ring : fit.contours) {
        result.contours.outlines.push_back(pathOf(ring));
        anchors += static_cast<int>(ring.size());
    }
    result.plan = plan;
    result.anchors = anchors;
    result.error = error;
    result.errors = fit.errors;
    return result;
}

std::string InkOutline::planKey(const InkOutlinePlan& plan) {
    auto joined = [](const std::vector<int>& counts) {
        std::string text;
        for (size_t index = 0; index < counts.size(); index++) text += (index ? "." : "") + std::to_string(counts[index]);
        return text;
    };
    auto endText = [](const std::optional<int>& pieces) { return pieces ? std::to_string(*pieces) : std::string("line"); };
    return "L" + joined(plan.left) + " E" + endText(plan.end) + runsKey(plan, InkEnd::end) + " R" + joined(plan.right) + " S" + endText(plan.start) +
           runsKey(plan, InkEnd::start);
}

} // namespace editor
