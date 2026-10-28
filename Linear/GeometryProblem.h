// The error a mesh operation throws when its input is not a surface it can work on.
#pragma once

#include <exception>
#include <string>

namespace editor {

/// A readable sentence saying why a mesh operation refused its mesh, such as
/// "Face 3 has fewer than 3 corners." The graph evaluator passes the sentence on as a problem
/// of the step that ran the operation.
///
/// Use when: a half-edge build or an extrude meets a mesh that is not a clean surface
/// Tags: mesh error, invalid topology, exception
/// Node: none (error reporting)
class GeometryProblem : public std::exception {
public:
    explicit GeometryProblem(std::string message) : message(std::move(message)) {}
    const char* what() const noexcept override { return message.c_str(); }

    std::string message;
};

} // namespace editor
