// The error the math types throw where Swift's `precondition` would stop the program.
#pragma once

#include <exception>
#include <string>

namespace editor {

/// A readable sentence saying why a math function refused its input, such as "A rotation axis
/// must have a direction; got a zero vector." Swift traps on these (`precondition`); the core
/// throws, like every other problem, and keeps Swift's sentence word for word.
///
/// Use when: a rotation is asked for around a zero axis; a zero quaternion is normalized
/// Tags: math error, precondition, invalid argument, exception
/// Node: none (error reporting)
class MathProblem : public std::exception {
public:
    explicit MathProblem(std::string message) : message(std::move(message)) {}
    const char* what() const noexcept override { return message.c_str(); }

    std::string message;
};

} // namespace editor
