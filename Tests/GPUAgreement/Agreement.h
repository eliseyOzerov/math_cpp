// GPU agreement cases: a shared kernel run on the GPU (a Metal compute function in this folder)
// and on the CPU (the same header, called from C++), over the same inputs, compared within a
// stated tolerance. The CPU result is the reference (Docs/Foundations.md). `make gpu`
// builds and runs them; Runner.mm is the only file that talks to Metal.
#pragma once

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace editor::agreement {

/// One kernel to compare. The GPU function `functionName` reads `inputsPerItem` floats per item
/// from buffer 0 and writes `outputsPerItem` floats per item to buffer 1; `cpu` does the same
/// for one item on the CPU.
struct AgreementCase {
    std::string functionName;
    int inputsPerItem;
    int outputsPerItem;
    /// The inputs of every item, `inputsPerItem` floats each, one after another.
    std::vector<float> inputs;
    std::function<void(const float* inputs, float* outputs)> cpu;
    /// The largest difference allowed: `absoluteTolerance + relativeTolerance · |cpu|`. The GPU
    /// builds with fast math, so a few units in the last place are expected.
    float absoluteTolerance;
    float relativeTolerance;
};

inline std::vector<AgreementCase>& cases() {
    static std::vector<AgreementCase> all;
    return all;
}

struct Registration {
    explicit Registration(AgreementCase agreementCase) { cases().push_back(std::move(agreementCase)); }
};

/// `count` evenly spaced numbers from `first` to `last`, both included: inputs for one-number kernels.
inline std::vector<float> evenlySpaced(float first, float last, int count) {
    std::vector<float> values;
    for (int index = 0; index < count; index += 1) {
        float fraction = count > 1 ? static_cast<float>(index) / static_cast<float>(count - 1) : 0.0f;
        values.push_back(first + (last - first) * fraction);
    }
    return values;
}

/// Every pair (first, second) of the two lists, as one flat list: inputs for two-number kernels.
inline std::vector<float> everyPair(const std::vector<float>& firsts, const std::vector<float>& seconds) {
    std::vector<float> values;
    for (float first : firsts) {
        for (float second : seconds) {
            values.push_back(first);
            values.push_back(second);
        }
    }
    return values;
}

/// Every triple (first, second, third) of the three lists, as one flat list: inputs for
/// three-number kernels.
inline std::vector<float> everyTriple(const std::vector<float>& firsts, const std::vector<float>& seconds,
    const std::vector<float>& thirds) {
    std::vector<float> values;
    for (float first : firsts) {
        for (float second : seconds) {
            for (float third : thirds) {
                values.push_back(first);
                values.push_back(second);
                values.push_back(third);
            }
        }
    }
    return values;
}

} // namespace editor::agreement

#define EDITOR_AGREEMENT_JOIN_INNER(first, second) first##second
#define EDITOR_AGREEMENT_JOIN(first, second) EDITOR_AGREEMENT_JOIN_INNER(first, second)
/// Registers an AgreementCase: `AGREEMENT({...});` at file scope.
#define AGREEMENT(...) \
    static const ::editor::agreement::Registration EDITOR_AGREEMENT_JOIN(agreementRegistration, __LINE__)(__VA_ARGS__)
