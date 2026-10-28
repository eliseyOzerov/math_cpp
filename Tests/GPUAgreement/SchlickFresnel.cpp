// Kernels/SchlickFresnel.h: the GPU (SchlickFresnel.metal) against the CPU.
#include "../../Kernels/SchlickFresnel.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::SchlickFresnel;

AGREEMENT({
    "schlickNormalIncidenceAgreement", 2, 1,
    everyPair(evenlySpaced(1.0f, 2.5f, 32), evenlySpaced(1.0f, 2.5f, 32)),
    [](const float* inputs, float* outputs) {
        outputs[0] = SchlickFresnel::reflectanceAtNormalIncidence(inputs[0], inputs[1]);
    },
    1e-7f, 1e-6f,
});

AGREEMENT({
    // Cosines a little outside 0…1 check the clamp as well.
    "schlickReflectanceAgreement", 2, 1,
    everyPair(evenlySpaced(-0.1f, 1.1f, 121), evenlySpaced(0.0f, 1.0f, 11)),
    [](const float* inputs, float* outputs) { outputs[0] = SchlickFresnel::reflectance(inputs[0], inputs[1]); },
    1e-7f, 1e-6f,
});
