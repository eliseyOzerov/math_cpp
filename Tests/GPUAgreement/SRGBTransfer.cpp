// Kernels/SRGBTransfer.h: the GPU (SRGBTransfer.metal) against the CPU.
#include "../../Kernels/SRGBTransfer.h"
#include "Agreement.h"

using namespace editor::agreement;
using editor::kernels::SRGBTransfer;

// Values a little outside 0…1 check the clamp; 4097 steps cross both thresholds (0.0031308 and
// 0.04045) many times over.
AGREEMENT({
    "srgbEncodedFromLinearAgreement", 1, 1,
    evenlySpaced(-0.1f, 1.1f, 4097),
    [](const float* inputs, float* outputs) { outputs[0] = SRGBTransfer::encodedFromLinear(inputs[0]); },
    1e-7f, 1e-6f,
});

AGREEMENT({
    "srgbLinearFromEncodedAgreement", 1, 1,
    evenlySpaced(-0.1f, 1.1f, 4097),
    [](const float* inputs, float* outputs) { outputs[0] = SRGBTransfer::linearFromEncoded(inputs[0]); },
    1e-7f, 1e-6f,
});
