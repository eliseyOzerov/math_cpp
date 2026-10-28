// Runs every GPU agreement case (Agreement.h): each kernel on the Mac's GPU through Metal, and on
// the CPU, then prints the largest difference and fails when one is beyond the case's tolerance.
// Exits successfully, with a message, when there is no Metal device (a headless or virtual machine).
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "Agreement.h"

using editor::agreement::AgreementCase;

namespace {

/// The CPU's outputs for every item of `agreementCase`.
std::vector<float> cpuOutputs(const AgreementCase& agreementCase, int itemCount) {
    std::vector<float> outputs(static_cast<std::size_t>(itemCount * agreementCase.outputsPerItem));
    for (int item = 0; item < itemCount; item += 1) {
        agreementCase.cpu(agreementCase.inputs.data() + item * agreementCase.inputsPerItem,
                          outputs.data() + item * agreementCase.outputsPerItem);
    }
    return outputs;
}

/// The GPU's outputs for every item, or an empty list after printing why it could not run.
std::vector<float> gpuOutputs(id<MTLDevice> device, id<MTLLibrary> library, id<MTLCommandQueue> queue,
                              const AgreementCase& agreementCase, int itemCount) {
    NSString* name = [NSString stringWithUTF8String:agreementCase.functionName.c_str()];
    id<MTLFunction> function = [library newFunctionWithName:name];
    if (function == nil) {
        std::fprintf(stderr, "%s: no such Metal function in the agreement library\n", agreementCase.functionName.c_str());
        return {};
    }
    NSError* error = nil;
    id<MTLComputePipelineState> pipeline = [device newComputePipelineStateWithFunction:function error:&error];
    if (pipeline == nil) {
        std::fprintf(stderr, "%s: %s\n", agreementCase.functionName.c_str(), error.localizedDescription.UTF8String);
        return {};
    }
    std::size_t inputBytes = agreementCase.inputs.size() * sizeof(float);
    std::size_t outputBytes = static_cast<std::size_t>(itemCount * agreementCase.outputsPerItem) * sizeof(float);
    id<MTLBuffer> inputs = [device newBufferWithBytes:agreementCase.inputs.data() length:inputBytes
                                              options:MTLResourceStorageModeShared];
    id<MTLBuffer> outputs = [device newBufferWithLength:outputBytes options:MTLResourceStorageModeShared];

    id<MTLCommandBuffer> commands = [queue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [commands computeCommandEncoder];
    [encoder setComputePipelineState:pipeline];
    [encoder setBuffer:inputs offset:0 atIndex:0];
    [encoder setBuffer:outputs offset:0 atIndex:1];
    // One thread per item; dispatchThreads lets the last group be partly empty.
    NSUInteger groupWidth = std::min<NSUInteger>(pipeline.maxTotalThreadsPerThreadgroup, 64);
    [encoder dispatchThreads:MTLSizeMake(static_cast<NSUInteger>(itemCount), 1, 1)
        threadsPerThreadgroup:MTLSizeMake(groupWidth, 1, 1)];
    [encoder endEncoding];
    [commands commit];
    [commands waitUntilCompleted];

    std::vector<float> result(static_cast<std::size_t>(itemCount * agreementCase.outputsPerItem));
    std::memcpy(result.data(), outputs.contents, outputBytes);
    return result;
}

/// Compares one case and prints a line about it; returns whether it agreed.
bool agrees(id<MTLDevice> device, id<MTLLibrary> library, id<MTLCommandQueue> queue, const AgreementCase& agreementCase) {
    int itemCount = static_cast<int>(agreementCase.inputs.size()) / agreementCase.inputsPerItem;
    std::vector<float> expected = cpuOutputs(agreementCase, itemCount);
    std::vector<float> actual = gpuOutputs(device, library, queue, agreementCase, itemCount);
    if (actual.size() != expected.size()) return false;

    float largestDifference = 0;
    std::size_t worst = 0;
    bool withinTolerance = true;
    for (std::size_t index = 0; index < expected.size(); index += 1) {
        // Equal values agree outright, so two infinities of one sign do (their difference is NaN).
        float difference = actual[index] == expected[index] ? 0.0f : std::fabs(actual[index] - expected[index]);
        // NaN must meet NaN; any other NaN is a disagreement.
        if (std::isnan(expected[index]) || std::isnan(actual[index])) {
            difference = std::isnan(expected[index]) && std::isnan(actual[index]) ? 0.0f : INFINITY;
        }
        float allowed = agreementCase.absoluteTolerance + agreementCase.relativeTolerance * std::fabs(expected[index]);
        if (!(difference <= allowed)) withinTolerance = false;
        if (!(difference <= largestDifference)) {
            largestDifference = difference;
            worst = index;
        }
    }
    std::size_t worstItem = worst / static_cast<std::size_t>(agreementCase.outputsPerItem);
    std::printf("%s %s: %d items, largest difference %g (item %zu: cpu %.9g, gpu %.9g)\n", withinTolerance ? "agree" : "DIFFER",
                agreementCase.functionName.c_str(), itemCount, static_cast<double>(largestDifference), worstItem,
                static_cast<double>(expected[worst]), static_cast<double>(actual[worst]));
    return withinTolerance;
}

} // namespace

int main(int argumentCount, const char** arguments) {
    @autoreleasepool {
        if (argumentCount < 2) {
            std::fprintf(stderr, "usage: gpu-agreement <library.metallib>\n");
            return 2;
        }
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            std::printf("gpu-agreement: no Metal device, so the GPU agreement cases were skipped\n");
            return 0;
        }
        NSError* error = nil;
        NSURL* url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:arguments[1]]];
        id<MTLLibrary> library = [device newLibraryWithURL:url error:&error];
        if (library == nil) {
            std::fprintf(stderr, "gpu-agreement: %s\n", error.localizedDescription.UTF8String);
            return 1;
        }
        id<MTLCommandQueue> queue = [device newCommandQueue];
        int disagreements = 0;
        for (const AgreementCase& agreementCase : editor::agreement::cases()) {
            if (!agrees(device, library, queue, agreementCase)) disagreements += 1;
        }
        std::printf("%zu GPU agreement cases on %s, %d disagree\n", editor::agreement::cases().size(),
                    device.name.UTF8String, disagreements);
        return disagreements == 0 ? 0 : 1;
    }
}
