// Compiles the preview of every output of every node kind in the catalog (fed numbers, vectors and
// colours) with Metal's runtime compiler, as graphics_metal's apps do: default options, so fast
// math and the system's default language version, which accepts the lambdas that scoped inputs
// (Loop, Raymarch) compile to. Each compiled library's vertex and fragment functions then build a
// render pipeline. Prints "N shader previews compile, M fail" and each failure's kind, output and
// compiler message. Also compiles the starter graph and a graph with every end. Exits successfully,
// with a message, when there is no Metal device.
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <cstdio>
#include <string>

#include "../../Shaders/NodeCatalog.h"
#include "../../Shaders/NodePreviews.h"
#include "../../Shaders/ShaderCompiler.h"

using namespace editor::shaders;

namespace {

/// Builds the shader's library and a render pipeline from its two functions; returns an empty
/// string on success, else what Metal said.
std::string buildPipeline(id<MTLDevice> device, const CompiledShader& shader) {
    NSError* error = nil;
    NSString* source = [NSString stringWithUTF8String:shader.source.c_str()];
    id<MTLLibrary> library = [device newLibraryWithSource:source options:nil error:&error];
    if (library == nil) return error.localizedDescription.UTF8String;
    MTLRenderPipelineDescriptor* descriptor = [[MTLRenderPipelineDescriptor alloc] init];
    descriptor.vertexFunction = [library newFunctionWithName:[NSString stringWithUTF8String:ShaderCompiler::vertexFunction()]];
    descriptor.fragmentFunction = [library newFunctionWithName:[NSString stringWithUTF8String:ShaderCompiler::fragmentFunction()]];
    descriptor.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
    if (shader.writesFeedback) descriptor.colorAttachments[1].pixelFormat = MTLPixelFormatRGBA16Float;
    id<MTLRenderPipelineState> pipeline = [device newRenderPipelineStateWithDescriptor:descriptor error:&error];
    if (pipeline == nil) return std::string("pipeline: ") + error.localizedDescription.UTF8String;
    return "";
}

} // namespace

int main() {
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            std::printf("shader previews: no Metal device, so the check was skipped\n");
            return 0;
        }
        int compiled = 0;
        int failed = 0;
        auto check = [&](const std::string& name, const CompiledShader& shader) {
            std::string problem = shader.error ? "graph error: " + *shader.error : buildPipeline(device, shader);
            if (problem.empty()) {
                compiled += 1;
            } else {
                failed += 1;
                std::printf("FAIL %s\n%s\n", name.c_str(), problem.c_str());
            }
        };
        for (const Preview& preview : allPreviews()) check(preview.name(), preview.compile());

        // Whole graphs: the starter, and every kind of end at once.
        check("starter graph", ShaderCompiler::compile(ShaderGraph::starter()));
        ShaderGraph ends;
        ends.addNode(kinds::shader, {}, "shader");
        ends.addNode(kinds::feedbackOutput, {}, "feedback");
        ends.addNode(kinds::color, {}, "color");
        ends.connect("color", "color", "shader", "color");
        ends.connect("color", "color", "feedback", "color");
        check("shader and feedback ends", ShaderCompiler::compile(ends));

        std::printf("%d shader previews compile, %d fail\n", compiled, failed);
        return failed == 0 ? 0 : 1;
    }
}
