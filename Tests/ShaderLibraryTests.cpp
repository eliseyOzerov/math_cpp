// The shader library text (Shaders/ShaderLibrarySource.cpp) must be what the kernel headers
// give now. `make shader-library` regenerates it.
#include <filesystem>

#include "../Shaders/ShaderLibrary.h"
#include "Testing.h"

using namespace editor::shaders;

namespace {

std::filesystem::path kernelsFolder() {
    return std::filesystem::path(MATH_CPP_FOLDER) / "Kernels";
}

} // namespace

TEST(shaderLibrarySourceIsNotStale) {
    std::string current = shaderLibraryTextFromKernels(kernelsFolder());
    if (shaderLibrarySource() != current) {
        editor::testing::fail(__FILE__, __LINE__, "ShaderLibrarySource.cpp is stale; run make shader-library");
    }
}

TEST(shaderLibraryHasEveryKernelHeaderInIncludeOrder) {
    std::string library = shaderLibrarySource();
    std::size_t previous = 0;
    // A header comes after the headers it includes.
    for (const char* name : {"KernelPrelude.h", "KernelVectors.h", "ShaderMath.h", "BlendModes.h"}) {
        std::size_t position = library.find(std::string("// ==== Kernels/") + name + " ====");
        EXPECT(position != std::string::npos);
        EXPECT(position >= previous);
        previous = position;
    }
    for (const auto& entry : std::filesystem::directory_iterator(kernelsFolder())) {
        EXPECT(library.find("// ==== Kernels/" + entry.path().filename().string() + " ====") != std::string::npos);
    }
    EXPECT(library.find("#include \"") == std::string::npos);
    EXPECT(library.find("#pragma once") == std::string::npos);
    EXPECT(library.find("#include <metal_stdlib>") != std::string::npos);
    const std::string ending = "using namespace editor::kernels;\n";
    EXPECT(library.size() >= ending.size() && library.substr(library.size() - ending.size()) == ending);
}

TEST(shaderLibrarySourceFileSplitsIntoShortLiterals) {
    std::string text(30000, 'x');
    for (std::size_t index = 99; index < text.size(); index += 100) text[index] = '\n';
    std::string file = shaderLibrarySourceFile(text);
    std::size_t literals = 0;
    for (std::size_t at = file.find("R\"metal("); at != std::string::npos; at = file.find("R\"metal(", at + 1)) literals += 1;
    EXPECT(literals >= 3);
}
