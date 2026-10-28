// Regenerates Shaders/ShaderLibrarySource.cpp from the kernel headers (`make shader-library`).
// The logic lives in ShaderLibraryText.cpp, which the stale-file test uses too.
#include <filesystem>
#include <fstream>
#include <iostream>

#include "../Shaders/ShaderLibrary.h"

int main() {
    namespace filesystem = std::filesystem;
    filesystem::path source = filesystem::path(MATH_CPP_FOLDER);
    std::string text = editor::shaders::shaderLibraryTextFromKernels(source / "Kernels");
    filesystem::path target = source / "Shaders" / "ShaderLibrarySource.cpp";
    std::ofstream(target, std::ios::binary) << editor::shaders::shaderLibrarySourceFile(text);
    std::cout << "wrote " << target.string() << " (" << text.size() << " characters of library)\n";
    return 0;
}
