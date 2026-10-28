// The Metal source every compiled shader needs: the shared kernels (Kernels), concatenated
// in include order, ending with `using namespace editor::kernels;`. The text is generated and
// committed (ShaderLibrarySource.cpp), so a host needs no files at run time; `make -C Core
// shader-library` regenerates it and a test fails when it is stale.
#pragma once

#include <filesystem>
#include <string>

namespace editor::shaders {

/// The library text as committed in ShaderLibrarySource.cpp.
///
/// Use when: assembling a shader's Metal source; checking the committed text
/// Tags: shader library, kernels as Metal, library source
/// Node: none (shader graph)
std::string shaderLibrarySource();

/// The library text built now from every header in `kernelsFolder`: each header after the ones it
/// includes (found from its `#include "..."` lines, which are removed), headers otherwise in
/// name order, then `using namespace editor::kernels;`. Angle-bracket includes stay.
///
/// Use when: regenerating ShaderLibrarySource.cpp; testing that it is current
/// Tags: concatenate headers, include order, amalgamation
/// Node: none (shader graph)
std::string shaderLibraryTextFromKernels(const std::filesystem::path& kernelsFolder);

/// The contents of ShaderLibrarySource.cpp for `libraryText`: the text in raw string literals of
/// about 12,000 characters (some compilers limit one literal), joined by one function.
std::string shaderLibrarySourceFile(const std::string& libraryText);

} // namespace editor::shaders
