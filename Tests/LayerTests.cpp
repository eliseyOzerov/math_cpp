// The include rule of Docs/Foundations.md: every module folder belongs to a module, every
// module to a layer, and a file may include only from its own module or a lower layer (plus the
// few sideways uses listed below). Also checks that every shared kernel is compiled as Metal.
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>

#include "Testing.h"

namespace {

namespace filesystem = std::filesystem;

struct Module {
    std::string name;
    int layer;
};

/// The module folders and their layers (Docs/Foundations.md). A new folder must be added here,
/// or to the folders that hold no module, or the test fails.
const std::map<std::string, Module>& moduleOfFolder() {
    static const std::map<std::string, Module> modules = {
        {"Kernels", {"kernels", 0}},
        {"Linear", {"linear", 0}},
        {"Numerics", {"numerics", 1}},
        {"Color", {"color", 1}},
        {"Geometry", {"geometry", 2}},
        {"Curves", {"curves", 2}},
        {"Mesh", {"mesh", 3}},
        {"Fields", {"fields", 3}},
        {"Optics", {"optics", 3}},
        {"Physics", {"physics", 4}},
        {"Animation", {"animation", 4}},
        {"Shaders", {"shaders", 4}},
        {"Shaders/Nodes", {"shaders", 4}},
    };
    return modules;
}

/// Top-level folders that are not modules: tests, tools, docs, the Swift package's wrappers and
/// build output.
const std::set<std::string>& foldersWithoutModule() {
    static const std::set<std::string> folders = {
        "Tests", "Tools", "Docs", "Swift", "build", ".build", ".git", ".swiftpm",
    };
    return folders;
}

/// Uses within one layer that Foundations.md allows: (from module, to module).
const std::set<std::pair<std::string, std::string>>& allowedSideways() {
    static const std::set<std::pair<std::string, std::string>> pairs = {
        {"linear", "kernels"}, // Scalar.h's clamp and blend are the shared kernels
        {"curves", "geometry"}, // curves measure against lines, planes and bounds
        {"fields", "mesh"},     // narrow-band meshing and convex hulls build meshes
    };
    return pairs;
}

filesystem::path sourceFolder() { return filesystem::path(MATH_CPP_FOLDER); }

/// The top-level folders that hold modules (Shaders/Nodes is found inside Shaders).
std::vector<std::string> topLevelModuleFolders() {
    std::vector<std::string> folders;
    for (const auto& [folder, module] : moduleOfFolder()) {
        if (folder.find('/') == std::string::npos) folders.push_back(folder);
    }
    return folders;
}

bool isSourceFile(const filesystem::path& path) { return path.extension() == ".h" || path.extension() == ".cpp"; }

/// The quoted include names in `path`, such as "../Math/Vector3.h".
std::vector<std::string> quotedIncludes(const filesystem::path& path) {
    std::vector<std::string> includes;
    std::ifstream file(path);
    std::string line;
    std::string directive = "#include \"";
    while (std::getline(file, line)) {
        if (line.rfind(directive, 0) != 0) continue;
        std::size_t end = line.find('"', directive.size());
        if (end != std::string::npos) includes.push_back(line.substr(directive.size(), end - directive.size()));
    }
    return includes;
}

} // namespace

TEST(everyFolderBelongsToALayer) {
    for (auto entry = filesystem::recursive_directory_iterator(sourceFolder()); entry != filesystem::recursive_directory_iterator(); ++entry) {
        if (!entry->is_directory()) continue;
        if (entry.depth() == 0 && foldersWithoutModule().count(entry->path().filename().string()) > 0) {
            entry.disable_recursion_pending();
            continue;
        }
        std::string folder = filesystem::relative(entry->path(), sourceFolder()).generic_string();
        if (moduleOfFolder().count(folder) == 0) {
            editor::testing::fail(__FILE__, __LINE__, folder + " has no layer; add it to moduleOfFolder");
        }
    }
}

TEST(filesIncludeOnlyFromLowerLayers) {
    for (const std::string& moduleFolder : topLevelModuleFolders()) {
        for (const auto& entry : filesystem::recursive_directory_iterator(sourceFolder() / moduleFolder)) {
            if (!entry.is_regular_file() || !isSourceFile(entry.path())) continue;
            std::string file = filesystem::relative(entry.path(), sourceFolder()).generic_string();
            std::string folder = filesystem::relative(entry.path().parent_path(), sourceFolder()).generic_string();
            auto from = moduleOfFolder().find(folder);
            if (from == moduleOfFolder().end()) continue; // reported by everyFolderBelongsToALayer
            for (const std::string& include : quotedIncludes(entry.path())) {
                filesystem::path target = (entry.path().parent_path() / include).lexically_normal();
                std::string targetName = filesystem::relative(target, sourceFolder()).generic_string();
                std::string targetFolder = filesystem::relative(target.parent_path(), sourceFolder()).generic_string();
                auto to = moduleOfFolder().find(targetFolder);
                if (to == moduleOfFolder().end()) {
                    editor::testing::fail(__FILE__, __LINE__, file + " includes " + include + ", outside the modules");
                    continue;
                }
                const Module& source = from->second;
                const Module& destination = to->second;
                bool allowed = source.name == destination.name || destination.layer < source.layer
                    || allowedSideways().count({source.name, destination.name}) > 0;
                if (!allowed) {
                    editor::testing::fail(__FILE__, __LINE__,
                        file + " (" + source.name + ", L" + std::to_string(source.layer) + ") includes " + targetName + " ("
                            + destination.name + ", L" + std::to_string(destination.layer) + ")");
                }
            }
        }
    }
}

TEST(everyKernelIsCompiledAsMetal) {
    // Tests/KernelChecks/<Header>.metal compiles Kernels/<Header>.h; `make kernels` builds them all.
    filesystem::path checks = filesystem::path(MATH_CPP_FOLDER) / "Tests" / "KernelChecks";
    for (const auto& entry : filesystem::directory_iterator(sourceFolder() / "Kernels")) {
        std::string header = entry.path().filename().string();
        filesystem::path check = checks / (entry.path().stem().string() + ".metal");
        bool included = false;
        for (const std::string& include : quotedIncludes(check)) {
            if (filesystem::path(include).filename().string() == header) included = true;
        }
        if (!included) {
            editor::testing::fail(__FILE__, __LINE__,
                "Kernels/" + header + " is not included by Tests/KernelChecks/" + entry.path().stem().string() + ".metal");
        }
    }
}
