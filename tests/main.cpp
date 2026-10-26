// The golden tests: every case in tests/golden/*.json (written by gen.ts from the TypeScript) run through the C++
// function of the same name, registered by tests/<file>.cpp, and compared with what the TS returned. Run with ctest,
// or directly to see failures; `curves_tests fit.json` runs one file.
#include "cases.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace {

using golden::Fns;
using json::Value;

/** Runs one golden file; returns the number of failures. */
int run(const std::string& file, const Fns& fns) {
  std::ifstream in(std::string(CURVES_GOLDEN_DIR) + "/" + file);
  if (!in) { std::printf("%s: missing (run tests/golden/gen.ts)\n", file.c_str()); return 1; }
  std::stringstream ss;
  ss << in.rdbuf();
  Value cases = json::parse(ss.str());
  int failed = 0;
  for (auto& c : cases.arr()) {
    const std::string& fn = c["fn"].str();
    auto f = fns.find(fn);
    if (f == fns.end()) { std::printf("%s: no C++ for %s\n", file.c_str(), fn.c_str()); failed++; continue; }
    Value got;
    try { got = f->second(c["args"]); } catch (const std::exception& e) { got = std::string("threw: ") + e.what(); }
    if (!json::close(c["out"], got, 1e-9)) {
      failed++;
      std::printf("FAIL %s%s\n  want %s\n  got  %s\n", fn.c_str(), json::print(c["args"]).c_str(),
                  json::print(c["out"]).c_str(), json::print(got).c_str());
    }
  }
  std::printf("%s: %zu cases, %d failed\n", file.c_str(), cases.size(), failed);
  return failed;
}

}  // namespace

int main(int argc, char** argv) {
  int failed = 0;
  auto& files = golden::registry();
  if (argc > 1) {
    for (int i = 1; i < argc; i++) {
      auto f = files.find(argv[i]);
      if (f == files.end()) { std::printf("%s: no tests registered\n", argv[i]); failed++; }
      else failed += run(f->first, f->second);
    }
    return failed ? 1 : 0;
  }
  // Every golden file needs its tests, and every registered file its golden values.
  for (auto& e : std::filesystem::directory_iterator(CURVES_GOLDEN_DIR))
    if (e.path().extension() == ".json" && !files.count(e.path().filename().string())) {
      std::printf("%s: no tests registered\n", e.path().filename().string().c_str());
      failed++;
    }
  for (auto& [file, fns] : files) failed += run(file, fns);
  return failed ? 1 : 0;
}
