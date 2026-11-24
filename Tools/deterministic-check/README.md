# deterministic-check

Proves that `Numerics/DeterministicMath` gives the same bits on every platform. The program feeds
each function a fixed, seeded list of 200,000 inputs (plus 27 special values such as ±0, ±∞, NaN,
1e22 and 2^1000) and prints one FNV-1a 64 hash of all the result bit patterns per function. The
printed lines must be identical on every platform; if one differs, a function gave a different
bit somewhere.

Run these from this folder (`Tools/deterministic-check`):

```bash
# Native (Apple arm64 or x86-64)
clang++ -std=c++20 -O2 -ffp-contract=off deterministic-check.cpp ../../Numerics/DeterministicMath.cpp -o check-native
./check-native

# WebAssembly (emscripten), run under node
/opt/homebrew/bin/emcc -std=c++20 -O2 -ffp-contract=off deterministic-check.cpp ../../Numerics/DeterministicMath.cpp -o check.js
node check.js
```

Compare the two outputs, for example `./check-native > native.txt; node check.js > wasm.txt;
diff native.txt wasm.txt`. The same holds with `-O0`, `-O1` and `-O3`. On an Apple Silicon Mac,
adding `-arch x86_64` to the native command builds the x86-64 version, which runs under Rosetta.
