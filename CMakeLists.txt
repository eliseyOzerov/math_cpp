cmake_minimum_required(VERSION 3.20)
project(graphics_cpp VERSION 0.1.0 LANGUAGES CXX)

# Generic curve maths ported from fountain-ts/src/curves: a C++20 library with a C API (include/curves.h) for Swift
# and wasm. See README.md.

option(CURVES_TESTS "Build the golden tests" ${PROJECT_IS_TOP_LEVEL})

file(GLOB CURVES_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(curves ${CURVES_SOURCES})
add_library(curves::curves ALIAS curves)
target_include_directories(curves PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(curves PUBLIC cxx_std_20)
if(NOT MSVC)
  # -ffp-contract=off too: the sources round like JS (see the pragma in src/), whatever a compiler defaults to.
  target_compile_options(curves PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
endif()

# Emscripten: `emcmake cmake -B build-wasm && cmake --build build-wasm` also links curves.js/curves.wasm: the embind
# API of src/wasm.cpp, shaped like the TS, and the C API (README.md, "From TypeScript"). The golden tests run under
# node there too, reading the JSON from disk (NODERAWFS).
if(EMSCRIPTEN)
  add_executable(curves_wasm ${CURVES_SOURCES})
  target_include_directories(curves_wasm PRIVATE include)
  target_compile_features(curves_wasm PRIVATE cxx_std_20)
  target_compile_options(curves_wasm PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
  set_target_properties(curves_wasm PROPERTIES OUTPUT_NAME curves)
  target_link_options(curves_wasm PRIVATE
    -lembind -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createCurves -sALLOW_MEMORY_GROWTH=1
    -sNO_EXIT_RUNTIME=1 -sDISABLE_EXCEPTION_CATCHING=0
    "-sEXPORTED_FUNCTIONS=['_malloc','_free','_cv_bez','_cv_split','_cv_arc_length','_cv_cubic_roots','_cv_extrema','_cv_bbox','_cv_inflections','_cv_canonical','_cv_project','_cv_mold','_cv_cubic_through']"
    "-sEXPORTED_RUNTIME_METHODS=['HEAPF64','ccall','cwrap']")
  target_compile_options(curves PRIVATE -fexceptions)
  target_compile_options(curves_wasm PRIVATE -fexceptions)
  # The module's TypeScript declarations, next to curves.js where TS looks for them.
  add_custom_command(TARGET curves_wasm POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/wasm.d.ts $<TARGET_FILE_DIR:curves_wasm>/curves.d.ts)
endif()

if(CURVES_TESTS)
  enable_testing()
  file(GLOB CURVES_TEST_SOURCES CONFIGURE_DEPENDS tests/*.cpp)
  add_executable(curves_tests ${CURVES_TEST_SOURCES})
  target_link_libraries(curves_tests PRIVATE curves)
  target_compile_definitions(curves_tests PRIVATE CURVES_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/golden")
  if(EMSCRIPTEN)
    target_compile_options(curves_tests PRIVATE -fexceptions)
    target_link_options(curves_tests PRIVATE -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH=1 -sDISABLE_EXCEPTION_CATCHING=0)
  endif()
  add_test(NAME golden COMMAND curves_tests)
endif()

include(GNUInstallDirs)
install(TARGETS curves EXPORT curvesTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT curvesTargets NAMESPACE curves:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/curves)
