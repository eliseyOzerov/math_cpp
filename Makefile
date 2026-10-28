# Builds every module as its own static library, and the tests, on macOS (or any machine with clang).
#
#   make            one library per module: build/lib/libmath_<module>.a
#   make test       builds and runs every test (and `make kernels`, `make gpu`, `make shaders`)
#   make kernels    checks the shared kernels compile as C++14 and, where Xcode's Metal
#                   compiler is installed, as Metal Shading Language
#   make gpu        runs the shared kernels on the GPU and the CPU and compares them
#                   (Tests/GPUAgreement; also run by `make test` where Metal is installed)
#   make shaders    compiles the preview of every node kind with Metal's runtime compiler
#                   (Tests/ShaderPreviews; also run by `make test` where Metal is installed)
#   make shader-library  regenerates Shaders/ShaderLibrarySource.cpp from the kernels
#   make libraries-of MODULE=Fields   prints the libraries a program using Fields links, in order
#   BUILD=build/x   builds somewhere else, so two builds can run at once
#
# The folder path may contain spaces, so sources are named relative to this folder and the
# absolute path only appears inside a quoted define.

CXX := clang++
BUILD ?= build
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -Wshadow -Werror -ffp-contract=off -MMD -MP
# -ffp-contract=off: no fused multiply-adds, so a*b+c rounds twice, as it does in Swift and on
# every other machine, and results agree to the last bit.

ROOT_FOLDER := -DMATH_CPP_FOLDER='"$(CURDIR)"'

# The modules, lowest layer first (Docs/Foundations.md). Kernels is header-only.
MODULES := Linear Numerics Color Geometry Curves Mesh Fields Optics Physics Animation Shaders

# What each module includes from the others, directly (Tests/LayerTests.cpp keeps this honest).
DEPENDS_Linear :=
DEPENDS_Numerics := Linear
DEPENDS_Color := Linear
DEPENDS_Geometry := Linear
DEPENDS_Curves := Geometry Numerics Linear
DEPENDS_Mesh := Curves Geometry Numerics Linear
DEPENDS_Fields := Mesh Curves Geometry Numerics Linear
DEPENDS_Optics := Color Geometry Linear
DEPENDS_Physics := Linear
DEPENDS_Animation := Linear
DEPENDS_Shaders :=

library = $(BUILD)/lib/libmath_$(shell echo $(1) | tr A-Z a-z).a
module_sources = $(shell find $(1) -name "*.cpp" | sort)
module_objects = $(patsubst %.cpp,$(BUILD)/obj/%.o,$(call module_sources,$(1)))

LIBRARIES := $(foreach module,$(MODULES),$(call library,$(module)))
# Highest layer first, so the linker meets a library before the ones it needs.
LINK_ORDER := $(foreach module,Shaders Animation Physics Optics Fields Mesh Curves Geometry Color Numerics Linear,$(call library,$(module)))

all: $(LIBRARIES)

define MODULE_RULE
$(call library,$(1)): $(call module_objects,$(1))
	@mkdir -p $$(dir $$@)
	rm -f $$@
	ar rcs $$@ $$^
endef
$(foreach module,$(MODULES),$(eval $(call MODULE_RULE,$(module))))

libraries-of:
	@echo $(call library,$(MODULE)) $(foreach module,$(DEPENDS_$(MODULE)),$(call library,$(module)))

$(BUILD)/obj/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(ROOT_FOLDER) -c $< -o $@

TEST_SOURCES := $(wildcard Tests/*.cpp)
TEST_OBJECTS := $(TEST_SOURCES:%.cpp=$(BUILD)/obj/%.o)
KERNEL_HEADERS := $(wildcard Kernels/*.h)

# Metal 3.0 is the oldest Metal Shading Language this compiler accepts that is still based on
# C++14, the floor the kernels keep to (Docs/Foundations.md).
METAL := $(shell xcrun -f metal 2>/dev/null)
METAL_FLAGS := -std=metal3.0 -Werror

$(BUILD)/math-tests: $(TEST_OBJECTS) $(LIBRARIES)
	$(CXX) $(CXXFLAGS) $(TEST_OBJECTS) $(LINK_ORDER) -o $@

$(BUILD)/kernels/cpp14.checked: $(KERNEL_HEADERS)
	@mkdir -p $(dir $@)
	for header in $(KERNEL_HEADERS); do \
		$(CXX) -std=c++14 -pedantic -Wall -Wextra -Wshadow -Werror -fsyntax-only -x c++-header "$$header" || exit 1; \
	done
	touch $@

# One check per kernel header (Tests/KernelChecks/<Header>.metal), so kernels added side by side
# never edit the same file.
KERNEL_CHECKS := $(wildcard Tests/KernelChecks/*.metal)
KERNEL_AIR := $(KERNEL_CHECKS:Tests/KernelChecks/%.metal=$(BUILD)/kernels/%.air)

$(BUILD)/kernels/%.air: Tests/KernelChecks/%.metal $(KERNEL_HEADERS)
	@mkdir -p $(dir $@)
	xcrun metal $(METAL_FLAGS) -c $< -o $@

ifneq ($(METAL),)
kernels: $(BUILD)/kernels/cpp14.checked $(KERNEL_AIR)
else
kernels: $(BUILD)/kernels/cpp14.checked
	@echo "kernels: no Metal compiler (xcrun metal), so the Metal check was skipped"
endif

# GPU agreement (Tests/GPUAgreement): every .metal there goes into one library, built with
# Metal's default fast math as real shaders are; the .cpp cases and the Objective-C++ runner link
# against Metal. The CPU result is the reference.
AGREEMENT_METAL := $(wildcard Tests/GPUAgreement/*.metal)
AGREEMENT_AIR := $(AGREEMENT_METAL:Tests/GPUAgreement/%.metal=$(BUILD)/agreement/%.air)
AGREEMENT_OBJECTS := $(patsubst %.cpp,$(BUILD)/obj/%.o,$(wildcard Tests/GPUAgreement/*.cpp)) \
	$(BUILD)/obj/Tests/GPUAgreement/Runner.o

$(BUILD)/agreement/%.air: Tests/GPUAgreement/%.metal $(KERNEL_HEADERS)
	@mkdir -p $(dir $@)
	xcrun metal -std=metal3.0 -Werror -c $< -o $@

$(BUILD)/agreement/agreement.metallib: $(AGREEMENT_AIR)
	xcrun metallib $^ -o $@

$(BUILD)/obj/%.o: %.mm
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -fobjc-arc -c $< -o $@

$(BUILD)/gpu-agreement: $(AGREEMENT_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -framework Metal -framework Foundation -o $@

ifneq ($(METAL),)
gpu: $(BUILD)/gpu-agreement $(BUILD)/agreement/agreement.metallib
	./$(BUILD)/gpu-agreement $(BUILD)/agreement/agreement.metallib
else
gpu:
	@echo "gpu: no Metal compiler (xcrun metal), so the GPU agreement cases were skipped"
endif

# Shader library: ShaderLibrarySource.cpp is generated from Kernels/*.h and committed.
$(BUILD)/shader-library: $(BUILD)/obj/Tools/shader-library.o $(call library,Shaders)
	$(CXX) $(CXXFLAGS) $^ -o $@

shader-library: $(BUILD)/shader-library
	./$(BUILD)/shader-library

# Shader previews (Tests/ShaderPreviews): every preview of every node kind goes through Metal's
# runtime compiler, as graphics_metal's apps do (default options, so fast math and the system's
# language version), and each compiled shader's vertex and fragment functions build a pipeline.
$(BUILD)/shader-previews: $(BUILD)/obj/Tests/ShaderPreviews/Runner.o $(call library,Shaders)
	$(CXX) $(CXXFLAGS) $^ -framework Metal -framework Foundation -o $@

ifneq ($(METAL),)
shaders: $(BUILD)/shader-previews
	./$(BUILD)/shader-previews
else
shaders:
	@echo "shaders: no Metal compiler (xcrun metal), so the shader preview check was skipped"
endif

test: all kernels $(BUILD)/math-tests gpu shaders
	./$(BUILD)/math-tests

clean:
	rm -rf build

.PHONY: all test kernels gpu shaders shader-library libraries-of clean

-include $(shell find $(BUILD)/obj -name "*.d" 2>/dev/null)
