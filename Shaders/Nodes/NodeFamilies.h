// The families of node kinds, in menu order. To add a family, write its file in this folder
// (a function `NodeGroup <name>()`, see Values.cpp) and add one line to the list below.
#pragma once

#include "../NodeSpec.h"

#define EDITOR_SHADER_NODE_FAMILIES(family) \
    family(valueNodes)                      \
    family(vectorNodes)                     \
    family(vectorMathNodes)                 \
    family(transformNodes)                  \
    family(mathNodes)                       \
    family(trigonometryNodes)               \
    family(waveNodes)                       \
    family(noiseNodes)                      \
    family(patternNodes)                    \
    family(shapeNodes)                      \
    family(sdfOperationNodes)               \
    family(colorNodes)                      \
    family(blendingNodes)                   \
    family(textureNodes)                    \
    family(imageEffectNodes)                \
    family(timeNodes)                       \
    family(interactionNodes)                \
    family(raymarchingNodes)                \
    family(simulationNodes)                 \
    family(utilityNodes)                    \
    family(outputNodes)

namespace editor::shaders {

#define EDITOR_DECLARE_FAMILY(name) NodeGroup name();
EDITOR_SHADER_NODE_FAMILIES(EDITOR_DECLARE_FAMILY)
#undef EDITOR_DECLARE_FAMILY

} // namespace editor::shaders
