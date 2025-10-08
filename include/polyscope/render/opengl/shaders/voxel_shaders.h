// Copyright 2017-2023, Nicholas Sharp and the Polyscope contributors. https://polyscope.run

#pragma once

#include "polyscope/render/opengl/gl_shaders.h"

namespace polyscope {
namespace render {
namespace backend_openGL3 {

// Voxel (3D cube) rendering shaders for point clouds
extern const ShaderStageSpecification FLEX_POINTVOXEL_VERT_SHADER;
extern const ShaderStageSpecification FLEX_POINTVOXEL_GEOM_SHADER;
extern const ShaderStageSpecification FLEX_POINTVOXEL_FRAG_SHADER;

// Rule for voxel culling
extern const ShaderReplacementRule SPHERE_CULLPOS_FROM_CENTER_VOXEL;

} // namespace backend_openGL3
} // namespace render
} // namespace polyscope
