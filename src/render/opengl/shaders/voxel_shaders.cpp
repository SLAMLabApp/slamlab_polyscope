// Copyright 2017-2023, Nicholas Sharp and the Polyscope contributors. http://polyscope.run

#include "polyscope/render/opengl/shaders/voxel_shaders.h"


namespace polyscope {
namespace render {
namespace backend_openGL3 {

// clang-format off

//  These POINTVOXEL shaders render a voxel (cube) at the location of the point.
//  Uses a geometry shader to expand each point into a cube (36 vertices for 12 triangles).

const ShaderStageSpecification FLEX_POINTVOXEL_VERT_SHADER = {

    ShaderStageType::Vertex,

    // uniforms
    {
    }, 

    // attributes
    {
        {"a_position", RenderDataType::Vector3Float},
    },

    {}, // textures

    // source
R"(
        ${ GLSL_VERSION }$

        in vec3 a_position;
        out vec3 worldPosition;
        
        ${ VERT_DECLARATIONS }$
        
        void main()
        {
            // Pass world position to geometry shader
            worldPosition = a_position;
            gl_Position = vec4(a_position, 1.0);

            ${ VERT_ASSIGNMENTS }$
        }
)"
};

const ShaderStageSpecification FLEX_POINTVOXEL_GEOM_SHADER = {
    
    ShaderStageType::Geometry,
    
    // uniforms
    {
        {"u_modelView", RenderDataType::Matrix44Float},
        {"u_projMatrix", RenderDataType::Matrix44Float},
        {"u_pointRadius", RenderDataType::Float},
    }, 

    // attributes
    {
    },

    {}, // textures

    // source
R"(
        ${ GLSL_VERSION }$

        layout(points) in;
        layout(triangle_strip, max_vertices=36) out;
        in vec3 worldPosition[];
        uniform mat4 u_modelView;
        uniform mat4 u_projMatrix;
        uniform float u_pointRadius;
        out vec3 voxelNormal;
        out vec2 quadCoord;

        ${ GEOM_DECLARATIONS }$

        void main() {
           
            float pointRadius = u_pointRadius;
            ${ SPHERE_SET_POINT_RADIUS_GEOM }$
            
            // Construct the 8 corners of a cube centered at the point in WORLD space
            vec3 centerWorld = worldPosition[0];
            float r = pointRadius;
            
            // Define 8 corners of the cube in WORLD space - aligned to world axes
            vec3 v0_world = centerWorld + vec3(-r, -r, -r);
            vec3 v1_world = centerWorld + vec3( r, -r, -r);
            vec3 v2_world = centerWorld + vec3( r,  r, -r);
            vec3 v3_world = centerWorld + vec3(-r,  r, -r);
            vec3 v4_world = centerWorld + vec3(-r, -r,  r);
            vec3 v5_world = centerWorld + vec3( r, -r,  r);
            vec3 v6_world = centerWorld + vec3( r,  r,  r);
            vec3 v7_world = centerWorld + vec3(-r,  r,  r);
            
            // Transform to view space, then to clip space
            mat4 MVP = u_projMatrix * u_modelView;
            vec4 v0 = MVP * vec4(v0_world, 1.0);
            vec4 v1 = MVP * vec4(v1_world, 1.0);
            vec4 v2 = MVP * vec4(v2_world, 1.0);
            vec4 v3 = MVP * vec4(v3_world, 1.0);
            vec4 v4 = MVP * vec4(v4_world, 1.0);
            vec4 v5 = MVP * vec4(v5_world, 1.0);
            vec4 v6 = MVP * vec4(v6_world, 1.0);
            vec4 v7 = MVP * vec4(v7_world, 1.0);
            
            ${ GEOM_COMPUTE_BEFORE_EMIT }$
    
            // Emit 6 cube faces (12 triangles, 36 vertices total)
            // Normals are in WORLD space aligned to axes, transformed to view space
            mat3 normalMatrix = mat3(u_modelView);
            
            // Front face (z+) - world space normal
            vec3 norm_front = normalMatrix * vec3(0, 0, 1);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(0,0); gl_Position = v4; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(1,0); gl_Position = v5; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(0,1); gl_Position = v7; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(1,0); gl_Position = v5; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(1,1); gl_Position = v6; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_front; quadCoord = vec2(0,1); gl_Position = v7; EmitVertex();
            EndPrimitive();
            
            // Back face (z-) - world space normal
            vec3 norm_back = normalMatrix * vec3(0, 0, -1);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(0,0); gl_Position = v0; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(0,1); gl_Position = v3; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(1,0); gl_Position = v1; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(1,0); gl_Position = v1; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(0,1); gl_Position = v3; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_back; quadCoord = vec2(1,1); gl_Position = v2; EmitVertex();
            EndPrimitive();
            
            // Right face (x+) - world space normal
            vec3 norm_right = normalMatrix * vec3(1, 0, 0);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(0,0); gl_Position = v1; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(0,1); gl_Position = v2; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(1,0); gl_Position = v5; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(0,1); gl_Position = v2; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(1,1); gl_Position = v6; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_right; quadCoord = vec2(1,0); gl_Position = v5; EmitVertex();
            EndPrimitive();
            
            // Left face (x-) - world space normal
            vec3 norm_left = normalMatrix * vec3(-1, 0, 0);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(0,0); gl_Position = v0; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(1,0); gl_Position = v4; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(0,1); gl_Position = v3; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(0,1); gl_Position = v3; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(1,0); gl_Position = v4; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_left; quadCoord = vec2(1,1); gl_Position = v7; EmitVertex();
            EndPrimitive();
            
            // Top face (y+) - world space normal
            vec3 norm_top = normalMatrix * vec3(0, 1, 0);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(0,0); gl_Position = v3; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(0,1); gl_Position = v7; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(1,0); gl_Position = v2; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(1,0); gl_Position = v2; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(0,1); gl_Position = v7; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_top; quadCoord = vec2(1,1); gl_Position = v6; EmitVertex();
            EndPrimitive();
            
            // Bottom face (y-) - world space normal
            vec3 norm_bottom = normalMatrix * vec3(0, -1, 0);
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(0,0); gl_Position = v0; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(1,0); gl_Position = v1; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(0,1); gl_Position = v4; EmitVertex();
            EndPrimitive();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(1,0); gl_Position = v1; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(1,1); gl_Position = v5; EmitVertex();
            ${ GEOM_PER_EMIT }$ voxelNormal = norm_bottom; quadCoord = vec2(0,1); gl_Position = v4; EmitVertex();
            EndPrimitive();
        }

)"
};


const ShaderStageSpecification FLEX_POINTVOXEL_FRAG_SHADER = {
    
    ShaderStageType::Fragment,
    
    // uniforms
    {
        {"u_projMatrix", RenderDataType::Matrix44Float},
        {"u_pointRadius", RenderDataType::Float},
        {"u_edgeWidth", RenderDataType::Float},
        {"u_edgeColor", RenderDataType::Vector3Float},
    }, 

    { }, // attributes
    
    // textures 
    {
    },
 
    // source
R"(
        ${ GLSL_VERSION }$
        uniform mat4 u_projMatrix; 
        uniform float u_pointRadius;
        uniform float u_edgeWidth;
        uniform vec3 u_edgeColor;
        in vec3 voxelNormal;
        in vec2 quadCoord;
        layout(location = 0) out vec4 outputF;

        float LARGE_FLOAT();
        vec3 lightSurfaceMat(vec3 normal, vec3 color, sampler2D t_mat_r, sampler2D t_mat_g, sampler2D t_mat_b, sampler2D t_mat_k);
        
        ${ FRAG_DECLARATIONS }$

        void main()
        {
           
           float depth = gl_FragCoord.z;
           ${ GLOBAL_FRAGMENT_FILTER_PREP }$
           ${ GLOBAL_FRAGMENT_FILTER }$

           float pointRadius = u_pointRadius;
           ${ SPHERE_SET_POINT_RADIUS_FRAG }$
          
           // Shading
           ${ GENERATE_SHADE_VALUE }$
           ${ GENERATE_SHADE_COLOR }$

           // Lighting - use the face normal from geometry shader
           vec3 shadeNormal = normalize(voxelNormal);
           ${ GENERATE_LIT_COLOR }$

           // Compute edge based on quad UV coordinates (0-1 on each face)
           // Distance to nearest edge of the quad (not triangle diagonals)
           vec2 fw = fwidth(quadCoord);
           vec2 distToEdgeBottom = quadCoord / fw;
           vec2 distToEdgeTop = (vec2(1.0) - quadCoord) / fw;
           float minDistToEdge = min(min(distToEdgeBottom.x, distToEdgeBottom.y), 
                                      min(distToEdgeTop.x, distToEdgeTop.y));
           
           // Mix between edge color and surface color based on distance
           float edgeFactor = smoothstep(u_edgeWidth - 0.5, u_edgeWidth + 0.5, minDistToEdge);
           vec3 finalColor = mix(u_edgeColor, litColor, edgeFactor);

           // Set alpha
           float alphaOut = 1.0;
           ${ GENERATE_ALPHA }$

           // Write output
           finalColor *= alphaOut; // premultiplied alpha
           outputF = vec4(finalColor, alphaOut);
        }
)"
};


// == Rules

const ShaderReplacementRule SPHERE_CULLPOS_FROM_CENTER_VOXEL(
    /* rule name */ "SPHERE_CULLPOS_FROM_CENTER_VOXEL",
    { /* replacement sources */
      {"GEOM_DECLARATIONS", R"(
          out vec3 sphereCenterView;
        )"},
      {"GEOM_COMPUTE_BEFORE_EMIT", R"(
          vec3 sphereCenterViewVal = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
        )"},
      {"GEOM_PER_EMIT", R"(
          sphereCenterView = sphereCenterViewVal;
        )"},
      {"FRAG_DECLARATIONS", R"(
          in vec3 sphereCenterView;
        )"},
      {"GLOBAL_FRAGMENT_FILTER_PREP", R"(
          vec3 cullPos = sphereCenterView;
        )"},
    },
    /* uniforms */ {},
    /* attributes */ {},
    /* textures */ {}
);

// clang-format on

} // namespace backend_openGL3
} // namespace render
} // namespace polyscope
