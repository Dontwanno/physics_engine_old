#version 330 core
layout (location = 0) out vec3 gColor;
layout (location = 1) out vec3 gNormal;

in vec2 TexCoords;
in vec3 FragPos;
in vec3 Normal;


void main()
{
    // also store the per-fragment normals into the gbuffer
    gNormal = normalize(Normal);
    // and the diffuse per-fragment color
    gColor = vec3(0.4, 0.4, 0);
}