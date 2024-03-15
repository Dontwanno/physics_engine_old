#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D firstScreenTex;
uniform sampler2D secondScreenTex;

void main()
{
    vec3 col1 = texture(firstScreenTex, TexCoords).rgb;
    vec3 col2 = texture(secondScreenTex, TexCoords).rgb;
    vec3 result = col1 + col2;
    if (col2 == vec3(0.1))
        result = vec3(1.0);
    FragColor = vec4(result, 1.0);
}