#version 330 core
out vec4 FragColor;

in vec3 Normal;

float near = 0.1;
float far  = 100.0;

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}


void main()
{
    FragColor = vec4(Normal, 1.0);
//     float depth = LinearizeDepth(gl_FragCoord.z)/ far; // divide by far for demonstration
//     FragColor = vec4(vec3(depth), 1.0);
}