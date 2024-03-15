#version 330 core
out vec4 FragColor;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

struct Light {
    vec3 direction;
    vec3 lightPos;

    vec3 color;
};

in vec3 FragPos;
in vec3 Normal;

uniform vec3 viewPos;
uniform Material material;
uniform Light light;

void main()
{
    vec3 rimColor = vec3(1.0f,1.0f,1.0f);
    float rimAmount = 0.716;
    float rimThreshold = 0.1;

    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 lightDir = normalize(light.lightPos - FragPos);
    vec3 normal = normalize(Normal);
    float NdotL = max(dot(normal, lightDir), 0.0);
    float lightIntensity = smoothstep(0, 0.01, NdotL);

    vec3 light_col = light.color * lightIntensity;

    vec3 halfVector = normalize(light.lightPos + viewDir);
    float NdotH = dot(normal, halfVector);
    float specularIntensity = pow(NdotH * lightIntensity, material.shininess * material.shininess);
    float specularIntensitySmooth = smoothstep(0.005, 0.01, specularIntensity);
    vec3 specular = specularIntensitySmooth * material.specular;

    float rimDot = 1 - dot(viewDir, normal);
    float rimIntensity = rimDot * pow(NdotL, rimThreshold);
    rimIntensity = smoothstep(rimAmount - 0.01, rimAmount + 0.01, rimIntensity);
    vec3 rim = rimIntensity * rimColor;


    vec3 result = (light_col + material.ambient  + specular + rim) * material.diffuse;

    FragColor = vec4(result, 1);
}