#version 460 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D colorTexture;
uniform sampler2D normalTexture;
uniform sampler2D depthTexture;

float far = 100.0;
float near = 0.1;

int width = 1280;
int height = 800;


float gray(vec3 colorvalue)
{
float gray = dot(colorvalue, vec3(0.2126, 0.7152, 0.0722));
return gray;
}

// Settings
vec4 _Sensitivity = vec4(1.0f, 1.0f, 1.0f, 1.0f);
vec4 _BgColor = vec4(0.1f, 0.1f, 0.1f, 1.0f);
float _BgFade = 1.0f;
float _SampleDistance = 1.0f;
float _Exponent = 1.0f;
float _Threshold = 0.2f;
vec4 _EdgesColor = vec4(0.0f);


bool anyValueLarger(vec4 a, vec4 b) {
    return (a.x > b.x) || (a.y > b.y) || (a.z > b.z) || (a.w > b.w);
}

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0;
    z = (2.0 * near * far) / (far + near - z * (far - near));
    return (z  - near) / (far - near);

}

vec4 SobelColor(vec3 color_in)
{
    //    vec4 color = texture(_MainTex, sampler_MainTex, i.texcoord[0]);
    vec4 color = vec4(color_in, 1.0f);

    vec2 texcoord = TexCoords;

    float centerDepth = LinearizeDepth(texture(depthTexture, TexCoords).r);

    vec4 depthsDiag;
    vec4 depthsAxis;

    vec2 _MainTex_TexelSize;

    _MainTex_TexelSize.x = 1.0 / width;
    _MainTex_TexelSize.y = 1.0 / height;
//    _MainTex_TexelSize.x = _MainTex_TexelSize.y = 1.0 / 300.0;

    vec2 uvDist = _SampleDistance * _MainTex_TexelSize.xy;

    depthsDiag.x = LinearizeDepth(texture(depthTexture, texcoord+uvDist).r); // TR
    depthsDiag.y = LinearizeDepth(texture(depthTexture, texcoord+uvDist*vec2(-1,1)).r); // TL
    depthsDiag.z = LinearizeDepth(texture(depthTexture, texcoord-uvDist*vec2(-1,1)).r); // BR
    depthsDiag.w = LinearizeDepth(texture(depthTexture, texcoord-uvDist).r); // BL

    depthsAxis.x = LinearizeDepth(texture(depthTexture, texcoord+uvDist*vec2(0,1)).r); // T
    depthsAxis.y = LinearizeDepth(texture(depthTexture, texcoord-uvDist*vec2(1,0)).r); // L
    depthsAxis.z = LinearizeDepth(texture(depthTexture, texcoord+uvDist*vec2(1,0)).r); // R
    depthsAxis.w = LinearizeDepth(texture(depthTexture, texcoord-uvDist*vec2(0,1)).r); // B

    depthsDiag = anyValueLarger(depthsDiag ,vec4(centerDepth)) ? depthsDiag : vec4(centerDepth);
    depthsAxis = anyValueLarger(depthsAxis , vec4(centerDepth)) ? depthsAxis : vec4(centerDepth);

    depthsDiag -= centerDepth;
    depthsAxis /= centerDepth;

    const vec4 HorizDiagCoeff = vec4(1,1,-1,-1);
    const vec4 VertDiagCoeff = vec4(-1,1,-1,1);
    const vec4 HorizAxisCoeff = vec4(1,0,0,-1);
    const vec4 VertAxisCoeff = vec4(0,1,-1,0);

    vec4 SobelH = depthsDiag * HorizDiagCoeff + depthsAxis * HorizAxisCoeff;
    vec4 SobelV = depthsDiag * VertDiagCoeff + depthsAxis * VertAxisCoeff;

    float SobelX = dot(SobelH, vec4(1,1,1,1));
    float SobelY = dot(SobelV, vec4(1,1,1,1));
    float Sobel = sqrt(SobelX * SobelX + SobelY * SobelY);


    float SobelExp = 1.0-pow(clamp(1.0, 0.0 , Sobel), _Exponent);
//    color = mix(mix(_BgColor, color, _BgFade), _EdgesColor, SobelExp);
    color = mix(mix(_BgColor, color, _BgFade), _EdgesColor, Sobel);

//    Sobel = 1.0-pow(saturate(Sobel), _Exponent);
//    color = lerp(lerp(_EdgesColor, _FogColor, fog), lerp(color, _BgColor, _BgFade), Sobel);

//    color = mix(_EdgesColor, mix(color, _BgColor, _BgFade), Sobelexp);
//    return vec4(Sobel);
    return vec4(color);
}


void main() {

    vec3 color = texture(colorTexture, TexCoords).rgb;
    vec3 normal = texture(normalTexture, TexCoords).rgb;
    float depth = texture(depthTexture, TexCoords).r;

    float linearDepth = LinearizeDepth(depth);

    float offsetx = 1.0 / width;
    float offsety = 1.0 / height;

    vec2 offsets[9] = {
    vec2(-offsetx, offsety), // top-left
    vec2(0.0f, offsety), // top-center
    vec2(offsetx, offsety), // top-right
    vec2(-offsetx, 0.0f), // center-left
    vec2(0.0f, 0.0f), // center-center
    vec2(offsetx, 0.0f), // center-right
    vec2(-offsetx, -offsety), // bottom-left
    vec2(0.0f, -offsety), // bottom-center
    vec2(offsetx, -offsetx)  // bottom-right
    };

    float kernel[9] = {
    1, 1, 1,
    1, -8, 1,
    1, 1, 1
    };

    float kernel_x[9] = {
    1, 0, -1,
    2, 0, -2,
    1, 0, -1
    };

    float kernel_y[9] = {
    1, 2, 1,
    0, 0, 0,
    -1, -2, -1
    };




    vec3 sampleTex[9];
    for(int i = 0; i < 9; i++)
    {
        sampleTex[i] = vec3(texture(normalTexture, TexCoords.st + offsets[i]));
    }

    float depthSampleTex[9];
    for(int i = 0; i < 9; i++)
    {
        depthSampleTex[i] = LinearizeDepth(texture(depthTexture, TexCoords.st + offsets[i]).r);
    }

    float greySampleTex[9];
    for(int i = 0; i < 9; i++)
    {
        greySampleTex[i] = gray(texture(normalTexture, TexCoords.st + offsets[i]).rgb);
    }

    float greyx = 0.0;
    for(int i = 0; i < 9; i++)
    {
        greyx += greySampleTex[i] * kernel_x[i];
    }

    float greyy = 0.0;
    for(int i = 0; i < 9; i++)
    {
        greyy += greySampleTex[i] * kernel_y[i];
    }

    float final_grey = sqrt((greyx * greyx) + (greyy * greyy));

    float depthx = 0.0;
    for(int i = 0; i < 9; i++)
    {
        depthx += depthSampleTex[i] * kernel_x[i];
    }

    float depthy = 0.0;
    for(int i = 0; i < 9; i++)
    {
        depthy += depthSampleTex[i] * kernel_y[i];
    }

    float final_depth = sqrt((depthx * depthx) + (depthy * depthy));


    vec3 col = vec3(0.0);
    for(int i = 0; i < 9; i++)
    col += sampleTex[i] * kernel[i];

    vec3 colx = vec3(0.0);
    for(int i = 0; i < 9; i++)
    colx += sampleTex[i] * kernel_x[i];

    vec3 coly = vec3(0.0);
    for(int i = 0; i < 9; i++)
    coly += sampleTex[i] * kernel_y[i];

    vec3 final_col = sqrt((colx * colx) + (coly * coly));

    float _OutlineNormalMultiplier = 1.0;
    float _OutlineNormalBias = 1.0;
    float final_depth1 = pow(final_depth * _OutlineNormalMultiplier, _OutlineNormalBias);

    float _OutlineDepthMultiplier = 1.0;
    float _OutlineDepthBias = 1.0;
    float final_grey1 = pow(final_grey * _OutlineDepthMultiplier, _OutlineDepthBias);

    float sobelOutline = clamp(1.0, 0.0, max(final_depth1, final_grey1));

    vec4 sobelCol = SobelColor(color);
    //    FragColor = vec4(color, 1.0);
    //    FragColor = vec4(vec3(depth), 1.0);
//        FragColor = vec4(vec3(normal), 1.0);
//    FragColor = vec4(vec3(linearDepth), 1.0);
    FragColor = sobelCol;
//    FragColor = vec4(vec3(final_col), 1.0);
//    FragColor = vec4(vec3(final_grey), 1.0);
//    FragColor = vec4(vec3(final_depth), 1.0);
//    FragColor = vec4(vec3(sobelCol+final_grey), 1.0);

}