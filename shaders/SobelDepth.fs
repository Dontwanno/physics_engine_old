#version 330 core


// Source image
uniform sampler2D _MainTex;
// Camera Normals
uniform sampler2D _CameraDepthNormalsTexture;
// Camera Depth
uniform sampler2D _CameraDepthTexture;
vec4 _CameraDepthTexture_ST;

// Settings
vec4 _Sensitivity = vec4(1.0f, 1.0f, 1.0f, 1.0f);
vec4 _BgColor = vec4(0.1f, 0.1f, 0.1f, 1.0f);
float _BgFade = 1.0f;
float _SampleDistance = 1.0f;
float _Exponent = 1.0f;
float _Threshold = 0.2f;
vec4 _EdgesColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);
uniform int 		width;
uniform int 		height;


float SAMPLE_DEPTH_TEXTURE(sampler2D texture, texcoord) {
    return texture(texture, texcoord.st).r;
}

float near = 0.1;
float far  = 100.0;

float Linear01Depth(float depth)
{
    float z = depth * 2.0 - 1.0; // back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    half4 color = texture(_MainTex, sampler_MainTex, i.texcoord[0]);

    float centerDepth = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]));

    vec4 depthsDiag;
    vec4 depthsAxis;

    vec2 uvDist = _SampleDistance * _MainTex_TexelSize.xy;

    vec2 _CameraDepthTexture_ST = _CameraDepthTexture.st;

    depthsDiag.x = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]+uvDist, _CameraDepthTexture_ST)); // TR
    depthsDiag.y = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]+uvDist*vec2(-1,1), _CameraDepthTexture_ST)); // TL
    depthsDiag.z = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]-uvDist*vec2(-1,1), _CameraDepthTexture_ST)); // BR
    depthsDiag.w = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]-uvDist, _CameraDepthTexture_ST)); // BL

    depthsAxis.x = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]+uvDist*vec2(0,1), _CameraDepthTexture_ST)); // T
    depthsAxis.y = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]-uvDist*vec2(1,0), _CameraDepthTexture_ST)); // L
    depthsAxis.z = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]+uvDist*vec2(1,0), _CameraDepthTexture_ST)); // R
    depthsAxis.w = Linear01Depth(SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.texcoord[1]-uvDist*vec2(0,1), _CameraDepthTexture_ST)); // B

    depthsDiag = (depthsDiag > vec4(centerDepth)) ? depthsDiag : vec4(centerDepth);
    depthsAxis = (depthsAxis > vec4(centerDepth)) ? depthsAxis : vec4(centerDepth);

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


    Sobel = 1.0-pow(clamp(0.0, 1.0, Sobel), _Exponent);
    color = mix(_EdgesColor, mix(color, _BgColor, _BgFade), Sobel);
    return color;
}
