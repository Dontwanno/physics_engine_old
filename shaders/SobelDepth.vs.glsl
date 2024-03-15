#version 410 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoords;

out vec2 TexCoords;
//VaryingsD VertD(AttributesDefault v)
//{
//    VaryingsD o;
//
//    o.vertex = float4(v.vertex.xy, 0.0, 1.0);
//    float2 texcoord = TransformTriangleVertexToUV(v.vertex.xy);
//
//    #if UNITY_UV_STARTS_AT_TOP
//    texcoord = texcoord * float2(1.0, -1.0) + float2(0.0, 1.0);
//    #endif
//
//    o.texcoordStereo = TransformStereoScreenSpaceTex(texcoord, 1.0);
//    o.texcoord[0] = UnityStereoScreenSpaceUVAdjust(texcoord, _MainTex_ST);
//    o.texcoord[1] = texcoord;
//
//    return o;
//}

void main()
{
    TexCoords = aTexCoords;
    gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
}