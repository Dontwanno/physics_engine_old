#version 410 core
out vec4 FragColor;

in vec2 TexCoords;
uniform int 		width;
uniform int 		height;

uniform sampler2D firstScreenTex;
uniform sampler2D secondScreenTex;
uniform sampler2D depthMap;


float gray(vec4 color) {
	float gray = dot(color.rgb, vec3(0.2126, 0.7152, 0.0722));
	return gray;
	}

const float offset = 1.0 / 300.0;

float near = 0.1;
float far  = 100.0;

float LinearizeDepth(float depth)
{
	float z = depth * 2.0 - 1.0; // back to NDC
	return (2.0 * near * far) / (far + near - z * (far - near));
}


void main()
{
	vec2 offsets[9] = vec2[](
	vec2(-offset,  offset), // top-left
	vec2( 0.0f,    offset), // top-center
	vec2( offset,  offset), // top-right
	vec2(-offset,  0.0f),   // center-left
	vec2( 0.0f,    0.0f),   // center-center
	vec2( offset,  0.0f),   // center-right
	vec2(-offset, -offset), // bottom-left
	vec2( 0.0f,   -offset), // bottom-center
	vec2( offset, -offset)  // bottom-right
	);

	float kernel[9] = float[](
	1, 1, 1,
	1, -8, 1,
	1, 1, 1
	);

	float kernel_x[9] = float[](
	1, 0, -1,
	2, 0, -2,
	1, 0, -1
	);

	float kernel_y[9] = float[](
	1, 2, 1,
	0, 0, 0,
	-1, -2, -1
	);



	vec3 sampleTex[9];
	vec3 sampleTex2[9];

	for(int i = 0; i < 9; i++)
	{
		sampleTex[i] = vec3(texture(secondScreenTex, TexCoords.st + offsets[i]).rgb);
	}

	float greySampleTex[9];
	for(int i = 0; i < 9; i++)
	{
		greySampleTex[i] = gray(texture(secondScreenTex, TexCoords.st + offsets[i]));
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

//	FragColor = vec4(final_col, 1.0);
//	FragColor = vec4(vec3(final_grey), 1.0);

	float result = 1 - pow(final_grey, 1.0);
//	FragColor = vec4(col, 1.0);

//	FragColor = vec4(result, result, result, 1.0);

	float depthValue = texture(depthMap, TexCoords).r;

	float depth = LinearizeDepth(depthValue); // divide by far for demonstration
	FragColor = vec4(vec3(depth), 1.0);
//	FragColor = vec4(vec3(depthValue), 1.0);

//	if (FragColor.r > 1)
//		FragColor = vec4(1.0, 1.0, 1.0, 1.0);
//	else
//		FragColor = vec4(0.0, 0.0, 0.0, 1.0);
}