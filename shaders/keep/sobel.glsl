#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
uniform float 		width;
uniform float 		height;

uniform sampler2D screenTexture;



void make_kernel(inout vec4 n[9], sampler2D tex, vec2 coord)
{
	float w = 1.0 / width;
	float h = 1.0 / height;

	n[0] = texture2D(tex, coord + vec2( -w, -h));
	n[1] = texture2D(tex, coord + vec2(0.0, -h));
	n[2] = texture2D(tex, coord + vec2(  w, -h));
	n[3] = texture2D(tex, coord + vec2( -w, 0.0));
	n[4] = texture2D(tex, coord);
	n[5] = texture2D(tex, coord + vec2(  w, 0.0));
	n[6] = texture2D(tex, coord + vec2( -w, h));
	n[7] = texture2D(tex, coord + vec2(0.0, h));
	n[8] = texture2D(tex, coord + vec2(  w, h));
}

void main()
{
	vec4 n[9];
	make_kernel( n, screenTexture, TexCoords.st );

	vec4 sobel_edge_h = n[2] + (2.0*n[5]) + n[8] - (n[0] + (2.0*n[3]) + n[6]);
  	vec4 sobel_edge_v = n[0] + (2.0*n[1]) + n[2] - (n[6] + (2.0*n[7]) + n[8]);
	vec4 sobel = sqrt((sobel_edge_h * sobel_edge_h) + (sobel_edge_v * sobel_edge_v));
	float abs = sobel.r + sobel.g + sobel.b;
	if (abs > 0.4) {
		FragColor = vec4(0.0, 0.0, 0.0, 1.0);
	} else {
		FragColor = vec4(1.0, 1.0, 1.0, 1.0);
	}
}