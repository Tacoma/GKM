#version 330

// takes x,y and depth
//   checks validity of depth
//   unprojects point
// computes texture coordinates
// applies model-view-projection matrix
// returns vertex width texture

layout (points) in;
layout (points, max_vertices=1) out;

in ivec2 texture_coordinates_int[];
out vec2 texture_coordinates;

uniform sampler2D depth_texture;
uniform mat4 mvp;
uniform mat4 unproj;

uniform int width;
uniform int height;
uniform int step = 1;
uniform float threshold2 = 0.2*0.2;

uniform float depth_correction = 65535.0f / 1000.0f;

void get_point(in ivec2 coord, out vec4 point, out vec2 tex_coord, out bool valid) {
	float depth = texelFetch(depth_texture, coord, 0).r;
	valid = (depth > 0);
	depth *= depth_correction;
	point = unproj * vec4(coord.x * depth, coord.y * depth, depth, 1);
	//point = vec4(coord.x/100.0, coord.y/100.0, 1, 1);
	tex_coord = coord / vec2(width, height);
}

void main() {

	for (int i = 0; i < gl_in.length(); i++) {
	
		vec4 p;
		vec2 t;
		bool v;
	
		get_point(texture_coordinates_int[i], p, t, v);
		
		if(v) {
			gl_Position = mvp * p;
			texture_coordinates = t;
			EmitVertex();
			EndPrimitive();
		}


	}
}
