#version 330

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in ivec2 texture_coordinates_int[];
out vec2 texture_coordinates;

uniform sampler2D depth_texture;
uniform mat4 mvp;
uniform mat4 unproj;

uniform int width;
uniform int height;
uniform int step = 1;
uniform float threshold2 = 0.2*0.2;
uniform float depth_correction = 65535.0f / 1000.0f; // Comes from depth stored in uint_16 texture in milimiters

void get_point(in ivec2 coord, out vec4 point, out vec2 tex_coord, out bool valid) {
	float depth = texelFetch(depth_texture, coord, 0).r;
	valid = (depth > 0);
	depth *= depth_correction;
	point = unproj * vec4(coord.x * depth, coord.y * depth, depth, 1);
	tex_coord = coord / vec2(width, height);
}

void emit(in vec4 p[4], in vec2 t[4], in int i0, in int i1, in int i2, in int i3) {
	gl_Position = mvp * p[i0];
	texture_coordinates = t[i0];
	EmitVertex();

	gl_Position = mvp * p[i1];
	texture_coordinates = t[i1];
	EmitVertex();

	gl_Position = mvp * p[i2];
	texture_coordinates = t[i2];
	EmitVertex();

	gl_Position = mvp * p[i3];
	texture_coordinates = t[i3];
	EmitVertex();
	EndPrimitive();

}

void emit(in vec4 p[4], in vec2 t[4], in int i0, in int i1, in int i2) {
	gl_Position = mvp * p[i0];
	texture_coordinates = t[i0];
	EmitVertex();

	gl_Position = mvp * p[i1];
	texture_coordinates = t[i1];
	EmitVertex();

	gl_Position = mvp * p[i2];
	texture_coordinates = t[i2];
	EmitVertex();
	EndPrimitive();
}

void main() {

	for (int i = 0; i < gl_in.length(); i++) {
		// p0 --- p1
		// |      |
		// p2 --- p3
		vec4 p[4];
		vec2 t[4];
		bool v[4];

		get_point(texture_coordinates_int[i], p[0], t[0], v[0]);
		get_point(texture_coordinates_int[i] + ivec2(step, 0), p[1], t[1], v[1]);
		get_point(texture_coordinates_int[i] + ivec2(0, step), p[2], t[2], v[2]);
		get_point(texture_coordinates_int[i] + ivec2(step, step), p[3], t[3], v[3]);

		vec4 tmp;
		float p0p1, p1p3, p3p2, p2p0, p0p3, p1p2;

		tmp = p[0] - p[1];
		p0p1 = dot(tmp, tmp);

		tmp = p[1] - p[3];
		p1p3 = dot(tmp, tmp);

		tmp = p[3] - p[2];
		p3p2 = dot(tmp, tmp);

		tmp = p[2] - p[0];
		p2p0 = dot(tmp, tmp);

		tmp = p[0] - p[3];
		p0p3 = dot(tmp, tmp);

		tmp = p[1] - p[2];
		p1p2 = dot(tmp, tmp);

		if (v[0] && v[1] && v[2] && v[3]) {

			if (p0p3 <= p1p2 && p0p3 < threshold2) {
				bool up_triangle = false;
				bool down_triangle = false;

				if (p1p3 < threshold2 && p0p1 < threshold2)
					up_triangle = true;
				if (p2p0 < threshold2 && p3p2 < threshold2)
					down_triangle = true;

				if (up_triangle && down_triangle) {
					emit(p,t,1,0,3,2);
				} else if (up_triangle) {
					emit(p,t,0,3,1);
				} else if (down_triangle) {
					emit(p,t,0,2,3);
				}

			} else if (p1p2 <= p0p3 && p1p2 < threshold2) {
				bool up_triangle = false;
				bool down_triangle = false;

				if (p0p1 < threshold2 && p2p0 < threshold2)
					up_triangle = true;
				if (p1p3 < threshold2 && p3p2 < threshold2)
					down_triangle = true;

				if (up_triangle && down_triangle) {
					emit(p,t,0,2,1,3);
				} else if (up_triangle) {
					emit(p,t,0,2,1);
				} else if (down_triangle) {
					emit(p,t,1,2,3);
				}

			}

		} else if (v[0] && v[1] && v[2]) {
			if (p0p1 < threshold2 && p1p2 < threshold2 && p2p0 < threshold2) emit(p,t,0,2,1);
		} else if (v[0] && v[1] && v[3]) {
			if (p0p1 < threshold2 && p1p3 < threshold2 && p0p3 < threshold2) emit(p,t,0,3,1);
		} else if (v[0] && v[2] && v[3]) {
			if (p0p3 < threshold2 && p3p2 < threshold2 && p2p0 < threshold2) emit(p,t,0,2,3);
		} else if (v[1] && v[2] && v[3]) {
			if (p1p3 < threshold2 && p3p2 < threshold2 && p1p2 < threshold2) emit(p,t,1,2,3);
		}
	}
}