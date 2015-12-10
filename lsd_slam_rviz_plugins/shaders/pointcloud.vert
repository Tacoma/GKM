#version 330

// takes vertexID and returns x=[0:step:width] and y=[0:step:height]

in int gl_VertexID;

uniform int width;
uniform int height;
uniform int step = 1;

out ivec2 texture_coordinates_int;

void main () {
  texture_coordinates_int = step * ivec2(gl_VertexID % (width/step), gl_VertexID / (width/step));
  gl_Position = vec4(0,0,0,1);
}
