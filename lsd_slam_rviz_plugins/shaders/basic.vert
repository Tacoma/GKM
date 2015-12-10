#version 330

uniform mat4 mvp;

in int gl_VertexID;

out vec2 texture_coordinates;


const vec2 quad_pos[6] = vec2[](
  vec2(-1.0, -1.0),
  vec2(1.0, -1.0),
  vec2(1.0,  1.0),
  vec2(1.0,  1.0),
  vec2(-1.0,  1.0),
  vec2(-1.0, -1.0)
);



const vec2 quad_st[6] = vec2[](
  vec2(0.0, 1.0),
  vec2(1.0, 1.0),
  vec2(1.0, 0.0),
  vec2(1.0, 0.0),
  vec2(0.0, 0.0),
  vec2(0.0, 1.0)
);


void main () {
  texture_coordinates = quad_st[gl_VertexID];
  gl_Position = mvp * vec4(quad_pos[gl_VertexID],1,1);
}
