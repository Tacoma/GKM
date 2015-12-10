#version 330

in vec2 texture_coordinates;

uniform sampler2D rgb_texture;

out vec4 frag_color;

void main () {
  frag_color = texture2D(rgb_texture, texture_coordinates);
}