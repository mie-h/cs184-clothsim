#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  //return 0.0;
  return texture(u_texture_3, uv).r;
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  
  vec3 t = v_tangent.xyz;
  vec3 n = v_normal.xyz;
  vec3 b = cross(n, t);
  mat3 TBN = mat3(t, b, n);

  float u = v_uv.x;
  float v = v_uv.y;
  float w = u_texture_3_size.x;
  float h_sub = u_texture_3_size.y;

  float dU = (h(vec2(u+1/w, v)) - h(vec2(u,v))) * u_height_scaling * u_normal_scaling;
  float dV = (h(vec2(u, v+1/h_sub)) - h(vec2(u,v))) * u_height_scaling * u_normal_scaling;
  vec3 n0 = vec3(-dU, -dV, 1);
  vec3 nd = TBN * n0;

  nd = normalize(nd);


  float k_a = 1.0;
  float k_d = 0.1;
  float k_s = 2.0;
  vec3 I_a = vec3(0.3);
  float p = 20.0;

  vec3 r_vector = u_light_pos - v_position.xyz;
  float r_magnitude = length(r_vector);

  vec3 v_vector = u_cam_pos - v_position.xyz;
  vec3 h_vector = (v_vector + r_vector) / length(v_vector + r_vector);
  
  out_color.xyz = k_a * I_a + k_d * u_light_intensity / (r_magnitude * r_magnitude) * max(0.0, dot(nd, r_vector)) + k_s * u_light_intensity / (r_magnitude * r_magnitude) * pow(max(0.0, dot(nd, h_vector)), p);

  
  out_color.a = 1;
}

