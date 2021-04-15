#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;

  float k_a = 2.0;
  float k_d = 0.1;
  float k_s = 2.0;
  vec3 I_a = vec3(0.3);
  float p = 20.0;

  vec3 r_vector = u_light_pos - v_position.xyz;
  float r_magnitude = length(r_vector);

  vec3 v_vector = u_cam_pos - v_position.xyz;
  vec3 h_vector = (v_vector + r_vector) / length(v_vector + r_vector);
  
  vec3 ambient = k_a * I_a;
  vec3 diffuse = k_d * u_light_intensity / (r_magnitude * r_magnitude) * max(0.0, dot(v_normal.xyz, r_vector));
  vec3 specular = k_s * u_light_intensity / (r_magnitude * r_magnitude) * pow(max(0.0, dot(v_normal.xyz, h_vector)), p);

  //out_color.xyz = ambient + diffuse + specular;
  out_color.xyz = ambient;

 
  out_color.a = 1;
}

