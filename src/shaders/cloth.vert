R"zzz(
#version 330 core
uniform vec4 light_position;
uniform vec3 camera_position;
uniform vec4 block_delta;

uniform vec4 vertex_positions[512];

in int vertex;
in vec4 normal;
in vec2 uv_coords;
in vec4 vertex_position;

out vec4 vs_light_direction;
out vec4 vs_normal;
out vec2 vs_uv;
out vec4 vs_camera_direction;

void main() {
  gl_Position = vertex_positions[vertex];
  vs_light_direction = light_position - gl_Position;
  vs_camera_direction = vec4(camera_position, 1.0) - gl_Position;
  vs_normal = normal;
	vs_uv = uv_coords;
}
)zzz"
