R"zzz(
#version 330 core
in vec4 face_normal;
in vec4 vertex_normal;
in vec4 light_direction;
in vec4 camera_direction;
in vec4 world_position;
in vec2 uv_coords;
in float height;
in float grass_height;
uniform vec4 diffuse;
//uniform vec4 ambient;
//uniform vec4 specular;
//uniform float shininess;
//uniform float alpha;
uniform sampler2D textureSampler;
uniform float time;
out vec4 fragment_color;


void main() {
		vec3 color = vec3(uv_coords[0], uv_coords[1], 0.12);
		//vec3 color = vec3(diffuse);
		//vec3 color = vec3(diffuse);
		//vec2 randuv = vec2(rand(light_direction.xy), rand(light_direction.zw));
		//vec3 color = vec3(diffuse) + texture(textureSampler, randuv).xyz;
		//vec3 color = texture(textureSampler, randuv).xyz;
		//vec3 color = vec3(diffuse) + vec3(randuv.x, randuv.y, 1.0);
    vec4 ambient = vec4(0.2, 0.2, 0.2, 1.0);
    vec4 specular = vec4(0.2, 0.2, 0.2, 1.0);
    float shininess = 0.0;
    float alpha = 1.0;
		float dot_nl = dot(normalize(light_direction), normalize(vertex_normal));
		dot_nl = clamp(dot_nl, 0.0, 1.0);
		vec4 spec = specular * pow(max(0.0, dot(reflect(-light_direction, vertex_normal), camera_direction)), shininess);
		color = clamp(dot_nl * color + vec3(ambient) + vec3(spec), 0.0, 1.0);
		fragment_color = vec4(color, alpha);
		//fragment_color = vec4(0,1,0,1);
}
)zzz"
