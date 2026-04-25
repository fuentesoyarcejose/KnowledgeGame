shader_type spatial;
render_mode blend_mix, depth_draw_opaque, cull_disabled, diffuse_burley, specular_schlick_ggx;

// ── Flow ───────────────────────────────────────────────────────────────────
uniform float flow_speed      : hint_range(0.0, 2.0)  = 0.4;
uniform vec2  flow_direction                           = vec2(1.0, 0.0);
uniform float tiling          : hint_range(0.1, 16.0) = 4.0;


// ── Normal maps (use two scrolling at different speeds for complexity) ─────
uniform sampler2D normal_map_a : hint_normal, filter_linear_mipmap, repeat_enable;
uniform sampler2D normal_map_b : hint_normal, filter_linear_mipmap, repeat_enable;
uniform float normal_strength  : hint_range(0.0, 2.0) = 0.6;
uniform sampler2D DEPTH_TEXTURE : hint_depth_texture, filter_linear_mipmap;

// ── Color ──────────────────────────────────────────────────────────────────
uniform vec4 color_shallow : source_color = vec4(0.20, 0.55, 0.60, 0.6);
uniform vec4 color_deep    : source_color = vec4(0.04, 0.18, 0.35, 0.9);
uniform float depth_scale  : hint_range(0.0, 10.0) = 3.0;

// ── Surface ────────────────────────────────────────────────────────────────
uniform float roughness    : hint_range(0.0, 1.0) = 0.05;
uniform float metallic     : hint_range(0.0, 1.0) = 0.0;
uniform float fresnel_power: hint_range(1.0, 8.0) = 4.0;

// ── Foam ───────────────────────────────────────────────────────────────────
uniform sampler2D foam_texture : source_color, filter_linear_mipmap, repeat_enable;
uniform float foam_threshold : hint_range(0.0, 1.0) = 0.3;
uniform float foam_tiling    : hint_range(0.1, 16.0) = 6.0;
uniform vec4  foam_color     : source_color = vec4(0.9, 0.95, 1.0, 1.0);

varying vec3 world_pos;
varying vec3 world_normal;

void vertex() {
	world_pos    = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	world_normal = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
}

void fragment() {
	vec2 uv = world_pos.xz / tiling;
	float t  = TIME * flow_speed;

	// Two normal map layers scrolling in slightly different directions
	vec2 dir_a = normalize(flow_direction);
	vec2 dir_b = normalize(flow_direction + vec2(-0.3, 0.2)); // slight offset angle

	vec2 uv_a = uv + dir_a * t;
	vec2 uv_b = uv * 1.4 + dir_b * t * 0.7; // different tiling + slower

	vec3 n_a = texture(normal_map_a, uv_a).rgb;
	vec3 n_b = texture(normal_map_b, uv_b).rgb;

	// Blend the two normal maps
	vec3 normal_blend = normalize(n_a + n_b - vec3(1.0));
	// Pull toward flat based on strength
	normal_blend = mix(vec3(0.5, 0.5, 1.0), normal_blend, normal_strength);

	// ── Depth-based color ─────────────────────────────────────────────────
	// Use screen-space UV to sample depth buffer for depth fade
	float depth_raw    = texture(DEPTH_TEXTURE, SCREEN_UV).r;
	vec4  depth_ndc    = vec4(SCREEN_UV * 2.0 - 1.0, depth_raw, 1.0);
	vec4  depth_view   = INV_PROJECTION_MATRIX * depth_ndc;
	depth_view        /= depth_view.w;
	float scene_depth  = -depth_view.z;
	float frag_depth   = -VERTEX.z; // VERTEX is view space in fragment()
	float water_depth  = clamp((scene_depth - frag_depth) * depth_scale, 0.0, 1.0);

	vec4 water_color = mix(color_shallow, color_deep, water_depth);

	// ── Fresnel ───────────────────────────────────────────────────────────
	vec3 view_dir  = normalize(VIEW);
	float fresnel  = pow(1.0 - clamp(dot(world_normal, view_dir), 0.0, 1.0), fresnel_power);

	// ── Foam at shallow edges ─────────────────────────────────────────────
	vec2  foam_uv    = world_pos.xz / foam_tiling + dir_a * t * 0.5;
	float foam_noise = texture(foam_texture, foam_uv).r;
	// Foam appears where water is shallow
	float foam_mask  = step(foam_threshold, foam_noise * (1.0 - water_depth));
	vec3  albedo     = mix(water_color.rgb, foam_color.rgb, foam_mask);

	// ── Output ────────────────────────────────────────────────────────────
	ALBEDO       = albedo;
	NORMAL_MAP   = normal_blend;
	ROUGHNESS    = mix(roughness, 0.4, foam_mask); // foam is less shiny
	METALLIC     = metallic;
	ALPHA        = mix(water_color.a, 1.0, fresnel + foam_mask);
	SPECULAR     = 0.9;
}