shader_type spatial;

uniform float height = 128.0;

uniform vec4 color_0 : source_color = vec4(0.76, 0.70, 0.50, 1.0);
uniform vec4 color_1 : source_color = vec4(0.25, 0.55, 0.20, 1.0);
uniform vec4 color_2 : source_color = vec4(0.50, 0.45, 0.40, 1.0);

uniform vec4 slope_color_low  : source_color = vec4(0.40, 0.32, 0.22, 1.0);
uniform vec4 slope_color_high : source_color = vec4(0.35, 0.30, 0.28, 1.0);

uniform float threshold_1     : hint_range(0.0, 1.0) = 0.3;
uniform float threshold_2     : hint_range(0.0, 1.0) = 0.6;
uniform float blend_sharpness : hint_range(0.01, 0.5) = 0.08;

uniform float slope_color_split : hint_range(0.0, 1.0) = 0.5;
uniform float slope_color_blend : hint_range(0.01, 0.3) = 0.1;
uniform float slope_threshold   : hint_range(0.0, 1.0) = 0.72;
uniform float slope_blend       : hint_range(0.01, 0.4) = 0.12;

varying float world_y;
varying vec3 world_normal;

void vertex() {
	world_y = (MODEL_MATRIX * vec4(VERTEX, 1.0)).y;
	// NORMAL in vertex() is object space — transform it to world space
	// using the normal matrix (transpose of inverse of MODEL_MATRIX upper 3x3)
	world_normal = normalize((MODEL_MATRIX * vec4(NORMAL, 0.0)).xyz);
}

void fragment() {
	float h = clamp((world_y / height) + 0.5, 0.0, 1.0);

	// Height bands
	float w0 = smoothstep(threshold_1, threshold_1 - blend_sharpness, h);
	float w1 = smoothstep(threshold_1 - blend_sharpness, threshold_1, h)
			 * smoothstep(threshold_2, threshold_2 - blend_sharpness, h);
	float w2 = smoothstep(threshold_2 - blend_sharpness, threshold_2, h);

	vec3 flat_color = color_0.rgb * w0 + color_1.rgb * w1 + color_2.rgb * w2;

	// Slope color split by height
	float slope_h_blend = smoothstep(
		slope_color_split - slope_color_blend,
		slope_color_split + slope_color_blend,
		h
	);
	vec3 steep_color = mix(slope_color_low.rgb, slope_color_high.rgb, slope_h_blend);

	// Slope detection using world normal — stable regardless of camera
	float flatness     = dot(world_normal, vec3(0.0, 1.0, 0.0));
	float slope_factor = 1.0 - smoothstep(
		slope_threshold - slope_blend,
		slope_threshold + slope_blend,
		flatness
	);

	ALBEDO = mix(flat_color, steep_color, slope_factor);
}