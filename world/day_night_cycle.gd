extends Node3D
class_name DayNightCycle

# 0 = midnight, 0.25 = sunrise, 0.5 = noon, 0.75 = sunset
@export_range(0.0, 1.0, 0.001) var time_of_day: float = 0.25
@export_range(10.0, 1200.0, 1.0) var day_duration: float = 120.0
@export var paused: bool = false

@export_group("References")
@export var sun: DirectionalLight3D
@export var environment: WorldEnvironment

@export_group("Sun")
@export_range(0.0, 8.0, 0.1) var max_sun_energy: float = 2.0
@export var day_color: Color    = Color(1.00, 0.98, 0.90)
@export var sunrise_color: Color = Color(1.00, 0.50, 0.15)

@export_group("Ambient")
@export var day_ambient: Color   = Color(0.30, 0.40, 0.50)
@export var night_ambient: Color = Color(0.01, 0.01, 0.04)

var _sky: ProceduralSkyMaterial = null


func _ready() -> void:
	if environment and environment.environment and environment.environment.sky:
		_sky = environment.environment.sky.sky_material as ProceduralSkyMaterial
	_apply_time()


func _process(delta: float) -> void:
	if not paused:
		time_of_day = fmod(time_of_day + delta / day_duration, 1.0)
	_apply_time()


# Returns -1 (midnight) → 0 (horizon) → 1 (noon).
func get_sun_height() -> float:
	return -cos(time_of_day * TAU)


func is_daytime() -> bool:
	return get_sun_height() > 0.0


func _apply_time() -> void:
	var sun_height := get_sun_height()
	_update_sun(sun_height)
	_update_environment(sun_height)


func _update_sun(sun_height: float) -> void:
	if not sun:
		return
	# X rotation: 90=below ground (midnight), 0=horizon, -90=overhead (noon)
	sun.rotation_degrees.x = 90.0 * cos(time_of_day * TAU)
	# Slight east-west arc so the sun doesn't just bob straight up and down
	sun.rotation_degrees.y = sin(time_of_day * TAU) * 30.0

	# Fade in from just below the horizon, full brightness by mid-morning
	var energy_t := smoothstep(-0.15, 0.35, sun_height)
	sun.light_energy = max_sun_energy * energy_t

	# Warm orange near horizon, white-yellow at noon
	var horizon_t := smoothstep(0.0, 0.6, 1.0 - abs(sun_height)) * energy_t
	sun.light_color = day_color.lerp(sunrise_color, horizon_t)


func _update_environment(sun_height: float) -> void:
	if not environment or not environment.environment:
		return
	var env  := environment.environment
	var day_t := smoothstep(-0.2, 0.3, sun_height)
	env.ambient_light_color  = night_ambient.lerp(day_ambient, day_t)
	env.ambient_light_energy = 1.0

	if _sky == null:
		return
	var dusk_t := smoothstep(0.0, 0.3, sun_height)
	_sky.sky_top_color        = night_ambient.lerp(Color(0.18, 0.46, 0.85), dusk_t)
	_sky.sky_horizon_color    = sunrise_color.lerp(Color(0.62, 0.82, 1.00), dusk_t)
	_sky.ground_bottom_color  = Color(0.05, 0.04, 0.03)
	_sky.ground_horizon_color = Color(0.15, 0.12, 0.08).lerp(Color(0.45, 0.37, 0.27), dusk_t)
