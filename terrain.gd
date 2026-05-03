@tool
extends MeshInstance3D
class_name TerrainMesh

@export var terrain_shader: Shader = preload("res://terrainShader.gdshader")
var _terrain_material: ShaderMaterial = null
const EDITOR_REBUILD_DEBOUNCE_SEC: float = 0.2
var _editor_rebuild_timer: Timer = null

# ── Terrain ──────────────────────────────────────────────────────────────────
@export var terrain_size := 1000.0
@export_range(4, 512, 4) var resolution := 32:
	set(v): resolution = v; _queue_update_mesh()
@export_range(4, 128, 4) var editor_preview_resolution: int = 16

var _generate_map := false
@export var generate_map: bool:
	get: return _generate_map
	set(v):
		_generate_map = v
		if v: _queue_update_mesh(); _generate_map = false

var _generate_rivers_only := false
@export var generate_rivers_only: bool:
	get: return _generate_rivers_only
	set(v):
		_generate_rivers_only = v
		if v:
			_river_paths.clear(); _river_points.clear()
			update_mesh(false); _generate_rivers_only = false

var _generate_height_only := false
@export var generate_height_only: bool:
	get: return _generate_height_only
	set(v):
		_generate_height_only = v
		if v: update_mesh(false); _generate_height_only = false

@export var freeze_river_layout_for_testing: bool = false

@export var noise: FastNoiseLite:
	set(new_noise):
		noise = new_noise
		if noise:
			noise.seed = _seed
			noise.frequency = _frequency
			noise.fractal_lacunarity = _lacunarity
			noise.fractal_gain = _gain
			if not noise.changed.is_connected(_queue_update_mesh):
				noise.changed.connect(_queue_update_mesh)
		_queue_update_mesh()

@export_range(4.0, 128.0, 4.0) var height := 64.0:
	set(v):
		height = v
		if _terrain_material:
			_terrain_material.set_shader_parameter("height", height * 2.0)
		_queue_update_mesh()

@export_range(-256.0, 256.0, 1.0) var ground_level: float = 0.0

var _frequency := 0.02
@export_range(0.001, 0.5, 0.001) var frequency: float:
	get: return _frequency
	set(v): _frequency = v; if noise: noise.frequency = v; _queue_update_mesh()

var _lacunarity := 2.0
@export_range(1.0, 4.0, 0.01) var lacunarity: float:
	get: return _lacunarity
	set(v): _lacunarity = v; if noise: noise.fractal_lacunarity = v; _queue_update_mesh()

var _gain := 0.5
@export_range(0.0, 1.0, 0.01) var gain: float:
	get: return _gain
	set(v): _gain = v; if noise: noise.fractal_gain = v; _queue_update_mesh()

# ── Forest ────────────────────────────────────────────────────────────────────
@export var forest_noise: FastNoiseLite:
	set(v): forest_noise = v; if forest_noise and not forest_noise.changed.is_connected(_queue_update_mesh): forest_noise.changed.connect(_queue_update_mesh); _queue_update_mesh()
@export_range(-1.0, 1.0, 0.01) var forest_threshold: float = 0.0:
	set(v): forest_threshold = v; _queue_update_mesh()

# ── Rivers ────────────────────────────────────────────────────────────────────
@export var river_depth: float = 10.0
@export_range(1, 8, 1) var river_count: int = 2
@export var river_width: float = 20.0
@export_range(0.0, 1.0, 0.01) var river_width_variation: float = 0.4
@export_range(0.1, 8.0, 0.1) var river_width_noise_frequency: float = 2.0
@export var river_min_width: float = 6.0
@export var river_ground_band: float = 5.0
@export var river_terrain_influence: float = 3.0
@export var water_level_offset: float = 1.0
@export_range(0, 8, 1) var river_curve_iterations: int = 5
@export var river_curve_strength: float = 8.0
@export_range(0.1, 8.0, 0.1) var river_curve_noise_frequency: float = 1.5

# ── Shore ─────────────────────────────────────────────────────────────────────
@export_range(0.1, 10.0, 0.1) var shore_width: float = 1.0

# ── Debug ──────────────────────────────────────────────────────────────────────
@export var show_biome_debug: bool = false:
	set(v): show_biome_debug = v; _queue_update_mesh()

# ── Seed ──────────────────────────────────────────────────────────────────────
var _seed: int = 0
@export var seed: int:
	get: return _seed
	set(v):
		_seed = v
		if noise: noise.seed = v
		_queue_update_mesh()

# ── Internal state ────────────────────────────────────────────────────────────
var _river_paths: Array[PackedVector3Array] = []
var _river_points: Array[Array] = []

# Carve cache: pre-baked blend factors (0 = river floor, 1 = full terrain).
# Built once per river layout at a fixed resolution; bilinearly sampled per vertex.
const _CARVE_RES: int = 128
var _carve_cache: PackedFloat32Array = PackedFloat32Array()
var _forest_modifier: PackedFloat32Array = PackedFloat32Array()
var _carve_step: float = 0.0
var _carve_half: float = 0.0
var _biome_texture: ImageTexture = null

enum Biome { WATER, SHORE, FOREST, PLAINS, ROCKY, CLIFF }


func _ready() -> void:
	if Engine.is_editor_hint():
		if _editor_rebuild_timer == null:
			_editor_rebuild_timer = Timer.new()
			_editor_rebuild_timer.one_shot = true
			_editor_rebuild_timer.wait_time = EDITOR_REBUILD_DEBOUNCE_SEC
			add_child(_editor_rebuild_timer)
		if not _editor_rebuild_timer.timeout.is_connected(_on_editor_rebuild_timeout):
			_editor_rebuild_timer.timeout.connect(_on_editor_rebuild_timeout)


func _on_editor_rebuild_timeout() -> void:
	update_mesh()


func _queue_update_mesh() -> void:
	if not Engine.is_editor_hint():
		update_mesh()
		return
	if _editor_rebuild_timer == null:
		_ready()
	if _editor_rebuild_timer:
		_editor_rebuild_timer.start()


func _get_active_resolution() -> int:
	if Engine.is_editor_hint():
		return max(4, editor_preview_resolution)
	return max(4, resolution)


func _ensure_terrain_material() -> void:
	if not terrain_shader:
		push_warning("TerrainMesh: terrain_shader is not set.")
		return
	if _terrain_material == null:
		_terrain_material = ShaderMaterial.new()
	if _terrain_material.shader != terrain_shader:
		_terrain_material.shader = terrain_shader
	_terrain_material.set_shader_parameter("height", height * 2.0)
	_terrain_material.set_shader_parameter("ground_level", ground_level)
	_terrain_material.set_shader_parameter("shore_width", shore_width)
	_terrain_material.set_shader_parameter("terrain_size", terrain_size)


# Single-sample height — fast path used by normal computation and edge sampling.
func get_height(x: float, z: float) -> float:
	if noise == null:
		return ground_level
	return maxf(ground_level, noise.get_noise_2d(x, z) * height)


# Central-difference normal using 4 direct noise samples (5× faster than the
# 5-sample weighted version previously used through get_height calls).
func get_normal(x: float, z: float) -> Vector3:
	if noise == null:
		return Vector3.UP
	var e := terrain_size / float(_get_active_resolution())
	var hL := maxf(ground_level, noise.get_noise_2d(x - e, z) * height)
	var hR := maxf(ground_level, noise.get_noise_2d(x + e, z) * height)
	var hB := maxf(ground_level, noise.get_noise_2d(x, z - e) * height)
	var hF := maxf(ground_level, noise.get_noise_2d(x, z + e) * height)
	return Vector3(hL - hR, 2.0 * e, hB - hF).normalized()


func _river_half_width_at_u(u: float, river_idx: int = 0) -> float:
	var base_half := maxf(river_min_width * 0.5, river_width * 0.5)
	if river_width_variation <= 0.0:
		return base_half
	var n := 0.0
	if noise:
		n = noise.get_noise_1d(u * river_width_noise_frequency + float(_seed + river_idx * 131) * 0.013)
	return maxf(river_min_width * 0.5, base_half * (1.0 + n * river_width_variation))


# Forward-mapping cache: iterate each full path segment and stamp blend values
# into only the cells within that segment's influence radius. Uses the complete
# Chaikin path (no decimation) so the carved valley matches the water mesh exactly.
func _build_carve_cache() -> void:
	var cols := _CARVE_RES + 1
	_carve_step = terrain_size / float(_CARVE_RES)
	_carve_half = terrain_size * 0.5
	_carve_cache.resize(cols * cols)
	_carve_cache.fill(1.0)

	for river_idx in _river_paths.size():
		var path := _river_paths[river_idx]
		if path.size() < 2:
			continue
		var denom := float(max(1, path.size() - 1))
		for i in path.size() - 1:
			var u      := float(i) / denom
			var half_w := _river_half_width_at_u(u, river_idx)
			var bank   := maxf(river_ground_band, half_w * river_terrain_influence)
			var total  := half_w + bank

			var a      := Vector2(path[i].x, path[i].z)
			var b      := Vector2(path[i + 1].x, path[i + 1].z)
			var ab     := b - a
			var len_sq := ab.dot(ab)

			var min_gx := int(clamp((min(a.x, b.x) - total + _carve_half) / _carve_step, 0.0, float(cols - 1)))
			var max_gx := int(clamp((max(a.x, b.x) + total + _carve_half) / _carve_step, 0.0, float(cols - 1)))
			var min_gz := int(clamp((min(a.y, b.y) - total + _carve_half) / _carve_step, 0.0, float(cols - 1)))
			var max_gz := int(clamp((max(a.y, b.y) + total + _carve_half) / _carve_step, 0.0, float(cols - 1)))

			for gz in range(min_gz, max_gz + 1):
				for gx in range(min_gx, max_gx + 1):
					var xz    := Vector2(-_carve_half + gx * _carve_step, -_carve_half + gz * _carve_step)
					var seg_t := 0.0
					if len_sq >= 0.0001:
						seg_t = clamp((xz - a).dot(ab) / len_sq, 0.0, 1.0)
					var d := xz.distance_to(a + ab * seg_t)
					if d >= total:
						continue
					var new_blend: float
					if d < half_w:
						new_blend = 0.0
					else:
						var blend_t := (d - half_w) / bank
						new_blend = blend_t * blend_t * (3.0 - 2.0 * blend_t)
					var idx := gz * cols + gx
					_carve_cache[idx] = minf(_carve_cache[idx], new_blend)


func get_biome(x: float, z: float) -> Biome:
	var h           := get_height(x, z)
	var slope       := 1.0 - get_normal(x, z).y
	var river_blend := _sample_carve_blend(Vector2(x, z))
	if river_blend < 0.05:               return Biome.WATER
	if river_blend < 0.2:                return Biome.SHORE
	if slope > 0.1:                      return Biome.CLIFF
	if h > ground_level + height * 0.10: return Biome.ROCKY
	if forest_noise and (forest_noise.get_noise_2d(x, z) + _sample_forest_modifier(x, z)) > forest_threshold:
		return Biome.FOREST
	return Biome.PLAINS


func _sample_carve_blend(xz: Vector2) -> float:
	if _carve_cache.is_empty():
		return 1.0
	var cols := _CARVE_RES + 1
	var gx: float = clamp((xz.x + _carve_half) / _carve_step, 0.0, float(cols - 1))
	var gz: float = clamp((xz.y + _carve_half) / _carve_step, 0.0, float(cols - 1))
	var x0 := int(gx)
	var x1 := mini(x0 + 1, cols - 1)
	var z0 := int(gz)
	var z1 := mini(z0 + 1, cols - 1)
	var tx := gx - float(x0)
	var tz := gz - float(z0)
	var v00 := _carve_cache[z0 * cols + x0]
	var v10 := _carve_cache[z0 * cols + x1]
	var v01 := _carve_cache[z1 * cols + x0]
	var v11 := _carve_cache[z1 * cols + x1]
	return lerpf(lerpf(v00, v10, tx), lerpf(v01, v11, tx), tz)


func _draw_road(paths: Array[PackedVector3Array]) -> void:
	var road := _get_or_create_marker("Road") as MeshInstance3D
	if paths.is_empty():
		road.mesh = null
		return
	var lifted := PackedVector3Array()
	for path in paths:
		for i in path.size() - 1:
			lifted.append(Vector3(path[i].x,     ground_level - river_depth, path[i].z))
			lifted.append(Vector3(path[i + 1].x, ground_level - river_depth, path[i + 1].z))
	if lifted.is_empty():
		road.mesh = null
		return
	var arrays := []; arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = lifted
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(1, 1, 0)
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	arr_mesh.surface_set_material(0, mat)
	road.mesh = arr_mesh


func _smooth_path_chaikin(path: PackedVector3Array, iterations: int = 4) -> PackedVector3Array:
	var result := path
	for _iter in iterations:
		var s := PackedVector3Array()
		s.append(result[0])
		for i in result.size() - 1:
			var a := result[i]
			var b := result[i + 1]
			s.append(Vector3(a.x * 0.75 + b.x * 0.25, a.y, a.z * 0.75 + b.z * 0.25))
			s.append(Vector3(a.x * 0.25 + b.x * 0.75, a.y, a.z * 0.25 + b.z * 0.75))
		s.append(result[result.size() - 1])
		result = s
	return result


# ── Water surface ────────────────────────────────────────────────────────────

func _draw_water_surface(paths: Array[PackedVector3Array]) -> void:
	var water := _get_or_create_marker("Water") as MeshInstance3D
	var water_y := ground_level - water_level_offset

	var verts   := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs     := PackedVector2Array()
	var indices := PackedInt32Array()

	for river_idx in paths.size():
		var path := paths[river_idx]
		var n := path.size()
		if n < 2:
			continue
		var cs_left  := PackedVector2Array()
		var cs_right := PackedVector2Array()
		var cs_u     := PackedFloat32Array()

		for i in n:
			var fwd := Vector3.ZERO
			if i < n - 1:
				fwd += Vector3(path[i+1].x - path[i].x, 0.0, path[i+1].z - path[i].z)
			if i > 0:
				fwd += Vector3(path[i].x - path[i-1].x, 0.0, path[i].z - path[i-1].z)
			if fwd.length_squared() < 0.0001:
				fwd = Vector3.FORWARD
			fwd = fwd.normalized()
			var right := Vector3(-fwd.z, 0.0, fwd.x)
			var u := float(i) / float(max(1, n - 1))
			var hw := _river_half_width_at_u(u, river_idx)
			var pl := Vector2(path[i].x - right.x * hw, path[i].z - right.z * hw)
			var pr := Vector2(path[i].x + right.x * hw, path[i].z + right.z * hw)
			cs_left.append(pl)
			cs_right.append(pr)
			cs_u.append(u)

		for i in n - 1:
			var base := verts.size()
			verts.append(Vector3(cs_left[i].x,    water_y, cs_left[i].y))
			verts.append(Vector3(cs_right[i].x,   water_y, cs_right[i].y))
			verts.append(Vector3(cs_left[i+1].x,  water_y, cs_left[i+1].y))
			verts.append(Vector3(cs_right[i+1].x, water_y, cs_right[i+1].y))
			normals.append(Vector3.UP); normals.append(Vector3.UP)
			normals.append(Vector3.UP); normals.append(Vector3.UP)
			uvs.append(Vector2(0.0, cs_u[i]));   uvs.append(Vector2(1.0, cs_u[i]))
			uvs.append(Vector2(0.0, cs_u[i+1])); uvs.append(Vector2(1.0, cs_u[i+1]))
			indices.append(base);     indices.append(base + 2); indices.append(base + 1)
			indices.append(base + 1); indices.append(base + 2); indices.append(base + 3)

	if verts.is_empty():
		water.mesh = null
		return

	var arrays := []; arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX]  = verts
	arrays[ArrayMesh.ARRAY_NORMAL]  = normals
	arrays[ArrayMesh.ARRAY_TEX_UV]  = uvs
	arrays[ArrayMesh.ARRAY_INDEX]   = indices

	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	var mat := ShaderMaterial.new()
	mat.shader = preload("res://RiverWater.gdshader")
	mat.set_shader_parameter("flow_speed",      0.35)
	mat.set_shader_parameter("flow_direction",  Vector2(1.0, 0.2))
	mat.set_shader_parameter("tiling",          5.0)
	mat.set_shader_parameter("depth_scale",     4.0)
	mat.set_shader_parameter("normal_strength", 0.65)
	arr_mesh.surface_set_material(0, mat)
	water.mesh = arr_mesh


# ── Edge / river path helpers ─────────────────────────────────────────────────

func _sample_edge_point(side: int, rng: RandomNumberGenerator) -> Vector3:
	var half := terrain_size * 0.5
	var x := 0.0
	var z := 0.0
	for _i in 1000:
		var t := rng.randf() * terrain_size - half
		match side:
			0: x = -half; z = t
			1: x =  half; z = t
			2: x = t; z = -half
			3: x = t; z =  half
		if not noise or get_height(x, z) == ground_level:
			break
	return Vector3(x, ground_level, z)


func get_random_edge_points(rng: RandomNumberGenerator, local_space: bool = true) -> Array[Vector3]:
	var first_side := rng.randi() % 4
	var second_side := (first_side + 1 + rng.randi() % 3) % 4
	var a := _sample_edge_point(first_side, rng)
	var b := _sample_edge_point(second_side, rng)
	if local_space: return [a, b]
	return [to_global(a), to_global(b)]


func _get_or_create_marker(marker_name: String) -> Node3D:
	if has_node(marker_name):
		return get_node(marker_name)
	var marker := MeshInstance3D.new()
	marker.name = marker_name
	add_child(marker)
	return marker


func _momentum_walk(start: Vector3, end: Vector3, _rng: RandomNumberGenerator, river_idx: int) -> PackedVector3Array:
	var path       : PackedVector3Array = PackedVector3Array()

	var p0         : Vector2 = Vector2(start.x, start.z)
	var p3         : Vector2 = Vector2(end.x, end.z)
	var total_dist : float   = p0.distance_to(p3)

	var forward    : Vector2 = (p3 - p0).normalized()
	var perp       : Vector2 = Vector2(-forward.y, forward.x)

	var rng2 : RandomNumberGenerator = RandomNumberGenerator.new()
	rng2.seed = _seed + river_idx * 9371

	var offset1 : float   = rng2.randf_range(-1.0, 1.0) * total_dist * 0.35
	var offset2 : float   = rng2.randf_range(-1.0, 1.0) * total_dist * 0.35

	var p1 : Vector2 = p0 + forward * total_dist * 0.33 + perp * offset1
	var p2 : Vector2 = p0 + forward * total_dist * 0.66 + perp * offset2

	var steps : int = 80
	for i in steps + 1:
		var t  : float = float(i) / float(steps)
		var it : float = 1.0 - t
		var bx : float = it*it*it*p0.x + 3.0*it*it*t*p1.x + 3.0*it*t*t*p2.x + t*t*t*p3.x
		var bz : float = it*it*it*p0.y + 3.0*it*it*t*p1.y + 3.0*it*t*t*p2.y + t*t*t*p3.y
		path.append(Vector3(bx, ground_level, bz))

	return path


func _find_road() -> void:
	_river_paths.clear()
	_river_points.clear()
	var rng : RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = int(_seed)

	for river_idx in river_count:
		var start_point : Vector3
		var end_point   : Vector3

		if river_idx == 0 or _river_paths.is_empty():
			# Main river: edge to edge
			var points  : Array[Vector3] = get_random_edge_points(rng, true)
			start_point = points[0]
			end_point   = points[1]
		else:
			# Tributary: start from edge, merge into midspan of an existing river
			start_point = _sample_edge_point(rng.randi() % 4, rng)
			var target_path : PackedVector3Array = _river_paths[rng.randi() % _river_paths.size()]
			var target_min  : int                = target_path.size() / 4
			var target_max  : int                = target_path.size() * 3 / 4
			var target_idx  : int                = rng.randi_range(target_min, target_max)
			end_point = target_path[target_idx]

		var raw      : PackedVector3Array = _momentum_walk(start_point, end_point, rng, river_idx)
		var smoothed : PackedVector3Array = _smooth_path_chaikin(raw, 3)
		_river_paths.append(smoothed)
		_river_points.append([start_point, end_point])


func _generate_river_layout() -> void:
	_find_road()
	_build_carve_cache()


func update_edge_marker() -> void:
	var colors := [Color(1, 0, 0), Color(0, 0, 1)]
	var names  := ["EdgeMarker", "EdgeMarker2"]
	for i in 2:
		var marker := _get_or_create_marker(names[i]) as MeshInstance3D
		if not marker.mesh:
			var sphere := SphereMesh.new()
			sphere.radius = terrain_size * 0.01
			marker.mesh = sphere
		var mat := StandardMaterial3D.new()
		mat.albedo_color = colors[i]
		marker.material_override = mat
		var xform := marker.transform
		xform.origin = Vector3.ZERO if _river_points.is_empty() else (_river_points[0] as Array)[i]
		marker.transform = xform
	_draw_road(_river_paths)
	_draw_water_surface(_river_paths)


func _build_biome_texture() -> void:
	var img  := Image.create(_CARVE_RES, _CARVE_RES, false, Image.FORMAT_RGB8)
	var half := terrain_size * 0.5
	var step := terrain_size / float(_CARVE_RES)
	for gz in _CARVE_RES:
		for gx in _CARVE_RES:
			var wx := -half + (gx + 0.5) * step
			var wz := -half + (gz + 0.5) * step
			var col: Color
			match get_biome(wx, wz):
				Biome.WATER:  col = Color(0.10, 0.30, 0.90)
				Biome.SHORE:  col = Color(0.90, 0.85, 0.40)
				Biome.FOREST: col = Color(0.20, 0.60, 0.15)
				Biome.PLAINS: col = Color(0.30, 0.50, 0.20)
				Biome.ROCKY:  col = Color(0.55, 0.52, 0.50)
				Biome.CLIFF:  col = Color(0.45, 0.33, 0.22)
				_:            col = Color.WHITE
			img.set_pixel(gx, gz, col)
	_biome_texture = ImageTexture.create_from_image(img)


func _update_biome_debug_overlay() -> void:
	var overlay := _get_or_create_marker("BiomeDebug") as MeshInstance3D
	if not show_biome_debug or _biome_texture == null:
		overlay.visible = false
		return
	overlay.visible = true
	var plane      := PlaneMesh.new()
	plane.size      = Vector2(terrain_size, terrain_size)
	overlay.mesh    = plane
	var mat        := StandardMaterial3D.new()
	mat.albedo_texture  = _biome_texture
	mat.albedo_color    = Color(1.0, 1.0, 1.0, 0.75)
	mat.transparency    = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode    = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test   = false
	overlay.material_override = mat
	var xform      := overlay.transform
	xform.origin    = Vector3(0.0, ground_level + height + 5.0, 0.0)
	overlay.transform = xform


func _init_forest_modifier() -> void:
	if not _forest_modifier.is_empty():
		return
	var cols := _CARVE_RES + 1
	_forest_modifier.resize(cols * cols)
	_forest_modifier.fill(0.0)


func _sample_forest_modifier(x: float, z: float) -> float:
	if _forest_modifier.is_empty() or _carve_step == 0.0:
		return 0.0
	var cols := _CARVE_RES + 1
	var gx: float = clamp((x + _carve_half) / _carve_step, 0.0, float(cols - 1))
	var gz: float = clamp((z + _carve_half) / _carve_step, 0.0, float(cols - 1))
	var x0 := int(gx); var x1 := mini(x0 + 1, cols - 1)
	var z0 := int(gz); var z1 := mini(z0 + 1, cols - 1)
	var tx := gx - float(x0); var tz := gz - float(z0)
	return lerpf(
		lerpf(_forest_modifier[z0 * cols + x0], _forest_modifier[z0 * cols + x1], tx),
		lerpf(_forest_modifier[z1 * cols + x0], _forest_modifier[z1 * cols + x1], tx),
		tz)


# Modify the forest biome at runtime. amount > 0 grows forest, amount < 0 shrinks it.
# radius is in world units. Updates the shader texture without rebuilding the mesh.
func modify_forest(world_x: float, world_z: float, radius: float, amount: float) -> void:
	_init_forest_modifier()
	var cols := _CARVE_RES + 1
	var min_gx := int(clamp((world_x - radius + _carve_half) / _carve_step, 0.0, float(cols - 1)))
	var max_gx := int(clamp((world_x + radius + _carve_half) / _carve_step, 0.0, float(cols - 1)))
	var min_gz := int(clamp((world_z - radius + _carve_half) / _carve_step, 0.0, float(cols - 1)))
	var max_gz := int(clamp((world_z + radius + _carve_half) / _carve_step, 0.0, float(cols - 1)))
	for gz in range(min_gz, max_gz + 1):
		for gx in range(min_gx, max_gx + 1):
			var cx := -_carve_half + gx * _carve_step
			var cz := -_carve_half + gz * _carve_step
			var d  := sqrt((cx - world_x) * (cx - world_x) + (cz - world_z) * (cz - world_z))
			if d >= radius:
				continue
			_forest_modifier[gz * cols + gx] += amount * (1.0 - d / radius)
	_refresh_biome_texture()


func reset_forest_modifiers() -> void:
	if _forest_modifier.is_empty():
		return
	_forest_modifier.fill(0.0)
	_refresh_biome_texture()


func _refresh_biome_texture() -> void:
	_build_biome_texture()
	_update_biome_debug_overlay()
	if _terrain_material and _biome_texture:
		_terrain_material.set_shader_parameter("biome_texture", _biome_texture)


func _on_map_gen_button_pressed() -> void:
	seed = randi()
	update_mesh()


func update_mesh(regenerate_rivers: bool = true) -> void:
	if noise == null:
		return
	_ensure_terrain_material()

	var should_regen := regenerate_rivers and (not freeze_river_layout_for_testing or _river_paths.is_empty())
	if should_regen or _river_paths.is_empty():
		_generate_river_layout()

	var plane := PlaneMesh.new()
	var active_res := _get_active_resolution()
	plane.subdivide_depth = active_res
	plane.subdivide_width = active_res
	plane.size = Vector2(terrain_size, terrain_size)

	var plane_arrays := plane.get_mesh_arrays()
	var verts    : PackedVector3Array  = plane_arrays[ArrayMesh.ARRAY_VERTEX]
	var normals  : PackedVector3Array  = plane_arrays[ArrayMesh.ARRAY_NORMAL]
	var tangents : PackedFloat32Array  = plane_arrays[ArrayMesh.ARRAY_TANGENT]

	var river_floor := ground_level - river_depth

	for i in verts.size():
		var v := verts[i]
		v.y = get_height(v.x, v.z)
		var n: Vector3   = get_normal(v.x, v.z)
		var tan: Vector3 = (Vector3.RIGHT - n * n.dot(Vector3.RIGHT)).normalized()

		# Apply pre-baked river carving in O(1) via bilinear cache sample
		var blend := _sample_carve_blend(Vector2(v.x, v.z))
		if blend < 1.0:
			v.y = lerpf(river_floor, v.y, blend)
			if blend < 0.01:
				n   = Vector3.UP
				tan = Vector3.RIGHT

		verts[i]   = v
		normals[i] = n
		tangents[4 * i]     = tan.x
		tangents[4 * i + 1] = tan.y
		tangents[4 * i + 2] = tan.z
		tangents[4 * i + 3] = 1.0

	var array_mesh := ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, plane_arrays)
	mesh = array_mesh
	if _terrain_material:
		material_override = _terrain_material
	update_edge_marker()
	_init_forest_modifier()
	_build_biome_texture()
	_update_biome_debug_overlay()
	if _terrain_material and _biome_texture:
		_terrain_material.set_shader_parameter("biome_texture", _biome_texture)
