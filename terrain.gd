@tool
extends MeshInstance3D
class_name TerrainMesh

@export var terrain_shader: Shader = preload("res://terrainShader.gdshader")
var _terrain_material: ShaderMaterial = null
const EDITOR_REBUILD_DEBOUNCE_SEC: float = 0.2
var _editor_rebuild_timer: Timer = null

@export var terrain_size := 1000.0;
@export_range(4, 512,4) var resolution:= 32:
	set(new_resolution):
		resolution = new_resolution
		_queue_update_mesh()
@export_range(4, 128, 4) var editor_preview_resolution: int = 16

var _generate_map := false
@export var generate_map: bool:
	get:
		return _generate_map
	set(value):
		_generate_map = value
		if value:
			_queue_update_mesh()
			_generate_map = false

var _generate_rivers_only := false
@export var generate_rivers_only: bool:
	get:
		return _generate_rivers_only
	set(value):
		_generate_rivers_only = value
		if value:
			# Force a clean river pass: clear previous layout, then rebuild mesh/rivers.
			_river_paths.clear()
			_river_points.clear()
			update_mesh(false)
			_generate_rivers_only = false

var _generate_height_only := false
@export var generate_height_only: bool:
	get:
		return _generate_height_only
	set(value):
		_generate_height_only = value
		if value:
			update_mesh(false)
			_generate_height_only = false

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
	set(new_height):
		height = new_height
		if _terrain_material:
			_terrain_material.set_shader_parameter("height", height * 2.0)
		_queue_update_mesh()

@export_range(-256.0, 256.0, 1.0) var ground_level: float = 0.0

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

var _frequency := 0.02
@export_range(0.001, 0.5, 0.001) var frequency: float:
	get:
		return _frequency
	set(value):
		_frequency = value
		if noise:
			noise.frequency = _frequency
		_queue_update_mesh()

var _lacunarity := 2.0
@export_range(1.0, 4.0, 0.01) var lacunarity: float:
	get:
		return _lacunarity
	set(value):
		_lacunarity = value
		if noise:
			noise.fractal_lacunarity = _lacunarity
		_queue_update_mesh()

var _gain := 0.5
@export_range(0.0, 1.0, 0.01) var gain: float:
	get:
		return _gain
	set(value):
		_gain = value
		if noise:
			noise.fractal_gain = _gain
		_queue_update_mesh()

var _astar: AStar3D = null
var _astar_cols: int = 0
var _river_paths: Array[PackedVector3Array] = []
var _river_points: Array[Array] = []
var _river_influence_cache: PackedFloat32Array = PackedFloat32Array()
var _river_cache_cols: int = 0
var _river_cache_step: float = 0.0
var _river_cache_half: float = 0.0
var _river_cache_valid: bool = false

var _seed: int = 0
@export var seed: int:
	get:
		return _seed
	set(value):
		_seed = value
		if noise:
			noise.seed = _seed
		_queue_update_mesh()

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

func get_height(x: float, y: float) -> float:

	#Safety check if noise is null
	if noise == null:
		return ground_level
	var active_resolution: int = _get_active_resolution()
	var step := terrain_size / float(active_resolution)
	var n00 := noise.get_noise_2d(x, y)
	var n10 := noise.get_noise_2d(x + step, y)
	var n_10 := noise.get_noise_2d(x - step, y)
	var n01 := noise.get_noise_2d(x, y + step)
	var n0_1 := noise.get_noise_2d(x, y - step)
	var n := (n00 * 4.0 + n10 + n_10 + n01 + n0_1) / 8.0
	var h := n * height
	if h < ground_level:
		h = ground_level
	return h

func get_normal(x: float, y: float) -> Vector3:
	var active_resolution: int = _get_active_resolution()
	var epsilon := terrain_size / float(active_resolution)
	var dh_dx := (get_height(x + epsilon, y) - get_height(x - epsilon, y)) / (2.0 * epsilon)
	var dh_dz := (get_height(x, y + epsilon) - get_height(x, y - epsilon)) / (2.0 * epsilon)
	# Height-field normal: cross(dP/dx, dP/dz) = (-dh/dx, 1, -dh/dz)
	return Vector3(-dh_dx, 1.0, -dh_dz).normalized()


func _astar_id(row: int, col: int) -> int:
	return row * _astar_cols + col


func build_astar() -> void:
	_astar = AStar3D.new()
	var active_resolution: int = _get_active_resolution()
	_astar_cols = active_resolution + 1
	var half := terrain_size * 0.5
	var step := terrain_size / float(active_resolution)

	for row in _astar_cols:
		for col in _astar_cols:
			var x := -half + col * step
			var z := -half + row * step
			_astar.add_point(_astar_id(row, col), Vector3(x, ground_level, z))

	for row in _astar_cols:
		for col in _astar_cols:
			var id := _astar_id(row, col)
			if not _astar.has_point(id):
				continue
			for dr in [-1, 0, 1]:
				for dc in [-1, 0, 1]:
					if dr == 0 and dc == 0:
						continue
					var nr: int = row + dr
					var nc: int = col + dc
					if nr < 0 or nr >= _astar_cols or nc < 0 or nc >= _astar_cols:
						continue
					var nid := _astar_id(nr, nc)
					if not _astar.are_points_connected(id, nid):
						_astar.connect_points(id, nid)
					
					var pos_b : Vector3 = _astar.get_point_position(nid)
					var neighbor_h : float = 0.0
					if noise:
						neighbor_h = max(0.0, noise.get_noise_2d(pos_b.x, pos_b.z) * height)
					var weight : float = 1.0 + neighbor_h * 0.08
					_astar.set_point_weight_scale(nid, weight)

func _dist_point_to_segment_2d(p: Vector2, a: Vector2, b: Vector2) -> float:
	var ab := b - a
	var len_sq := ab.dot(ab)
	if len_sq < 0.0001:
		return p.distance_to(a)
	var t: float = clamp((p - a).dot(ab) / len_sq, 0.0, 1.0)
	return p.distance_to(a + ab * t)


func _river_half_width_at_u(u: float, river_idx: int = 0) -> float:
	var base_half: float = max(river_min_width * 0.5, river_width * 0.5)
	if river_width_variation <= 0.0:
		return base_half
	var n: float = 0.0
	if noise:
		n = noise.get_noise_1d(u * river_width_noise_frequency + float(_seed + river_idx * 131) * 0.013)
	var width_scale: float = 1.0 + n * river_width_variation
	return max(river_min_width * 0.5, base_half * width_scale)


func _closest_river_info(xz: Vector2) -> Dictionary:
	if _river_paths.is_empty():
		return {"dist": INF, "u": 0.0, "river_idx": -1}
	var min_dist: float = INF
	var best_u: float = 0.0
	var best_idx: int = -1
	for river_idx in _river_paths.size():
		var river_path := _river_paths[river_idx]
		if river_path.size() < 2:
			continue
		var denom: float = max(1.0, float(river_path.size() - 1))
		for i in river_path.size() - 1:
			var a := Vector2(river_path[i].x, river_path[i].z)
			var b := Vector2(river_path[i + 1].x, river_path[i + 1].z)
			var ab := b - a
			var len_sq := ab.dot(ab)
			var t: float = 0.0
			if len_sq >= 0.0001:
				t = clamp((xz - a).dot(ab) / len_sq, 0.0, 1.0)
			var p := a + ab * t
			var d := xz.distance_to(p)
			if d < min_dist:
				min_dist = d
				best_u = (float(i) + t) / denom
				best_idx = river_idx
	return {"dist": min_dist, "u": best_u, "river_idx": best_idx}


func _compute_river_influence_uncached(xz: Vector2) -> float:
	if _river_paths.is_empty():
		return 1.0
	var river_info: Dictionary = _closest_river_info(xz)
	var dist: float = float(river_info["dist"])
	var river_idx: int = int(river_info["river_idx"])
	if river_idx < 0:
		return 1.0
	var half_width: float = _river_half_width_at_u(float(river_info["u"]), river_idx)
	var influence_radius: float = half_width + river_ground_band * river_terrain_influence
	if dist >= influence_radius:
		return 1.0
	var t: float = clamp(dist / influence_radius, 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)


func _build_river_influence_cache() -> void:
	var active_resolution: int = _get_active_resolution()
	_river_cache_cols = active_resolution + 1
	_river_cache_step = terrain_size / float(active_resolution)
	_river_cache_half = terrain_size * 0.5
	_river_influence_cache.resize(_river_cache_cols * _river_cache_cols)
	for row in _river_cache_cols:
		for col in _river_cache_cols:
			var x := -_river_cache_half + col * _river_cache_step
			var z := -_river_cache_half + row * _river_cache_step
			var idx := row * _river_cache_cols + col
			_river_influence_cache[idx] = _compute_river_influence_uncached(Vector2(x, z))
	_river_cache_valid = true


func _sample_river_influence(xz: Vector2) -> float:
	if not _river_cache_valid or _river_cache_cols <= 1:
		return _compute_river_influence_uncached(xz)
	var gx: float = (xz.x + _river_cache_half) / _river_cache_step
	var gz: float = (xz.y + _river_cache_half) / _river_cache_step
	gx = clamp(gx, 0.0, float(_river_cache_cols - 1))
	gz = clamp(gz, 0.0, float(_river_cache_cols - 1))
	var x0: int = int(floor(gx))
	var z0: int = int(floor(gz))
	var x1: int = min(x0 + 1, _river_cache_cols - 1)
	var z1: int = min(z0 + 1, _river_cache_cols - 1)
	var tx: float = gx - float(x0)
	var tz: float = gz - float(z0)
	var i00: int = z0 * _river_cache_cols + x0
	var i10: int = z0 * _river_cache_cols + x1
	var i01: int = z1 * _river_cache_cols + x0
	var i11: int = z1 * _river_cache_cols + x1
	var v00: float = _river_influence_cache[i00]
	var v10: float = _river_influence_cache[i10]
	var v01: float = _river_influence_cache[i01]
	var v11: float = _river_influence_cache[i11]
	var vx0: float = lerpf(v00, v10, tx)
	var vx1: float = lerpf(v01, v11, tx)
	return lerpf(vx0, vx1, tz)


func _apply_river_terrain_adaptation(base_height: float, xz: Vector2) -> float:
	var t: float = _sample_river_influence(xz)
	return lerpf(ground_level, base_height, t)


func _draw_road(paths: Array[PackedVector3Array]) -> void:
	var road := _get_or_create_marker("Road")
	if paths.is_empty():
		road.mesh = null
		return

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	var lifted := PackedVector3Array()
	for path in paths:
		if path.size() < 2:
			continue
		for i in path.size() - 1:
			var a := path[i]
			var b := path[i + 1]
			lifted.append(Vector3(a.x, ground_level - river_depth, a.z))
			lifted.append(Vector3(b.x, ground_level - river_depth, b.z))
	if lifted.is_empty():
		road.mesh = null
		return
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


func _make_river_curvier(path: PackedVector3Array) -> PackedVector3Array:
	if path.size() < 3:
		return path
	var curved := _smooth_path_chaikin(path, river_curve_iterations)
	if river_curve_strength <= 0.0:
		return curved
	for i in range(1, curved.size() - 1):
		var prev := curved[i - 1]
		var next := curved[i + 1]
		var forward := Vector3(next.x - prev.x, 0.0, next.z - prev.z)
		if forward.length_squared() < 0.0001:
			continue
		forward = forward.normalized()
		var right := Vector3(-forward.z, 0.0, forward.x)
		var u := float(i) / float(curved.size() - 1)
		var n: float = 0.0
		if noise:
			n = noise.get_noise_1d(u * river_curve_noise_frequency + float(_seed) * 0.021 + 17.0)
		curved[i] += right * (n * river_curve_strength)
	return _smooth_path_chaikin(curved, 1)


func _draw_water(paths: Array[PackedVector3Array]) -> void:
	var water := _get_or_create_marker("RiverWater")
	if paths.is_empty():
		water.mesh = null
		return

	var water_y  := ground_level - water_level_offset
	var verts   := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs     := PackedVector2Array()
	var indices := PackedInt32Array()
	for river_idx in paths.size():
		var path := paths[river_idx]
		var n := path.size()
		if n < 2:
			continue
		var base_vertex := verts.size()
		for i in n:
			var p := Vector3(path[i].x, water_y, path[i].z)
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
			var half_w := _river_half_width_at_u(u, river_idx)
			verts.append(p - right * half_w)
			verts.append(p + right * half_w)
			normals.append(Vector3.UP)
			normals.append(Vector3.UP)
			uvs.append(Vector2(0.0, u))
			uvs.append(Vector2(1.0, u))
		for i in n - 1:
			var base := base_vertex + i * 2
			indices.append(base);     indices.append(base + 2); indices.append(base + 1)
			indices.append(base + 1); indices.append(base + 2); indices.append(base + 3)
	if verts.is_empty():
		water.mesh = null
		return

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX]  = verts
	arrays[ArrayMesh.ARRAY_NORMAL]  = normals
	arrays[ArrayMesh.ARRAY_TEX_UV]  = uvs
	arrays[ArrayMesh.ARRAY_INDEX]   = indices

	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	var mat := ShaderMaterial.new()
	mat.shader = preload("res://RiverWater.gdshader")

	mat.set_shader_parameter("flow_speed",    0.35)
	mat.set_shader_parameter("flow_direction", Vector2(1.0, 0.2))
	mat.set_shader_parameter("tiling",         5.0)
	mat.set_shader_parameter("depth_scale",    4.0)
	mat.set_shader_parameter("normal_strength", 0.65)

	arr_mesh.surface_set_material(0, mat)
	water.mesh = arr_mesh


func _sample_edge_point(side: int, rng: RandomNumberGenerator) -> Vector3:
	var half := terrain_size * 0.5
	var x := 0.0
	var z := 0.0
	var max_attempts := 1000

	for _i in max_attempts:
		var t := rng.randf() * terrain_size - half
		match side:
			0:
				x = -half
				z = t
			1:
				x = half
				z = t
			2:
				x = t
				z = -half
			3:
				x = t
				z = half

		if not noise or get_height(x, z) == ground_level:
			break

	return Vector3(x, ground_level, z)


func get_random_edge_points(rng: RandomNumberGenerator, local_space: bool = true) -> Array[Vector3]:
	var first_side := rng.randi() % 4
	var second_side := (first_side + 1 + rng.randi() % 3) % 4

	var a := _sample_edge_point(first_side, rng)
	var b := _sample_edge_point(second_side, rng)

	if local_space:
		return [a, b]
	return [to_global(a), to_global(b)]


func _get_or_create_marker(marker_name: String) -> MeshInstance3D:
	if has_node(marker_name):
		return get_node(marker_name) as MeshInstance3D
	var marker := MeshInstance3D.new()
	marker.name = marker_name
	add_child(marker)
	return marker


func _find_road() -> void:
	_river_paths.clear()
	_river_points.clear()
	var max_retries := 120
	var rng := RandomNumberGenerator.new()
	rng.seed = int(_seed)
	for river_idx in river_count:
		for _attempt in max_retries:
			var points := get_random_edge_points(rng, true)
			if _astar == null:
				_river_points.append(points)
				break
			var a_id := _astar.get_closest_point(points[0])
			var b_id := _astar.get_closest_point(points[1])
			if a_id == -1 or b_id == -1:
				continue
			var path := _astar.get_point_path(a_id, b_id)
			if path.size() > 0:
				var final_path := _make_river_curvier(path)
				_river_paths.append(final_path)
				_river_points.append(points)
				break


func _generate_river_layout() -> void:
	build_astar()
	_find_road()
	_river_cache_valid = false


func update_edge_marker() -> void:
	var colors := [Color(1, 0, 0), Color(0, 0, 1)]
	var names  := ["EdgeMarker", "EdgeMarker2"]

	for i in 2:
		var marker := _get_or_create_marker(names[i])
		if not marker.mesh:
			var sphere := SphereMesh.new()
			sphere.radius = terrain_size * 0.01
			marker.mesh = sphere
		var mat := StandardMaterial3D.new()
		mat.albedo_color = colors[i]
		marker.material_override = mat
		var xform := marker.transform
		if _river_points.is_empty():
			xform.origin = Vector3.ZERO
		else:
			var first_pair: Array = _river_points[0]
			xform.origin = first_pair[i]
		marker.transform = xform

	_draw_road(_river_paths)
	_draw_water(_river_paths)


func _on_map_gen_button_pressed() -> void:
	seed = randi()
	update_mesh()

func update_mesh(regenerate_rivers: bool = true) -> void:
	# If no noise, no generation
	if noise == null:
		return
	_ensure_terrain_material()
	var should_regen_rivers := regenerate_rivers and (not freeze_river_layout_for_testing or _river_paths.is_empty())
	if should_regen_rivers:
		_generate_river_layout()
	elif _river_paths.is_empty():
		_generate_river_layout()

	var plane : PlaneMesh = PlaneMesh.new()
	var active_resolution: int = _get_active_resolution()
	plane.subdivide_depth = active_resolution
	plane.subdivide_width = active_resolution
	plane.size = Vector2(terrain_size, terrain_size)

	var plane_arrays := plane.get_mesh_arrays()
	var vertex_arrays : PackedVector3Array= plane_arrays[ArrayMesh.ARRAY_VERTEX]
	var normal_arrays : PackedVector3Array= plane_arrays[ArrayMesh.ARRAY_NORMAL]
	var tangent_arrays : PackedFloat32Array= plane_arrays[ArrayMesh.ARRAY_TANGENT]
	var uv_arrays : PackedVector2Array= plane_arrays[ArrayMesh.ARRAY_TEX_UV]

	var river_floor    := ground_level - river_depth
	var has_river      := not _river_paths.is_empty()

	for i: int in vertex_arrays.size():
		var vertex : Vector3 = vertex_arrays[i]
		var normal  := Vector3.UP
		var tangent := Vector3.RIGHT
		if noise:
			vertex.y = get_height(vertex.x, vertex.z)
			normal   = get_normal(vertex.x, vertex.z)
			# Gram-Schmidt: project world-X onto the surface plane for a stable tangent
			tangent  = (Vector3.RIGHT - normal * normal.dot(Vector3.RIGHT)).normalized()

		if has_river and noise:
			var river_info : Dictionary = _closest_river_info(Vector2(vertex.x, vertex.z))
			var dist: float = river_info["dist"]
			var river_idx: int = int(river_info["river_idx"])
			var half_width: float = _river_half_width_at_u(float(river_info["u"]), max(river_idx, 0))
			
			var height_diff : float = max(0.0, vertex.y - river_floor)

			var adaptive_bank : float = max(river_ground_band, height_diff * 1.5)
			var total_radius : float = half_width + adaptive_bank

			if dist < half_width:
				vertex.y = river_floor
				normal   = Vector3.UP
				tangent  = Vector3.RIGHT
			elif dist < total_radius:
				var t : float = (dist - half_width) / adaptive_bank
				t = t * t * (3.0 - 2.0 * t)
				vertex.y = lerpf(river_floor, vertex.y, t)

		vertex_arrays[i] = vertex
		normal_arrays[i] = normal
		tangent_arrays[4 * i]     = tangent.x
		tangent_arrays[4 * i + 1] = tangent.y
		tangent_arrays[4 * i + 2] = tangent.z
		tangent_arrays[4 * i + 3] = 1.0
		uv_arrays[i] = uv_arrays[i]

	var array_mesh := ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, plane_arrays)
	mesh = array_mesh
	if _terrain_material:
		material_override = _terrain_material
	update_edge_marker()