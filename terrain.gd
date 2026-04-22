@tool
extends MeshInstance3D

@export var terrain_size := 1000.0;
@export_range(4, 512,4) var resolution:= 32:
	set(new_resolution):
		resolution = new_resolution
		update_mesh()

var _generate_map := false
@export var generate_map: bool:
	get:
		return _generate_map
	set(value):
		_generate_map = value
		if value:
			update_mesh()
			_generate_map = false

@export var noise: FastNoiseLite:
	set(new_noise):
		noise = new_noise
		if noise:
			noise.seed = _seed
			noise.frequency = _frequency
			noise.fractal_lacunarity = _lacunarity
			noise.fractal_gain = _gain
			noise.changed.connect(update_mesh)
		update_mesh()

@export_range(4.0, 128.0, 4.0) var height := 64.0:
	set(new_height):
		height = new_height
		material_override.set_shader_parameter("height", height * 2.0)
		update_mesh()

@export_range(-256.0, 256.0, 1.0) var ground_level: float = 0.0

@export var river_depth: float = 10.0
@export var river_width: float = 20.0
@export_range(0.0, 1.0, 0.01) var river_width_variation: float = 0.4
@export_range(0.1, 8.0, 0.1) var river_width_noise_frequency: float = 2.0
@export var river_min_width: float = 6.0
@export var river_ground_band: float = 5.0
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
		update_mesh()

var _lacunarity := 2.0
@export_range(1.0, 4.0, 0.01) var lacunarity: float:
	get:
		return _lacunarity
	set(value):
		_lacunarity = value
		if noise:
			noise.fractal_lacunarity = _lacunarity
		update_mesh()

var _gain := 0.5
@export_range(0.0, 1.0, 0.01) var gain: float:
	get:
		return _gain
	set(value):
		_gain = value
		if noise:
			noise.fractal_gain = _gain
		update_mesh()

var _astar: AStar3D = null
var _astar_cols: int = 0
var _road_path: PackedVector3Array = []
var _river_path: PackedVector3Array = []
var _road_points: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO]

var _seed: int = 0
@export var seed: int:
	get:
		return _seed
	set(value):
		_seed = value
		if noise:
			noise.seed = _seed
		update_mesh()

func get_height(x: float, y: float) -> float:
	var step := terrain_size / float(resolution)
	var n00 := noise.get_noise_2d(x, y)
	var n10 := noise.get_noise_2d(x + step, y)
	var n_10 := noise.get_noise_2d(x - step, y)
	var n01 := noise.get_noise_2d(x, y + step)
	var n0_1 := noise.get_noise_2d(x, y - step)

	# Simple 5-sample blur: center weighted more than neighbors
	var n := (n00 * 4.0 + n10 + n_10 + n01 + n0_1) / 8.0
	# Raise blended noise to the 4th power (preserve sign) for sharper peaks/valleys
	var h := n * height
	# Clamp everything below ground_level up to a flat ground plateau
	if h < ground_level:
		h = ground_level
	return h

func get_normal(x: float, y: float) -> Vector3:
	var epsilon := terrain_size / resolution
	var dh_dx := (get_height(x + epsilon, y) - get_height(x - epsilon, y)) / (2.0 * epsilon)
	var dh_dz := (get_height(x, y + epsilon) - get_height(x, y - epsilon)) / (2.0 * epsilon)
	# Height-field normal: cross(dP/dx, dP/dz) = (-dh/dx, 1, -dh/dz)
	return Vector3(-dh_dx, 1.0, -dh_dz).normalized()


func _astar_id(row: int, col: int) -> int:
	return row * _astar_cols + col


func build_astar() -> void:
	_astar = AStar3D.new()
	_astar_cols = resolution + 1
	var half := terrain_size * 0.5
	var step := terrain_size / float(resolution)

	for row in _astar_cols:
		for col in _astar_cols:
			var x := -half + col * step
			var z := -half + row * step
			var h := get_height(x, z) if noise else ground_level
			if h == ground_level:
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
					if _astar.has_point(nid) and not _astar.are_points_connected(id, nid):
						_astar.connect_points(id, nid)


func _dist_point_to_segment_2d(p: Vector2, a: Vector2, b: Vector2) -> float:
	var ab := b - a
	var len_sq := ab.dot(ab)
	if len_sq < 0.0001:
		return p.distance_to(a)
	var t: float = clamp((p - a).dot(ab) / len_sq, 0.0, 1.0)
	return p.distance_to(a + ab * t)


func _river_half_width_at_u(u: float) -> float:
	var base_half: float = max(river_min_width * 0.5, river_width * 0.5)
	if river_width_variation <= 0.0:
		return base_half
	var n: float = 0.0
	if noise:
		n = noise.get_noise_1d(u * river_width_noise_frequency + float(_seed) * 0.013)
	var width_scale: float = 1.0 + n * river_width_variation
	return max(river_min_width * 0.5, base_half * width_scale)


func _closest_river_info(xz: Vector2) -> Dictionary:
	if _river_path.size() < 2:
		return {"dist": INF, "u": 0.0}
	var min_dist: float = INF
	var best_u: float = 0.0
	var denom: float = max(1.0, float(_river_path.size() - 1))
	for i in _river_path.size() - 1:
		var a := Vector2(_river_path[i].x, _river_path[i].z)
		var b := Vector2(_river_path[i + 1].x, _river_path[i + 1].z)
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
	return {"dist": min_dist, "u": best_u}


func _draw_road(path: PackedVector3Array) -> void:
	var road := _get_or_create_marker("Road")
	if path.size() < 2:
		road.mesh = null
		return

	path = _smooth_path_chaikin(path)
	var lifted := PackedVector3Array()
	for p in path:
		lifted.append(Vector3(p.x, ground_level - river_depth, p.z))

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = lifted

	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINE_STRIP, arrays)

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
		var n := 0.0
		if noise:
			n = noise.get_noise_1d(u * river_curve_noise_frequency + float(_seed) * 0.021 + 17.0)
		curved[i] += right * (n * river_curve_strength)
	return _smooth_path_chaikin(curved, 1)


func _draw_water(path: PackedVector3Array) -> void:
	var water := _get_or_create_marker("RiverWater")
	if path.size() < 2:
		water.mesh = null
		return

	path = _smooth_path_chaikin(path)
	var water_y := ground_level - water_level_offset
	var n       := path.size()

	var verts   := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs     := PackedVector2Array()
	var indices := PackedInt32Array()

	for i in n:
		var p := Vector3(path[i].x, water_y, path[i].z)

		var fwd := Vector3.ZERO
		if i < n - 1:
			fwd += Vector3(path[i + 1].x - path[i].x, 0.0, path[i + 1].z - path[i].z)
		if i > 0:
			fwd += Vector3(path[i].x - path[i - 1].x, 0.0, path[i].z - path[i - 1].z)
		if fwd.length_squared() < 0.0001:
			fwd = Vector3.FORWARD
		fwd = fwd.normalized()

		# Perpendicular in XZ plane
		var right := Vector3(-fwd.z, 0.0, fwd.x)

		var u := float(i) / float(max(1, n - 1))
		var half_w := _river_half_width_at_u(u)
		verts.append(p - right * half_w)
		verts.append(p + right * half_w)
		normals.append(Vector3.UP)
		normals.append(Vector3.UP)
		uvs.append(Vector2(0.0, u))
		uvs.append(Vector2(1.0, u))

	for i in n - 1:
		var base := i * 2
		indices.append(base);     indices.append(base + 1); indices.append(base + 2)
		indices.append(base + 1); indices.append(base + 3); indices.append(base + 2)

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	arrays[ArrayMesh.ARRAY_NORMAL] = normals
	arrays[ArrayMesh.ARRAY_TEX_UV] = uvs
	arrays[ArrayMesh.ARRAY_INDEX]  = indices

	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	var mat := StandardMaterial3D.new()
	mat.albedo_color        = Color(0.08, 0.38, 0.74, 0.78)
	mat.transparency        = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.roughness           = 0.05
	mat.metallic            = 0.0
	mat.metallic_specular   = 0.9
	mat.cull_mode           = BaseMaterial3D.CULL_DISABLED
	arr_mesh.surface_set_material(0, mat)

	water.mesh = arr_mesh


func _sample_edge_point(side: int) -> Vector3:
	var half := terrain_size * 0.5
	var x := 0.0
	var z := 0.0
	var max_attempts := 1000

	for _i in max_attempts:
		var t := randf() * terrain_size - half
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


func get_random_edge_points(local_space: bool = true) -> Array[Vector3]:
	var first_side := randi() % 4
	var second_side := (first_side + 1 + randi() % 3) % 4

	var a := _sample_edge_point(first_side)
	var b := _sample_edge_point(second_side)

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
	_road_path = []
	_river_path = []
	_road_points = [Vector3.ZERO, Vector3.ZERO]
	var max_retries := 100

	for _attempt in max_retries:
		var points := get_random_edge_points(true)
		if _astar == null:
			_road_points = points
			break
		var a_id := _astar.get_closest_point(points[0])
		var b_id := _astar.get_closest_point(points[1])
		if a_id == -1 or b_id == -1:
			continue
		_road_path = _astar.get_point_path(a_id, b_id)
		if _road_path.size() > 0:
			_river_path = _make_river_curvier(_road_path)
			_road_points = points
			break


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
		xform.origin = _road_points[i]
		marker.transform = xform

	_draw_road(_river_path)
	_draw_water(_river_path)


func _on_map_gen_button_pressed() -> void:
	seed = randi()
	update_mesh()

func update_mesh() -> void:
	build_astar()
	_find_road()

	var plane : PlaneMesh = PlaneMesh.new()
	plane.subdivide_depth = resolution
	plane.subdivide_width = resolution
	plane.size = Vector2(terrain_size, terrain_size)

	var plane_arrays := plane.get_mesh_arrays()
	var vertex_arrays : PackedVector3Array= plane_arrays[ArrayMesh.ARRAY_VERTEX]
	var normal_arrays : PackedVector3Array= plane_arrays[ArrayMesh.ARRAY_NORMAL]
	var tangent_arrays : PackedFloat32Array= plane_arrays[ArrayMesh.ARRAY_TANGENT]
	var uv_arrays : PackedVector2Array= plane_arrays[ArrayMesh.ARRAY_TEX_UV]

	var river_floor    := ground_level - river_depth
	var has_river      := _river_path.size() >= 2

	for i: int in vertex_arrays.size():
		var vertex : Vector3 = vertex_arrays[i]
		var normal  := Vector3.UP
		var tangent := Vector3.RIGHT
		if noise:
			vertex.y = get_height(vertex.x, vertex.z)
			normal   = get_normal(vertex.x, vertex.z)
			# Gram-Schmidt: project world-X onto the surface plane for a stable tangent
			tangent  = (Vector3.RIGHT - normal * normal.dot(Vector3.RIGHT)).normalized()

		if has_river:
			var river_info := _closest_river_info(Vector2(vertex.x, vertex.z))
			var dist: float = river_info["dist"]
			var half_width := _river_half_width_at_u(river_info["u"])
			var total_radius := half_width + river_ground_band
			if dist < half_width:
				# Flat river bed
				vertex.y = river_floor
				normal   = Vector3.UP
				tangent  = Vector3.RIGHT
			elif dist < total_radius:
				# Smooth bank: blend from river floor up to natural terrain height
				var t := (dist - half_width) / river_ground_band
				t = smoothstep(0.0, 1.0, t)
				vertex.y = lerpf(river_floor, vertex.y, t)
				# Leave normal as the terrain normal; close enough for a gentle slope

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

	update_edge_marker()
