extends Node3D
class_name BuildGrid

signal mode_changed(mode: int)   # emits Mode value
signal terrain_pending_changed(has_cells: bool, is_connected: bool)

const GRID_STEP := 5.0

@export_range(5, 30, 1) var view_radius: int = 20

enum Mode      { NONE, BUILD, ROAD, TERRAIN }
enum CellState { VALID, WATER, BLOCKED, OUT_OF_BOUNDS }

var _terrain:      TerrainMesh    = null
var _mode:         Mode           = Mode.NONE
var _hovered:      Vector2i       = Vector2i(999999, 999999)

# Build mode
var _drag_start:   Vector2i       = Vector2i.ZERO
var _dragging:     bool           = false
var _buildings:    Dictionary     = {}   # Vector2i → MeshInstance3D

# Road mode
var _road_cells:   Dictionary     = {}   # Vector2i → MeshInstance3D
var _road_start:   Vector2i       = Vector2i(999999, 999999)
var _road_placing: bool           = false

# Terrain zone mode
var _terrain_drag_start:    Vector2i     = Vector2i(999999, 999999)
var _terrain_dragging:      bool         = false
var _pending_terrain_cells: Dictionary   = {}   # Vector2i → true (accumulated pending)
var _pending_mesh:          MeshInstance3D = null
var _final_terrain_zones:   Array        = []   # Array[Dictionary] {id, label, cells, mesh}
var _terrain_zone_cell_set: Dictionary   = {}   # Vector2i → true, flat lookup for get_cell_state
var _zone_id_counter:       int          = 0

# Build direction (cycles with right-click during BUILD mode)
var _build_direction: EmptyPlot.Direction = EmptyPlot.Direction.SOUTH

# Shared
var _prop_cells:   Dictionary     = {}   # Vector2i → true
var _overlay_mi:   MeshInstance3D = null
var _overlay_mat:  ShaderMaterial = null


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_terrain = get_parent().get_node_or_null("TerrainMesh") as TerrainMesh
	if not _terrain:
		return
	_terrain.terrain_generated.connect(_on_terrain_ready)
	_on_terrain_ready()
	_init_overlay()


func _on_terrain_ready() -> void:
	_rebuild_prop_cache()
	for b in _buildings.values():
		if is_instance_valid(b): b.queue_free()
	_buildings.clear()
	for r in _road_cells.values():
		if is_instance_valid(r): r.queue_free()
	_road_cells.clear()
	for z: Dictionary in _final_terrain_zones:
		var m := z.get("mesh") as MeshInstance3D
		if is_instance_valid(m): m.queue_free()
	_final_terrain_zones.clear()
	_terrain_zone_cell_set.clear()
	_clear_terrain_pending()


# ── Input ──────────────────────────────────────────────────────────────────────

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_B:
				set_mode(Mode.NONE if _mode == Mode.BUILD else Mode.BUILD)
				get_viewport().set_input_as_handled()
				return
			KEY_R:
				if _mode == Mode.BUILD:
					_build_direction = (_build_direction + 1) % 4 as EmptyPlot.Direction
					_refresh_overlay()
				else:
					set_mode(Mode.NONE if _mode == Mode.ROAD else Mode.ROAD)
				get_viewport().set_input_as_handled()
				return
			KEY_ESCAPE:
				if _mode == Mode.BUILD and _dragging:
					_dragging = false
					_refresh_overlay()
				elif _mode == Mode.ROAD and _road_placing:
					_road_placing = false
					_refresh_overlay()
				elif _mode == Mode.TERRAIN and _terrain_dragging:
					_terrain_dragging   = false
					_terrain_drag_start = Vector2i(999999, 999999)
					_refresh_overlay()
				elif _mode != Mode.NONE:
					set_mode(Mode.NONE)
				get_viewport().set_input_as_handled()
				return

	if _mode == Mode.NONE:
		return

	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton

		# Right-click during BUILD: cycle placement direction
		if _mode == Mode.BUILD and mb.button_index == MOUSE_BUTTON_RIGHT and mb.pressed:
			_build_direction = (_build_direction + 1) % 4 as EmptyPlot.Direction
			_refresh_overlay()
			get_viewport().set_input_as_handled()
			return

		if mb.button_index != MOUSE_BUTTON_LEFT:
			return

		if _mode == Mode.BUILD:
			if mb.pressed:
				_place_empty_plot(_hovered, _build_direction)
				_refresh_overlay()

		elif _mode == Mode.ROAD:
			if mb.pressed:
				if not _road_placing:
					_road_start   = _hovered
					_road_placing = true
					_refresh_overlay()
				else:
					_commit_road()
					_road_placing = false
					_refresh_overlay()

		elif _mode == Mode.TERRAIN:
			if mb.pressed:
				_terrain_drag_start = _hovered
				_terrain_dragging   = true
				_refresh_overlay()
			elif _terrain_dragging:
				_add_drag_to_pending()
				_terrain_dragging   = false
				_terrain_drag_start = Vector2i(999999, 999999)
				_refresh_overlay()

		get_viewport().set_input_as_handled()


func _process(_delta: float) -> void:
	if _mode == Mode.NONE or not _terrain:
		return
	var camera := get_viewport().get_camera_3d()
	if not camera:
		return
	var cell := _screen_to_cell(camera)
	if cell == _hovered:
		return
	_hovered = cell
	_refresh_overlay()


# ── Grid helpers ───────────────────────────────────────────────────────────────

func _screen_to_cell(camera: Camera3D) -> Vector2i:
	var mouse := get_viewport().get_mouse_position()
	var ro    := camera.project_ray_origin(mouse)
	var rd    := camera.project_ray_normal(mouse)
	if absf(rd.y) < 0.001:
		return _hovered
	var t := (_terrain.ground_level - ro.y) / rd.y
	if t < 0.0:
		return _hovered
	var hit := ro + rd * t
	return Vector2i(roundi(hit.x / GRID_STEP), roundi(hit.z / GRID_STEP))


func cell_to_world(cell: Vector2i) -> Vector2:
	return Vector2(cell.x * GRID_STEP, cell.y * GRID_STEP)


func get_cell_state(cell: Vector2i) -> CellState:
	var c    := cell_to_world(cell)
	var half := _terrain.terrain_size * 0.5
	if absf(c.x) > half or absf(c.y) > half:
		return CellState.OUT_OF_BOUNDS
	if _buildings.has(cell) or _prop_cells.has(cell) or _road_cells.has(cell) \
			or _terrain_zone_cell_set.has(cell):
		return CellState.BLOCKED
	match _terrain.get_biome(c.x, c.y):
		TerrainMesh.Biome.ROCKY, TerrainMesh.Biome.CLIFF:
			return CellState.BLOCKED
		TerrainMesh.Biome.WATER, TerrainMesh.Biome.SHORE:
			return CellState.WATER
	return CellState.VALID


func _is_routable(cell: Vector2i) -> bool:
	var c    := cell_to_world(cell)
	var half := _terrain.terrain_size * 0.5
	if absf(c.x) > half or absf(c.y) > half:
		return false
	if _buildings.has(cell) or _prop_cells.has(cell):
		return false
	match _terrain.get_biome(c.x, c.y):
		TerrainMesh.Biome.ROCKY, TerrainMesh.Biome.CLIFF, \
		TerrainMesh.Biome.WATER, TerrainMesh.Biome.SHORE:
			return false
	return true


# ── Prop cache ─────────────────────────────────────────────────────────────────

func _rebuild_prop_cache() -> void:
	_prop_cells.clear()
	if not _terrain:
		return
	for child in _terrain.get_children():
		if not child is PropLayer:
			continue
		for prop in (child as PropLayer).get_children():
			if not prop is Node3D:
				continue
			var p := prop as Node3D
			_prop_cells[Vector2i(roundi(p.position.x / GRID_STEP), roundi(p.position.z / GRID_STEP))] = true


# ── Building placement ─────────────────────────────────────────────────────────

func _drag_cells() -> Array:
	var c0 := Vector2i(mini(_drag_start.x, _hovered.x), mini(_drag_start.y, _hovered.y))
	var c1 := Vector2i(maxi(_drag_start.x, _hovered.x), maxi(_drag_start.y, _hovered.y))
	c1.x = mini(c1.x, c0.x + 49)
	c1.y = mini(c1.y, c0.y + 49)
	var cells: Array = []
	for r in range(c0.y, c1.y + 1):
		for c in range(c0.x, c1.x + 1):
			cells.append(Vector2i(c, r))
	return cells


func _place_empty_plot(origin: Vector2i, dir: EmptyPlot.Direction = EmptyPlot.Direction.SOUTH) -> void:
	for r in EmptyPlot.PLOT_CELLS:
		for c in EmptyPlot.PLOT_CELLS:
			if get_cell_state(Vector2i(origin.x + c, origin.y + r)) != CellState.VALID:
				return

	var plot := EmptyPlot.new()
	get_parent().add_child(plot)
	plot.setup(origin, _terrain, dir)

	for r in EmptyPlot.PLOT_CELLS:
		for c in EmptyPlot.PLOT_CELLS:
			_buildings[Vector2i(origin.x + c, origin.y + r)] = plot


# ── Road placement ─────────────────────────────────────────────────────────────

func _road_path(a: Vector2i, b: Vector2i) -> Array:
	var cells: Array = []
	var x := a.x
	var y := a.y
	var dx := signi(b.x - a.x)
	var dy := signi(b.y - a.y)
	while x != b.x:
		cells.append(Vector2i(x, y))
		x += dx
	while y != b.y:
		cells.append(Vector2i(x, y))
		y += dy
	cells.append(Vector2i(x, y))
	return cells


func _commit_road() -> void:
	if _road_start == Vector2i(999999, 999999):
		return
	for cell in _road_path(_road_start, _hovered):
		if _is_routable(cell) or _road_cells.has(cell):
			_place_road(cell)


func _place_road(cell: Vector2i) -> void:
	if _road_cells.has(cell):
		return
	var c   := cell_to_world(cell)
	var box := BoxMesh.new()
	box.size = Vector3(GRID_STEP, 0.12, GRID_STEP)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.28, 0.25, 0.21)
	mat.roughness    = 0.95
	var mi  := MeshInstance3D.new()
	mi.mesh              = box
	mi.material_override = mat
	mi.position          = Vector3(c.x, _terrain.ground_level + 0.06, c.y)
	get_parent().add_child(mi)
	_road_cells[cell] = mi


# ── Mode ───────────────────────────────────────────────────────────────────────

func set_mode(m: Mode) -> void:
	_mode               = m
	_dragging           = false
	_road_placing       = false
	_terrain_dragging   = false
	_terrain_drag_start = Vector2i(999999, 999999)
	if m != Mode.TERRAIN:
		_clear_terrain_pending()
	if _overlay_mi:
		_overlay_mi.visible = (m != Mode.NONE)
	if m != Mode.NONE:
		_rebuild_prop_cache()
		_hovered = Vector2i(999999, 999999)
	else:
		if _overlay_mi:
			_overlay_mi.mesh = null
	mode_changed.emit(int(_mode))


func get_mode() -> Mode:
	return _mode


# Backward-compatible helpers used by game_hud before mode_changed was added.
func _toggle(on: bool) -> void:
	set_mode(Mode.BUILD if on else Mode.NONE)


func is_build_active() -> bool:
	return _mode == Mode.BUILD


# ── Overlay ────────────────────────────────────────────────────────────────────

func _init_overlay() -> void:
	var shader := Shader.new()
	shader.code = """
shader_type spatial;
render_mode unshaded, cull_disabled, depth_draw_never;
void fragment() {
	ALBEDO = COLOR.rgb;
	ALPHA  = COLOR.a;
}
"""
	_overlay_mat        = ShaderMaterial.new()
	_overlay_mat.shader = shader
	_overlay_mi         = MeshInstance3D.new()
	_overlay_mi.material_override = _overlay_mat
	_overlay_mi.visible = false
	add_child(_overlay_mi)


func _refresh_overlay() -> void:
	if not _overlay_mi or not _overlay_mi.visible:
		return
	_overlay_mi.mesh = _build_overlay_mesh()


func _build_overlay_mesh() -> ArrayMesh:
	var gy := _terrain.ground_level + 0.05
	var qh := GRID_STEP * 0.5 - 0.05

	var verts  := PackedVector3Array()
	var colors := PackedColorArray()
	var idxs   := PackedInt32Array()

	if _mode == Mode.BUILD:
		_fill_build_quads(verts, colors, idxs, gy, qh)
	elif _mode == Mode.ROAD:
		_fill_road_quads(verts, colors, idxs, gy, qh)
	elif _mode == Mode.TERRAIN:
		_fill_terrain_quads(verts, colors, idxs, gy, qh)

	# Grid lines (shared)
	var lverts  := PackedVector3Array()
	var lcolors := PackedColorArray()
	var lcol    := Color(1.0, 1.0, 1.0, 0.12)
	var n       := 2 * view_radius + 2
	var x0 := float(_hovered.x - view_radius) * GRID_STEP - GRID_STEP * 0.5
	var x1 := float(_hovered.x + view_radius) * GRID_STEP + GRID_STEP * 0.5
	var z0 := float(_hovered.y - view_radius) * GRID_STEP - GRID_STEP * 0.5
	var z1 := float(_hovered.y + view_radius) * GRID_STEP + GRID_STEP * 0.5
	for i in n:
		var x := float(_hovered.x - view_radius + i) * GRID_STEP - GRID_STEP * 0.5
		lverts.append(Vector3(x, gy, z0)); lverts.append(Vector3(x, gy, z1))
		lcolors.append(lcol);              lcolors.append(lcol)
		var z := float(_hovered.y - view_radius + i) * GRID_STEP - GRID_STEP * 0.5
		lverts.append(Vector3(x0, gy, z)); lverts.append(Vector3(x1, gy, z))
		lcolors.append(lcol);              lcolors.append(lcol)

	# Perimeter outline + direction arrow in BUILD mode
	if _mode == Mode.BUILD and _hovered != Vector2i(999999, 999999):
		var half := EmptyPlot.PLOT_CELLS * GRID_STEP * 0.5   # 10.0
		var pw   := EmptyPlot.PLOT_CELLS * GRID_STEP          # 20.0

		# Check if entire footprint is valid
		var all_valid := true
		for r in EmptyPlot.PLOT_CELLS:
			for c in EmptyPlot.PLOT_CELLS:
				if get_cell_state(Vector2i(_hovered.x + c, _hovered.y + r)) != CellState.VALID:
					all_valid = false
					break
			if not all_valid:
				break

		var pcol := Color(0.30, 1.00, 0.30, 0.95) if all_valid else Color(1.00, 0.28, 0.28, 0.95)
		var ly   := gy + 0.02
		var ox   := float(_hovered.x) * GRID_STEP - GRID_STEP * 0.5
		var oz   := float(_hovered.y) * GRID_STEP - GRID_STEP * 0.5
		var corners := [
			Vector3(ox,      ly, oz),
			Vector3(ox + pw, ly, oz),
			Vector3(ox + pw, ly, oz + pw),
			Vector3(ox,      ly, oz + pw),
		]
		for i in 4:
			lverts.append(corners[i]); lverts.append(corners[(i + 1) % 4])
			lcolors.append(pcol); lcolors.append(pcol)

		# Direction arrow (cyan when valid, dimmed when blocked)
		var cx      := (float(_hovered.x) + EmptyPlot.PLOT_CELLS * 0.5) * GRID_STEP
		var cz      := (float(_hovered.y) + EmptyPlot.PLOT_CELLS * 0.5) * GRID_STEP
		var angle   := _build_direction * PI / 2.0
		var shaft   := Vector3(sin(angle), 0.0, -cos(angle))
		var perp    := Vector3(cos(angle), 0.0,  sin(angle))
		var center3 := Vector3(cx, ly, cz)
		var tip     := center3 + shaft * (half - 0.8)
		var wpos    := tip - shaft * 4.0
		var acol    := Color(0.40, 0.85, 1.00, 0.92) if all_valid else Color(0.85, 0.50, 0.40, 0.70)
		lverts.append(center3 + shaft * 2.0); lverts.append(tip)
		lcolors.append(acol); lcolors.append(acol)
		lverts.append(tip); lverts.append(wpos + perp * 2.5)
		lcolors.append(acol); lcolors.append(acol)
		lverts.append(tip); lverts.append(wpos - perp * 2.5)
		lcolors.append(acol); lcolors.append(acol)

	var arr_mesh := ArrayMesh.new()
	if not verts.is_empty():
		var a := []; a.resize(ArrayMesh.ARRAY_MAX)
		a[ArrayMesh.ARRAY_VERTEX] = verts
		a[ArrayMesh.ARRAY_COLOR]  = colors
		a[ArrayMesh.ARRAY_INDEX]  = idxs
		arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, a)
		arr_mesh.surface_set_material(0, _overlay_mat)
	if not lverts.is_empty():
		var a := []; a.resize(ArrayMesh.ARRAY_MAX)
		a[ArrayMesh.ARRAY_VERTEX] = lverts
		a[ArrayMesh.ARRAY_COLOR]  = lcolors
		arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, a)
		arr_mesh.surface_set_material(arr_mesh.get_surface_count() - 1, _overlay_mat)
	return arr_mesh


func _fill_build_quads(verts: PackedVector3Array, colors: PackedColorArray,
		idxs: PackedInt32Array, gy: float, qh: float) -> void:
	# Evaluate the full 4×4 footprint at _hovered
	var footprint: Dictionary = {}   # Vector2i → CellState
	var all_valid := true
	for r in EmptyPlot.PLOT_CELLS:
		for c in EmptyPlot.PLOT_CELLS:
			var cell  := Vector2i(_hovered.x + c, _hovered.y + r)
			var state := get_cell_state(cell)
			footprint[cell] = state
			if state != CellState.VALID:
				all_valid = false

	# Faint background grid outside the footprint
	var half := _terrain.terrain_size * 0.5
	for dr in range(-view_radius, view_radius + 1):
		for dc in range(-view_radius, view_radius + 1):
			var cell := Vector2i(_hovered.x + dc, _hovered.y + dr)
			if footprint.has(cell):
				continue
			var cw := cell_to_world(cell)
			if absf(cw.x) > half or absf(cw.y) > half:
				continue
			_append_quad(verts, colors, idxs, cell, gy, qh, Color(0.30, 0.70, 0.30, 0.07))

	# 4×4 footprint fill
	for cell: Vector2i in footprint.keys():
		var state := footprint[cell] as CellState
		var col: Color
		if all_valid:
			col = Color(0.28, 0.92, 0.28, 0.42)
		else:
			match state:
				CellState.VALID:         col = Color(0.92, 0.88, 0.28, 0.30)
				CellState.WATER:         col = Color(0.28, 0.52, 0.95, 0.50)
				CellState.BLOCKED:       col = Color(0.95, 0.22, 0.22, 0.55)
				CellState.OUT_OF_BOUNDS: col = Color(0.60, 0.20, 0.20, 0.45)
		_append_quad(verts, colors, idxs, cell, gy, qh, col)


func _fill_road_quads(verts: PackedVector3Array, colors: PackedColorArray,
		idxs: PackedInt32Array, gy: float, qh: float) -> void:
	var path_set: Dictionary = {}
	if _road_placing and _road_start != Vector2i(999999, 999999):
		for pc in _road_path(_road_start, _hovered):
			path_set[pc] = true

	for dr in range(-view_radius, view_radius + 1):
		for dc in range(-view_radius, view_radius + 1):
			var cell := Vector2i(_hovered.x + dc, _hovered.y + dr)
			var col:   Color

			if path_set.has(cell):
				col = Color(0.90, 0.72, 0.35, 0.70) if _is_routable(cell) or _road_cells.has(cell) \
				    else Color(1.00, 0.30, 0.30, 0.60)
			elif cell == _road_start and _road_placing:
				col = Color(1.00, 0.85, 0.30, 0.85)
			elif cell == _hovered:
				col = Color(0.90, 0.72, 0.35, 0.65) if _is_routable(cell) or _road_cells.has(cell) \
				    else Color(1.00, 0.30, 0.30, 0.55)
			elif _road_cells.has(cell):
				col = Color(0.60, 0.46, 0.28, 0.40)
			else:
				var c    := cell_to_world(cell)
				var half := _terrain.terrain_size * 0.5
				if absf(c.x) > half or absf(c.y) > half:
					continue
				if _is_routable(cell):
					col = Color(0.70, 0.58, 0.30, 0.15)
				else:
					col = Color(0.80, 0.25, 0.25, 0.10)
			_append_quad(verts, colors, idxs, cell, gy, qh, col)


func _append_quad(verts: PackedVector3Array, colors: PackedColorArray,
		idxs: PackedInt32Array, cell: Vector2i, gy: float, qh: float, col: Color) -> void:
	var cx := float(cell.x) * GRID_STEP
	var cz := float(cell.y) * GRID_STEP
	var b   := verts.size()
	verts.append(Vector3(cx - qh, gy, cz - qh))
	verts.append(Vector3(cx + qh, gy, cz - qh))
	verts.append(Vector3(cx + qh, gy, cz + qh))
	verts.append(Vector3(cx - qh, gy, cz + qh))
	for _i in 4:
		colors.append(col)
	idxs.append_array([b, b + 1, b + 2, b, b + 2, b + 3])


# ── Terrain zone placement ─────────────────────────────────────────────────────

func _terrain_rect_cells() -> Array:
	if _terrain_drag_start == Vector2i(999999, 999999):
		return []
	var c0 := Vector2i(mini(_terrain_drag_start.x, _hovered.x), mini(_terrain_drag_start.y, _hovered.y))
	var c1 := Vector2i(maxi(_terrain_drag_start.x, _hovered.x), maxi(_terrain_drag_start.y, _hovered.y))
	var cells: Array = []
	for r in range(c0.y, c1.y + 1):
		for c in range(c0.x, c1.x + 1):
			cells.append(Vector2i(c, r))
	return cells


func _add_drag_to_pending() -> void:
	for cell in _terrain_rect_cells():
		_pending_terrain_cells[cell] = true
	_rebuild_pending_mesh()


func _clear_terrain_pending() -> void:
	_pending_terrain_cells.clear()
	if is_instance_valid(_pending_mesh):
		_pending_mesh.queue_free()
	_pending_mesh = null
	terrain_pending_changed.emit(false, false)


func is_terrain_connected() -> bool:
	if _pending_terrain_cells.is_empty():
		return false
	var start: Vector2i = _pending_terrain_cells.keys()[0] as Vector2i
	var visited: Dictionary = {}
	var queue: Array        = [start]
	visited[start]          = true
	while not queue.is_empty():
		var cell: Vector2i = queue.pop_front() as Vector2i
		for nb: Vector2i in [Vector2i(cell.x + 1, cell.y), Vector2i(cell.x - 1, cell.y),
		                      Vector2i(cell.x, cell.y + 1), Vector2i(cell.x, cell.y - 1)]:
			if _pending_terrain_cells.has(nb) and not visited.has(nb):
				visited[nb] = true
				queue.append(nb)
	return visited.size() == _pending_terrain_cells.size()


func _rebuild_pending_mesh() -> void:
	if is_instance_valid(_pending_mesh):
		_pending_mesh.queue_free()
	_pending_mesh = null

	if _pending_terrain_cells.is_empty():
		terrain_pending_changed.emit(false, false)
		return

	var connected := is_terrain_connected()
	var gy  := _terrain.ground_level + 0.04
	var qh  := GRID_STEP * 0.5 - 0.05
	var fill := Color(0.68, 0.54, 0.22, 0.32)
	var lcol := Color(0.95, 0.80, 0.30, 0.85) if connected \
	          else Color(0.95, 0.35, 0.20, 0.85)

	var verts  := PackedVector3Array()
	var colors := PackedColorArray()
	var idxs   := PackedInt32Array()
	for cell in _pending_terrain_cells.keys():
		_append_quad(verts, colors, idxs, cell, gy, qh, fill)

	var lverts  := PackedVector3Array()
	var lcolors := PackedColorArray()
	var ly      := gy + 0.01
	for cell: Vector2i in _pending_terrain_cells.keys():
		var cx := float(cell.x) * GRID_STEP
		var cz := float(cell.y) * GRID_STEP
		var nbs    := [Vector2i(cell.x - 1, cell.y), Vector2i(cell.x + 1, cell.y),
		               Vector2i(cell.x, cell.y - 1), Vector2i(cell.x, cell.y + 1)]
		var segs   := [
			[Vector3(cx - qh, ly, cz - qh), Vector3(cx - qh, ly, cz + qh)],
			[Vector3(cx + qh, ly, cz - qh), Vector3(cx + qh, ly, cz + qh)],
			[Vector3(cx - qh, ly, cz - qh), Vector3(cx + qh, ly, cz - qh)],
			[Vector3(cx - qh, ly, cz + qh), Vector3(cx + qh, ly, cz + qh)],
		]
		for i in 4:
			if not _pending_terrain_cells.has(nbs[i]):
				lverts.append(segs[i][0]); lverts.append(segs[i][1])
				lcolors.append(lcol);      lcolors.append(lcol)

	var arr_mesh := ArrayMesh.new()
	if not verts.is_empty():
		var a := []; a.resize(ArrayMesh.ARRAY_MAX)
		a[ArrayMesh.ARRAY_VERTEX] = verts
		a[ArrayMesh.ARRAY_COLOR]  = colors
		a[ArrayMesh.ARRAY_INDEX]  = idxs
		arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, a)
		arr_mesh.surface_set_material(0, _overlay_mat)
	if not lverts.is_empty():
		var a := []; a.resize(ArrayMesh.ARRAY_MAX)
		a[ArrayMesh.ARRAY_VERTEX] = lverts
		a[ArrayMesh.ARRAY_COLOR]  = lcolors
		arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, a)
		arr_mesh.surface_set_material(arr_mesh.get_surface_count() - 1, _overlay_mat)

	_pending_mesh      = MeshInstance3D.new()
	_pending_mesh.mesh = arr_mesh
	add_child(_pending_mesh)

	terrain_pending_changed.emit(true, connected)


func accept_terrain() -> bool:
	if _pending_terrain_cells.is_empty() or not is_terrain_connected():
		return false
	_zone_id_counter += 1
	var zone := {
		"id":    _zone_id_counter,
		"label": "Zone %d  (%d cells)" % [_zone_id_counter, _pending_terrain_cells.size()],
		"cells": _pending_terrain_cells.duplicate(),
		"mesh":  _pending_mesh,
	}
	_final_terrain_zones.append(zone)
	for cell: Vector2i in zone.cells.keys():
		_terrain_zone_cell_set[cell] = true
	_pending_mesh = null
	_pending_terrain_cells.clear()
	set_mode(Mode.NONE)
	return true


func get_terrain_zones() -> Array:
	return _final_terrain_zones


func get_terrain_zones_adjacent_to(plot: EmptyPlot) -> Array:
	var plot_cells: Dictionary = {}
	for cell in plot.get_cells():
		plot_cells[cell] = true

	var result: Array = []
	for zone: Dictionary in _final_terrain_zones:
		var zone_cells: Dictionary = zone.get("cells", {})
		for zc: Vector2i in zone_cells.keys():
			var adjacent_to_plot := false
			for nb: Vector2i in [Vector2i(zc.x + 1, zc.y), Vector2i(zc.x - 1, zc.y),
			                      Vector2i(zc.x, zc.y + 1), Vector2i(zc.x, zc.y - 1)]:
				if plot_cells.has(nb):
					adjacent_to_plot = true
					break
			if adjacent_to_plot:
				result.append(zone)
				break
	return result


func _fill_terrain_quads(verts: PackedVector3Array, colors: PackedColorArray,
		idxs: PackedInt32Array, gy: float, qh: float) -> void:
	var half := _terrain.terrain_size * 0.5

	if _terrain_dragging and _terrain_drag_start != Vector2i(999999, 999999):
		var drag_set: Dictionary = {}
		for dc in _terrain_rect_cells():
			drag_set[dc] = true

		for dr in range(-view_radius, view_radius + 1):
			for dc in range(-view_radius, view_radius + 1):
				var cell := Vector2i(_hovered.x + dc, _hovered.y + dr)
				if drag_set.has(cell):
					_append_quad(verts, colors, idxs, cell, gy, qh, Color(0.95, 0.82, 0.28, 0.62))
				else:
					var c := cell_to_world(cell)
					if absf(c.x) > half or absf(c.y) > half:
						continue
					_append_quad(verts, colors, idxs, cell, gy, qh, Color(0.60, 0.48, 0.18, 0.08))
	else:
		for dr in range(-view_radius, view_radius + 1):
			for dc in range(-view_radius, view_radius + 1):
				var cell := Vector2i(_hovered.x + dc, _hovered.y + dr)
				var c    := cell_to_world(cell)
				if absf(c.x) > half or absf(c.y) > half:
					continue
				var col := Color(0.88, 0.72, 0.22, 0.65) if cell == _hovered \
				         else Color(0.60, 0.48, 0.18, 0.06)
				_append_quad(verts, colors, idxs, cell, gy, qh, col)
