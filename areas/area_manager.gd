extends Node3D

const GRID_STEP := 12.5
const MIN_CELLS := 1
const MAP_SIZE  := 256
const _DECAY_CHECK_INTERVAL: float = 5.0

@export_range(10.0, 300.0, 5.0) var forest_decay_time: float = 60.0

var _terrain:    TerrainMesh      = null
var _areas:      Array            = []
var _id_grid:    PackedInt32Array = []
var _grid_cols:  int              = 0
var _grid_rows:  int              = 0
var _map_mode:   bool             = false
var _sel_area:   AreaData         = null
var _placing_district: bool       = false

# Forest decay: tracks how long each forest area has been empty of props
var _forest_empty_time: Dictionary = {}   # area_id (int) -> float (seconds empty)
var _decay_check_accum: float      = 0.0

# UI — split into two independent canvas layers
var _map_canvas:  CanvasLayer = null   # layer 20: full-screen map mode overlay
var _area_canvas: CanvasLayer = null   # layer 25: area info panel (3D + map mode)
var _map_rect:    TextureRect = null
var _hint_label:  Label       = null
var _hover_label: Label       = null
var _area_panel:  Control     = null

# 3D boundary outline
var _boundary_mesh:       MeshInstance3D = null
var _debug_boundary_mesh: MeshInstance3D = null


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_terrain = get_parent().get_node_or_null("TerrainMesh") as TerrainMesh
	_build_canvas()
	Debug.toggled.connect(_on_debug_toggled)
	Debug._area_manager = self
	if _terrain:
		_terrain.terrain_generated.connect(_on_terrain_ready)
		_on_terrain_ready()


func _on_terrain_ready() -> void:
	_forest_empty_time.clear()
	_generate_areas()
	_build_map_texture()


func _process(delta: float) -> void:
	if Engine.is_editor_hint() or _areas.is_empty():
		return
	_decay_check_accum += delta
	if _decay_check_accum < _DECAY_CHECK_INTERVAL:
		return
	_decay_check_accum = 0.0
	_check_forest_decay()


# ── Input ─────────────────────────────────────────────────────────────────────

func _unhandled_input(event: InputEvent) -> void:
	if Engine.is_editor_hint():
		return

	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_M:
			_toggle_map_mode()
			get_viewport().set_input_as_handled()
			return
		if event.keycode == KEY_ESCAPE and _map_mode:
			_toggle_map_mode()
			get_viewport().set_input_as_handled()
			return

	if not _map_mode:
		return

	if event is InputEventMouseMotion:
		_update_hover(event.position)

	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		if _placing_district:
			_place_district(event.position)
		else:
			_handle_map_click(event.position)
		get_viewport().set_input_as_handled()


# ── Area generation ───────────────────────────────────────────────────────────

func _biome_to_type(biome: int) -> int:
	match biome:
		TerrainMesh.Biome.WATER, TerrainMesh.Biome.SHORE:
			return AreaData.Type.RIVER
		TerrainMesh.Biome.FOREST:
			return AreaData.Type.FOREST
		TerrainMesh.Biome.ROCKY, TerrainMesh.Biome.CLIFF:
			return AreaData.Type.MOUNTAIN
		_:
			return AreaData.Type.PLAINS


func _generate_areas() -> void:
	_areas.clear()
	if not _terrain:
		return

	var size: float = _terrain.terrain_size
	_grid_cols = int(size / GRID_STEP)
	_grid_rows = int(size / GRID_STEP)
	var n := _grid_cols * _grid_rows

	var type_grid := PackedInt32Array()
	type_grid.resize(n)
	for row in _grid_rows:
		for col in _grid_cols:
			var wx := (float(col) / _grid_cols - 0.5) * size
			var wz := (float(row) / _grid_rows - 0.5) * size
			type_grid[row * _grid_cols + col] = _biome_to_type(_terrain.get_biome(wx, wz))

	_id_grid.resize(n)
	_id_grid.fill(-1)
	var visited := PackedByteArray()
	visited.resize(n)
	var type_count: Array = [0, 0, 0, 0, 0]

	for start in n:
		if visited[start]:
			continue
		visited[start] = 1
		var at: int = type_grid[start]

		var queue: PackedInt32Array = PackedInt32Array([start])
		var head := 0
		var cells: Array = []

		while head < queue.size():
			var idx: int = queue[head]
			head += 1
			var r := idx / _grid_cols
			var c := idx % _grid_cols
			cells.append(Vector2i(c, r))

			for nb in [Vector2i(c-1,r), Vector2i(c+1,r), Vector2i(c,r-1), Vector2i(c,r+1)]:
				var nx: int = nb.x; var ny: int = nb.y
				if nx < 0 or nx >= _grid_cols or ny < 0 or ny >= _grid_rows:
					continue
				var ni: int = ny * _grid_cols + nx
				if visited[ni] or type_grid[ni] != at:
					continue
				visited[ni] = 1
				queue.append(ni)

		if cells.size() < MIN_CELLS:
			continue

		var area := AreaData.new()
		area.id         = _areas.size()
		area.area_type  = at as AreaData.Type
		area.color      = AreaData.TYPE_COLOR.get(at, Color.WHITE)
		area.cell_count = cells.size()
		type_count[at] += 1
		area.area_name  = AreaData.TYPE_LABEL.get(at, "Area") + " #" + str(type_count[at])

		var cx := 0.0; var cz := 0.0
		for cell in cells:
			var cv := cell as Vector2i
			cx += cv.x; cz += cv.y
			_id_grid[cv.y * _grid_cols + cv.x] = area.id

		area.centroid = Vector2(
			(cx / cells.size() / _grid_cols - 0.5) * size,
			(cz / cells.size() / _grid_rows - 0.5) * size
		)
		_areas.append(area)

	_count_props()


func _count_props() -> void:
	if not _terrain:
		return
	for child in _terrain.get_children():
		if not child is PropLayer:
			continue
		var layer := child as PropLayer
		var type_name := "Props"
		if layer.prop_scene:
			type_name = layer.prop_scene.resource_path.get_file().get_basename().capitalize()
		for prop in layer.get_children():
			if not prop is Node3D:
				continue
			var p := prop as Node3D
			var aid := get_area_id_at(Vector2(p.global_position.x, p.global_position.z))
			if aid < 0:
				continue
			var area := _areas[aid] as AreaData
			area.prop_count += 1
			area.prop_types[type_name] = area.prop_types.get(type_name, 0) + 1


# ── World ↔ grid helpers ──────────────────────────────────────────────────────

func get_area_id_at(world_xz: Vector2) -> int:
	if _grid_cols == 0 or not _terrain:
		return -1
	var size: float = _terrain.terrain_size
	var c := int((world_xz.x / size + 0.5) * _grid_cols)
	var r := int((world_xz.y / size + 0.5) * _grid_rows)
	c = clampi(c, 0, _grid_cols - 1)
	r = clampi(r, 0, _grid_rows - 1)
	return _id_grid[r * _grid_cols + c]


func get_area_at(world_xz: Vector2) -> AreaData:
	var aid := get_area_id_at(world_xz)
	if aid < 0 or aid >= _areas.size():
		return null
	return _areas[aid] as AreaData


func _map_pos_to_world(screen_pos: Vector2) -> Vector2:
	var vp := get_viewport().get_visible_rect().size
	var s  := _terrain.terrain_size if _terrain else 1000.0
	return Vector2((screen_pos.x / vp.x - 0.5) * s, (screen_pos.y / vp.y - 0.5) * s)


# ── Map texture ───────────────────────────────────────────────────────────────

func _build_map_texture() -> void:
	if _grid_cols == 0 or not _map_rect:
		return

	var img := Image.create(MAP_SIZE, MAP_SIZE, false, Image.FORMAT_RGB8)
	for row in MAP_SIZE:
		for col in MAP_SIZE:
			var gc := clampi(int(float(col) / MAP_SIZE * _grid_cols), 0, _grid_cols - 1)
			var gr := clampi(int(float(row) / MAP_SIZE * _grid_rows), 0, _grid_rows - 1)
			var aid: int = _id_grid[gr * _grid_cols + gc]
			var c := Color(0.10, 0.10, 0.10)
			if aid >= 0 and aid < _areas.size():
				c = (_areas[aid] as AreaData).color
			img.set_pixel(col, row, c)

	var copy := img.duplicate() as Image
	for row in MAP_SIZE:
		for col in MAP_SIZE:
			var gc := clampi(int(float(col) / MAP_SIZE * _grid_cols), 0, _grid_cols - 1)
			var gr := clampi(int(float(row) / MAP_SIZE * _grid_rows), 0, _grid_rows - 1)
			var aid: int = _id_grid[gr * _grid_cols + gc]
			for nb in [Vector2i(col-1,row), Vector2i(col+1,row), Vector2i(col,row-1), Vector2i(col,row+1)]:
				if nb.x < 0 or nb.x >= MAP_SIZE or nb.y < 0 or nb.y >= MAP_SIZE:
					continue
				var nc := clampi(int(float(nb.x) / MAP_SIZE * _grid_cols), 0, _grid_cols - 1)
				var nr := clampi(int(float(nb.y) / MAP_SIZE * _grid_rows), 0, _grid_rows - 1)
				if _id_grid[nr * _grid_cols + nc] != aid:
					copy.set_pixel(col, row, Color(0.0, 0.0, 0.0))
					break

	_map_rect.texture = ImageTexture.create_from_image(copy)


# ── 3D boundary outline ───────────────────────────────────────────────────────

func _adj_add(adj: Dictionary, a: Vector2i, b: Vector2i) -> void:
	if not adj.has(a): adj[a] = []
	if not adj.has(b): adj[b] = []
	(adj[a] as Array).append(b)
	(adj[b] as Array).append(a)


func _corner_to_world(corner: Vector2i) -> Vector2:
	var size: float = _terrain.terrain_size
	return Vector2(
		float(corner.x) / _grid_cols * size - size * 0.5,
		float(corner.y) / _grid_rows * size - size * 0.5
	)


func _trace_loops(adj: Dictionary) -> Array:
	var loops: Array = []
	var rem: Dictionary = {}
	for corner in adj:
		rem[corner] = (adj[corner] as Array).duplicate()

	while true:
		var start: Variant = null
		for c in rem:
			if not (rem[c] as Array).is_empty():
				start = c
				break
		if start == null:
			break
		var loop: Array = []
		var current: Variant = start
		while true:
			loop.append(current)
			var neighbors: Array = rem[current]
			if neighbors.is_empty():
				break
			var nxt: Variant = neighbors[0]
			(rem[current] as Array).erase(nxt)
			if rem.has(nxt):
				(rem[nxt] as Array).erase(current)
			current = nxt
			if current == start:
				break
		if loop.size() >= 3:
			loops.append(loop)

	return loops


func _chaikin_loop(pts: Array, iterations: int) -> Array:
	var result: Array = pts.duplicate()
	for _i in iterations:
		var n := result.size()
		var next_pts: Array = []
		for j in n:
			var a: Vector2 = result[j]
			var b: Vector2 = result[(j + 1) % n]
			next_pts.append(a * 0.75 + b * 0.25)
			next_pts.append(a * 0.25 + b * 0.75)
		result = next_pts
	return result


func _draw_boundary(area: AreaData) -> void:
	if _boundary_mesh:
		_boundary_mesh.queue_free()
		_boundary_mesh = null
	if not _terrain or _grid_cols == 0:
		return

	var adj: Dictionary = {}
	for row in _grid_rows:
		for col in _grid_cols:
			if _id_grid[row * _grid_cols + col] != area.id:
				continue
			var aid: int = area.id
			var c00 := Vector2i(col,     row)
			var c10 := Vector2i(col + 1, row)
			var c01 := Vector2i(col,     row + 1)
			var c11 := Vector2i(col + 1, row + 1)
			if col == 0 or _id_grid[row * _grid_cols + (col - 1)] != aid:
				_adj_add(adj, c00, c01)
			if col == _grid_cols - 1 or _id_grid[row * _grid_cols + (col + 1)] != aid:
				_adj_add(adj, c10, c11)
			if row == 0 or _id_grid[(row - 1) * _grid_cols + col] != aid:
				_adj_add(adj, c00, c10)
			if row == _grid_rows - 1 or _id_grid[(row + 1) * _grid_cols + col] != aid:
				_adj_add(adj, c01, c11)

	if adj.is_empty():
		return

	var verts := PackedVector3Array()
	const Y_OFFSET := 1.5
	for loop in _trace_loops(adj):
		var world_pts: Array = []
		for corner in loop:
			world_pts.append(_corner_to_world(corner as Vector2i))
		var smooth := _chaikin_loop(world_pts, 3)
		var n := smooth.size()
		for j in n:
			var a: Vector2 = smooth[j]
			var b: Vector2 = smooth[(j + 1) % n]
			verts.append(_terrain.to_global(Vector3(a.x, _terrain.get_height(a.x, a.y) + Y_OFFSET, a.y)))
			verts.append(_terrain.to_global(Vector3(b.x, _terrain.get_height(b.x, b.y) + Y_OFFSET, b.y)))

	if verts.is_empty():
		return

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)

	var mat := StandardMaterial3D.new()
	mat.albedo_color = area.color
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test = false
	arr_mesh.surface_set_material(0, mat)

	_boundary_mesh = MeshInstance3D.new()
	_boundary_mesh.mesh = arr_mesh
	add_child(_boundary_mesh)


func _clear_boundary() -> void:
	if _boundary_mesh:
		_boundary_mesh.queue_free()
		_boundary_mesh = null


# ── Public API (called from SelectionManager) ─────────────────────────────────

func select_area_in_world(area: AreaData) -> void:
	_sel_area = area
	_draw_boundary(area)
	if _area_panel:
		_area_panel.call("show_for", area)
	_area_canvas.visible = true


func deselect_area_in_world() -> void:
	_clear_boundary()
	_sel_area = null
	if _area_panel:
		_area_panel.visible = false
	if not _map_mode:
		_area_canvas.visible = false


# ── UI construction ───────────────────────────────────────────────────────────

func _build_canvas() -> void:
	# Map canvas — full-screen map mode overlay
	_map_canvas = CanvasLayer.new()
	_map_canvas.layer = 20
	add_child(_map_canvas)

	var bg := ColorRect.new()
	bg.color = Color(0.0, 0.0, 0.0, 0.50)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_PASS
	_map_canvas.add_child(bg)

	_map_rect = TextureRect.new()
	_map_rect.set_anchors_preset(Control.PRESET_FULL_RECT)
	_map_rect.expand_mode  = TextureRect.EXPAND_IGNORE_SIZE
	_map_rect.stretch_mode = TextureRect.STRETCH_SCALE
	_map_rect.mouse_filter = Control.MOUSE_FILTER_PASS
	_map_canvas.add_child(_map_rect)

	_hint_label = Label.new()
	_hint_label.text = "MAP MODE   [M / ESC] Close   [Click] Select area"
	_hint_label.set_anchors_preset(Control.PRESET_TOP_WIDE)
	_hint_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_hint_label.add_theme_font_size_override("font_size", 14)
	_hint_label.add_theme_color_override("font_color", Color(1, 1, 1, 0.9))
	_hint_label.offset_top    = 10
	_hint_label.offset_bottom = 36
	_hint_label.mouse_filter  = Control.MOUSE_FILTER_IGNORE
	_map_canvas.add_child(_hint_label)

	_hover_label = Label.new()
	_hover_label.add_theme_font_size_override("font_size", 13)
	_hover_label.add_theme_color_override("font_color", Color(1, 1, 1))
	_hover_label.add_theme_color_override("font_shadow_color", Color(0, 0, 0, 0.8))
	_hover_label.add_theme_constant_override("shadow_offset_x", 1)
	_hover_label.add_theme_constant_override("shadow_offset_y", 1)
	_hover_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_map_canvas.add_child(_hover_label)

	_map_canvas.visible = false

	# Area canvas — panel shown both in 3D mode and on top of the map
	_area_canvas = CanvasLayer.new()
	_area_canvas.layer = 25
	add_child(_area_canvas)

	_area_panel = preload("res://ui/area_panel.tscn").instantiate()
	_area_panel.visible = false
	(_area_panel as Node).connect("closed",             _on_area_panel_closed)
	(_area_panel as Node).connect("district_requested", _on_district_requested)
	(_area_panel as Node).connect("name_changed",       _on_area_name_changed)
	_area_canvas.add_child(_area_panel)

	_area_canvas.visible = false


# ── Map mode toggle ───────────────────────────────────────────────────────────

func _toggle_map_mode() -> void:
	_map_mode = not _map_mode
	_map_canvas.visible = _map_mode
	_placing_district   = false

	if _map_mode:
		# Hide the 3D boundary when entering map (already coloured on map)
		if _boundary_mesh:
			_boundary_mesh.visible = false
	else:
		# Restore boundary and area panel when exiting map
		if _boundary_mesh:
			_boundary_mesh.visible = true
		if not _sel_area:
			_area_canvas.visible = false


# ── Map interaction ───────────────────────────────────────────────────────────

func _update_hover(screen_pos: Vector2) -> void:
	var area := get_area_at(_map_pos_to_world(screen_pos))
	if area:
		_hover_label.text     = area.area_name
		_hover_label.position = screen_pos + Vector2(14.0, -8.0)
	else:
		_hover_label.text = ""


func _handle_map_click(screen_pos: Vector2) -> void:
	var area := get_area_at(_map_pos_to_world(screen_pos))
	if area:
		_select_area_map(area)
	else:
		_on_area_panel_closed()


func _select_area_map(area: AreaData) -> void:
	_sel_area = area
	if _area_panel:
		_area_panel.call("show_for", area)
	_area_canvas.visible = true


func _on_area_panel_closed() -> void:
	_sel_area = null
	if _area_panel:
		_area_panel.visible = false
	_area_canvas.visible = false
	_clear_boundary()
	_placing_district = false
	if _hint_label:
		_hint_label.text = "MAP MODE   [M / ESC] Close   [Click] Select area"


# ── District placement ────────────────────────────────────────────────────────

func _on_district_requested() -> void:
	_placing_district = true
	if _hint_label:
		_hint_label.text = "DISTRICT PLACEMENT   Click on the map to place   [ESC] Cancel"


func _place_district(screen_pos: Vector2) -> void:
	_placing_district = false
	if _hint_label:
		_hint_label.text = "MAP MODE   [M / ESC] Close   [Click] Select area"
	if not _terrain:
		return

	var world := _map_pos_to_world(screen_pos)
	var size:  float = _terrain.terrain_size
	const DISTRICT_RADIUS := 150.0

	var area := AreaData.new()
	area.id        = _areas.size()
	area.area_type = AreaData.Type.DISTRICT
	area.color     = AreaData.TYPE_COLOR.get(AreaData.Type.DISTRICT, Color.ORANGE)
	area.area_name = "District #" + str(_count_of_type(AreaData.Type.DISTRICT) + 1)
	area.centroid  = world

	for r in _grid_rows:
		for c in _grid_cols:
			var wx := (float(c) / _grid_cols - 0.5) * size
			var wz := (float(r) / _grid_rows - 0.5) * size
			if Vector2(wx, wz).distance_to(world) <= DISTRICT_RADIUS:
				area.cell_count += 1
				_id_grid[r * _grid_cols + c] = area.id

	_areas.append(area)
	_build_map_texture()
	_select_area_map(area)


func _count_of_type(t: AreaData.Type) -> int:
	var n := 0
	for a in _areas:
		if (a as AreaData).area_type == t:
			n += 1
	return n


# ── Area rename ───────────────────────────────────────────────────────────────

func _on_area_name_changed(new_name: String) -> void:
	if _sel_area:
		_sel_area.area_name = new_name


# ── Forest decay ──────────────────────────────────────────────────────────────

# Returns the world-XZ centre of every grid cell that belongs to this area.
func _area_cell_centers(area: AreaData) -> Array:
	var out:  Array = []
	var size: float = _terrain.terrain_size
	for row in _grid_rows:
		for col in _grid_cols:
			if _id_grid[row * _grid_cols + col] != area.id:
				continue
			out.append(Vector2(
				(float(col) + 0.5) / _grid_cols * size - size * 0.5,
				(float(row) + 0.5) / _grid_rows * size - size * 0.5
			))
	return out


# Returns a dict mapping area_id → live prop count across all PropLayers.
func _build_live_prop_counts() -> Dictionary:
	var counts: Dictionary = {}
	if not _terrain:
		return counts
	for child in _terrain.get_children():
		if not child is PropLayer:
			continue
		for prop in (child as PropLayer).get_children():
			if not prop is Node3D or not is_instance_valid(prop):
				continue
			var p := prop as Node3D
			var aid := get_area_id_at(Vector2(p.global_position.x, p.global_position.z))
			if aid >= 0:
				counts[aid] = counts.get(aid, 0) + 1
	return counts


func _check_forest_decay() -> void:
	var counts := _build_live_prop_counts()
	var changed := false
	for a in _areas:
		var area := a as AreaData
		if area.area_type != AreaData.Type.FOREST:
			_forest_empty_time.erase(area.id)
			continue
		var live: int = counts.get(area.id, 0)
		if live == 0:
			var elapsed: float = _forest_empty_time.get(area.id, 0.0) + _DECAY_CHECK_INTERVAL
			if elapsed >= forest_decay_time:
				var centers := _area_cell_centers(area)
				area.area_name = "Plains #" + str(_count_of_type(AreaData.Type.PLAINS) + 1)
				area.area_type = AreaData.Type.PLAINS
				area.color     = AreaData.TYPE_COLOR.get(AreaData.Type.PLAINS, Color.WHITE)
				_forest_empty_time.erase(area.id)
				if _terrain:
					_terrain.deforest_cells(centers)
				changed = true
			else:
				_forest_empty_time[area.id] = elapsed
		else:
			_forest_empty_time.erase(area.id)
	if changed:
		_refresh_after_biome_change()


# Called by the debug "Update Biomes" button — immediately converts every
# empty forest area to plains without waiting for the decay timer.
func update_biome_conversions() -> void:
	var counts  := _build_live_prop_counts()
	var changed := false
	for a in _areas:
		var area := a as AreaData
		if area.area_type != AreaData.Type.FOREST:
			continue
		if counts.get(area.id, 0) == 0:
			var centers := _area_cell_centers(area)
			area.area_name = "Plains #" + str(_count_of_type(AreaData.Type.PLAINS) + 1)
			area.area_type = AreaData.Type.PLAINS
			area.color     = AreaData.TYPE_COLOR.get(AreaData.Type.PLAINS, Color.WHITE)
			_forest_empty_time.erase(area.id)
			if _terrain:
				_terrain.deforest_cells(centers)
			changed = true
	if changed:
		_refresh_after_biome_change()


func _refresh_after_biome_change() -> void:
	_build_map_texture()
	if _debug_boundary_mesh:
		_debug_boundary_mesh.queue_free()
		_debug_boundary_mesh = null
	if Debug.enabled:
		_draw_all_boundaries()


# ── Debug ─────────────────────────────────────────────────────────────────────

func _on_debug_toggled(dbg: bool) -> void:
	if _debug_boundary_mesh:
		_debug_boundary_mesh.queue_free()
		_debug_boundary_mesh = null
	if dbg:
		_draw_all_boundaries()


func _draw_all_boundaries() -> void:
	if not _terrain or _grid_cols == 0 or _areas.is_empty():
		return

	var per_area_adj: Dictionary = {}
	for row in _grid_rows:
		for col in _grid_cols:
			var aid: int = _id_grid[row * _grid_cols + col]
			if aid < 0:
				continue
			if not per_area_adj.has(aid):
				per_area_adj[aid] = {}
			var adj: Dictionary = per_area_adj[aid]
			var c00 := Vector2i(col,     row)
			var c10 := Vector2i(col + 1, row)
			var c01 := Vector2i(col,     row + 1)
			var c11 := Vector2i(col + 1, row + 1)
			if col == 0 or _id_grid[row * _grid_cols + (col - 1)] != aid:
				_adj_add(adj, c00, c01)
			if col == _grid_cols - 1 or _id_grid[row * _grid_cols + (col + 1)] != aid:
				_adj_add(adj, c10, c11)
			if row == 0 or _id_grid[(row - 1) * _grid_cols + col] != aid:
				_adj_add(adj, c00, c10)
			if row == _grid_rows - 1 or _id_grid[(row + 1) * _grid_cols + col] != aid:
				_adj_add(adj, c01, c11)

	var verts  := PackedVector3Array()
	var colors := PackedColorArray()
	const Y_OFFSET := 1.5

	for a in _areas:
		var area := a as AreaData
		if not per_area_adj.has(area.id):
			continue
		for loop in _trace_loops(per_area_adj[area.id]):
			var world_pts: Array = []
			for corner in loop:
				world_pts.append(_corner_to_world(corner as Vector2i))
			var smooth := _chaikin_loop(world_pts, 3)
			var n := smooth.size()
			for j in n:
				var p1: Vector2 = smooth[j]
				var p2: Vector2 = smooth[(j + 1) % n]
				verts.append(_terrain.to_global(Vector3(p1.x, _terrain.get_height(p1.x, p1.y) + Y_OFFSET, p1.y)))
				verts.append(_terrain.to_global(Vector3(p2.x, _terrain.get_height(p2.x, p2.y) + Y_OFFSET, p2.y)))
				colors.append(area.color)
				colors.append(area.color)

	if verts.is_empty():
		return

	var arrays := []
	arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	arrays[ArrayMesh.ARRAY_COLOR]  = colors
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)

	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test = false
	arr_mesh.surface_set_material(0, mat)

	_debug_boundary_mesh = MeshInstance3D.new()
	_debug_boundary_mesh.mesh = arr_mesh
	add_child(_debug_boundary_mesh)
