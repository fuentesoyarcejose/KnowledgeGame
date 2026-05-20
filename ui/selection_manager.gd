extends Node3D

var _selected:     Node3D         = null
var _ring:         MeshInstance3D = null
var _panel:        PanelContainer = null
var _terrain:      TerrainMesh    = null
var _area_manager: Node           = null
var _build_grid:   BuildGrid      = null
var _current_area: AreaData       = null


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_terrain      = get_parent().get_node_or_null("TerrainMesh") as TerrainMesh
	_area_manager = get_parent().get_node_or_null("AreaManager")
	_build_grid   = _find_typed(get_tree().get_root(), BuildGrid) as BuildGrid

	_ring = _build_ring()
	add_child(_ring)

	var canvas := CanvasLayer.new()
	canvas.layer = 10
	add_child(canvas)

	_panel = preload("res://ui/selection_panel.tscn").instantiate()
	_panel.visible = false
	_panel.deselected.connect(_deselect)
	_panel.area_select_requested.connect(_on_area_select_requested)
	_panel.remove_requested.connect(_on_remove_requested)
	_panel.cut_mark_toggled.connect(_on_cut_mark_toggled)
	_panel.rotate_requested.connect(_on_rotate_requested)
	_panel.terrain_linked.connect(_on_terrain_linked)
	_panel.terrain_unlinked.connect(_on_terrain_unlinked)
	canvas.add_child(_panel)


func _unhandled_input(event: InputEvent) -> void:
	if Engine.is_editor_hint():
		return
	if event is InputEventMouseButton \
			and event.button_index == MOUSE_BUTTON_LEFT \
			and event.pressed:
		_handle_click(event.position)


func _handle_click(screen_pos: Vector2) -> void:
	var camera := get_viewport().get_camera_3d()
	if camera == null:
		return
	var origin := camera.project_ray_origin(screen_pos)
	var dir    := camera.project_ray_normal(screen_pos)
	var end    := origin + dir * 2000.0
	var space  := get_world_3d().direct_space_state
	var query  := PhysicsRayQueryParameters3D.create(origin, end)
	if _terrain:
		var terrain_rid := _terrain.get_terrain_body_rid()
		if terrain_rid.is_valid():
			query.exclude = [terrain_rid]
	var result := space.intersect_ray(query)

	if result.is_empty():
		print("[SelectionManager] ray hit nothing")
		_deselect()
		return
	print("[SelectionManager] hit: ", result.collider, " | type: ", result.collider.get_class(),
		  " | parent: ", result.collider.get_parent(),
		  " | grandparent: ", result.collider.get_parent().get_parent() if result.collider.get_parent() else "none")
	var prop := _find_prop_ancestor(result.collider)
	if prop == null:
		prop = _find_human_ancestor(result.collider)
	if prop == null:
		prop = _find_plot_ancestor(result.collider)
	if prop == null:
		print("[SelectionManager] no selectable ancestor found")
		_deselect()
		return
	_select(prop)


func _select(prop: Node3D) -> void:
	_selected = prop
	_ring.visible = true
	_update_ring()
	_update_ring_color()

	_current_area = null
	if _area_manager and _area_manager.has_method("get_area_at"):
		_current_area = _area_manager.get_area_at(
			Vector2(prop.global_position.x, prop.global_position.z)
		)

	_panel.show_for(prop, _terrain, _current_area)

	if prop is EmptyPlot:
		var zones: Array = _build_grid.get_terrain_zones_adjacent_to(prop as EmptyPlot) \
		                   if _build_grid else []
		_panel.add_plot_controls(prop as EmptyPlot, zones)


func _deselect() -> void:
	_selected     = null
	_current_area = null
	_ring.visible = false
	_panel.clear()


func _update_ring_color() -> void:
	var mat := _ring.mesh.surface_get_material(0) as StandardMaterial3D
	if _selected and _selected.get("marked_for_cutting"):
		mat.albedo_color = Color(1.0, 0.45, 0.0)
	else:
		mat.albedo_color = Color(1.0, 1.0, 1.0)


func _on_cut_mark_toggled() -> void:
	if _selected == null or not is_instance_valid(_selected):
		return
	_selected.toggle_cut_mark()
	_update_ring_color()
	_panel.show_for(_selected, _terrain, _current_area)


func _on_remove_requested() -> void:
	if _selected and is_instance_valid(_selected):
		_selected.queue_free()
	_deselect()


func _on_rotate_requested() -> void:
	if _selected is EmptyPlot and is_instance_valid(_selected):
		var plot := _selected as EmptyPlot
		plot.set_direction((plot.direction + 1) % 4 as EmptyPlot.Direction)
		_select(plot)


func _on_terrain_linked(zone: Dictionary) -> void:
	if _selected is EmptyPlot and is_instance_valid(_selected):
		(_selected as EmptyPlot).link_terrain(zone)
		_select(_selected)


func _on_terrain_unlinked() -> void:
	if _selected is EmptyPlot and is_instance_valid(_selected):
		(_selected as EmptyPlot).unlink_terrain()
		_select(_selected)


func _on_area_select_requested() -> void:
	if _current_area and _area_manager and _area_manager.has_method("select_area_in_world"):
		_area_manager.select_area_in_world(_current_area)
		_deselect()


func _process(_delta: float) -> void:
	if _selected == null:
		return
	if not is_instance_valid(_selected):
		_deselect()
		return
	_update_ring()


func _update_ring() -> void:
	_ring.global_position = _selected.global_position


# Walk up from the hit collider and return the first node whose parent is a
# PropLayer. Handles any depth of nesting inside instanced scenes.
func _find_prop_ancestor(node: Node) -> Node3D:
	var current := node
	while current != null:
		if current.get_parent() is PropLayer:
			return current as Node3D
		current = current.get_parent()
	return null


# Walk up from the hit collider and return the first Human node.
func _find_human_ancestor(node: Node) -> Node3D:
	var current := node
	while current != null:
		if current is Human:
			return current as Node3D
		current = current.get_parent()
	return null


func _find_typed(node: Node, type) -> Node:
	if is_instance_of(node, type):
		return node
	for child in node.get_children():
		var found := _find_typed(child, type)
		if found:
			return found
	return null


# Walk up from the hit collider and return the first EmptyPlot node.
func _find_plot_ancestor(node: Node) -> Node3D:
	var current := node
	while current != null:
		if current is EmptyPlot:
			return current as Node3D
		current = current.get_parent()
	return null


# ── Ring mesh ─────────────────────────────────────────────────────────────────

func _build_ring() -> MeshInstance3D:
	var verts := PackedVector3Array()
	var segs  := 48
	var r     := 1.8
	for i in segs + 1:
		var a := float(i) / float(segs) * TAU
		verts.append(Vector3(cos(a) * r, 0.12, sin(a) * r))
	var arrays := []; arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINE_STRIP, arrays)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(1.0, 1.0, 1.0)
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test = true
	arr_mesh.surface_set_material(0, mat)
	var mi := MeshInstance3D.new()
	mi.mesh    = arr_mesh
	mi.visible = false
	return mi
