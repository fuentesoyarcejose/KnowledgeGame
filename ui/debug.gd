## Global debug / cheat singleton.
## Access from any script as:  Debug.enabled
## Toggle in-game:             F1
## React to changes:           Debug.toggled.connect(my_func)
extends Node

signal toggled(enabled: bool)

var enabled: bool = false:
	set(v):
		enabled = v
		_hud.visible = v
		if not v:
			_set_biome_overlay(false)
			if _placing_human:
				_end_placement(false)
		toggled.emit(v)

# ── HUD ───────────────────────────────────────────────────────────────────────
var _hud:            CanvasLayer = null
var _fps_lbl:        Label       = null
var _info_lbl:       Label       = null
var _biome_btn:      Button      = null
var _spawn_btn:      Button      = null
var _terrain:        Node        = null   # set by terrain.gd
var _camera:         Node3D      = null   # set by camera_base.gd
var _area_manager:   Node        = null   # set by area_manager.gd
var _colony_manager: Node        = null   # set by colony_manager.gd

# ── Human placement ───────────────────────────────────────────────────────────
var _placing_human: bool   = false
var _human_ghost:   Node3D = null


func _ready() -> void:
	_build_hud()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_F1:
			enabled = not enabled
			get_viewport().set_input_as_handled()
			return
		if _placing_human and event.keycode == KEY_ESCAPE:
			_end_placement(false)
			get_viewport().set_input_as_handled()
			return

	if _placing_human and event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if not mb.pressed:
			return
		if mb.button_index == MOUSE_BUTTON_LEFT:
			_end_placement(true)
			get_viewport().set_input_as_handled()
		elif mb.button_index == MOUSE_BUTTON_RIGHT:
			_end_placement(false)
			get_viewport().set_input_as_handled()


func _process(_delta: float) -> void:
	if not enabled or not _fps_lbl:
		return

	_fps_lbl.text = "FPS: %d" % Engine.get_frames_per_second()

	var info_lines: Array[String] = []

	if _camera:
		var pos: Vector3 = (_camera as Node3D).global_position
		info_lines.append("Cam: (%.0f, %.0f, %.0f)" % [pos.x, pos.y, pos.z])

	if _terrain and _terrain.has_method("get_biome"):
		var cam_pos := Vector3.ZERO
		if _camera:
			cam_pos = (_camera as Node3D).global_position
		var biome: int    = _terrain.call("get_biome", cam_pos.x, cam_pos.z)
		var keys: Array   = TerrainMesh.Biome.keys()
		var biome_name: String = keys[biome].capitalize() if biome < keys.size() else "?"
		info_lines.append("Biome: " + biome_name)

		var seed_val = _terrain.get("_seed")
		if seed_val != null:
			info_lines.append("Seed: " + str(seed_val))

	_info_lbl.text = "\n".join(info_lines)

	# Update ghost position every frame while placing.
	if _placing_human and _human_ghost:
		var hit := _get_terrain_hit()
		if hit.x < 1e9:
			_human_ghost.global_position = hit


# ── Human placement ───────────────────────────────────────────────────────────

func _on_spawn_human_pressed() -> void:
	if _placing_human:
		_end_placement(false)
	else:
		_start_placement()


func _start_placement() -> void:
	_placing_human = true
	if _spawn_btn:
		_spawn_btn.text = "Cancel [Esc / RMB]"
	_human_ghost = _build_ghost()
	var scene := get_tree().get_current_scene()
	if scene:
		scene.add_child(_human_ghost)


func _end_placement(confirm: bool) -> void:
	if confirm and _human_ghost and _colony_manager:
		var pos := _human_ghost.global_position
		_colony_manager.call("spawn_human", pos)
	if _human_ghost:
		_human_ghost.queue_free()
		_human_ghost = null
	_placing_human = false
	if _spawn_btn:
		_spawn_btn.text = "Spawn Human"


func _build_ghost() -> Node3D:
	var root := Node3D.new()

	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.35, 1.0, 0.35, 0.55)
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA

	var body_mesh := CapsuleMesh.new()
	body_mesh.radius = 0.28
	body_mesh.height = 1.5
	var body_mi := MeshInstance3D.new()
	body_mi.mesh              = body_mesh
	body_mi.material_override = mat
	body_mi.position.y        = 0.85
	body_mi.cast_shadow       = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	root.add_child(body_mi)

	var head_mesh := SphereMesh.new()
	head_mesh.radius = 0.22
	head_mesh.height = 0.44
	var head_mi := MeshInstance3D.new()
	head_mi.mesh              = head_mesh
	head_mi.material_override = mat
	head_mi.position.y        = 1.8
	head_mi.cast_shadow       = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	root.add_child(head_mi)

	var ring_mat := StandardMaterial3D.new()
	ring_mat.albedo_color  = Color(0.3, 1.0, 0.3, 1.0)
	ring_mat.shading_mode  = BaseMaterial3D.SHADING_MODE_UNSHADED
	ring_mat.transparency  = BaseMaterial3D.TRANSPARENCY_DISABLED

	var ring_mesh := TorusMesh.new()
	ring_mesh.inner_radius = 0.45
	ring_mesh.outer_radius = 0.65
	var ring_mi := MeshInstance3D.new()
	ring_mi.mesh              = ring_mesh
	ring_mi.material_override = ring_mat
	ring_mi.cast_shadow       = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	root.add_child(ring_mi)

	return root


func _get_terrain_hit() -> Vector3:
	var camera := get_viewport().get_camera_3d()
	if not camera or not _terrain:
		return Vector3(1e9, 0.0, 1e9)
	var mouse := get_viewport().get_mouse_position()
	var ro    := camera.project_ray_origin(mouse)
	var rd    := camera.project_ray_normal(mouse)
	if absf(rd.y) < 0.001:
		return Vector3(1e9, 0.0, 1e9)
	var ground_y: float = _terrain.get("ground_level") if _terrain.get("ground_level") != null else 0.0
	var t := (ground_y - ro.y) / rd.y
	if t < 0.0:
		return Vector3(1e9, 0.0, 1e9)
	var hit := ro + rd * t
	var y: float = _terrain.call("get_height", hit.x, hit.z)
	return Vector3(hit.x, y, hit.z)


# ── Biome overlay ─────────────────────────────────────────────────────────────

func _on_biome_btn_pressed() -> void:
	if not _terrain:
		return
	var current: bool = _terrain.get("show_biome_debug")
	_set_biome_overlay(not current)


func _on_biome_update_pressed() -> void:
	if _area_manager and _area_manager.has_method("update_biome_conversions"):
		_area_manager.call("update_biome_conversions")


func _set_biome_overlay(on: bool) -> void:
	if not _terrain:
		return
	_terrain.set("show_biome_debug", on)
	if _biome_btn:
		_biome_btn.text = "Biome Overlay: " + ("ON" if on else "OFF")


# ── HUD construction ──────────────────────────────────────────────────────────

func _build_hud() -> void:
	_hud = CanvasLayer.new()
	_hud.layer = 99
	add_child(_hud)

	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.offset_left   = 10
	panel.offset_top    = 10
	panel.offset_right  = 240
	panel.offset_bottom = 10
	panel.mouse_filter  = Control.MOUSE_FILTER_STOP
	_hud.add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 4)
	panel.add_child(vbox)

	var title := Label.new()
	title.text = "— DEBUG MODE (F1) —"
	title.add_theme_font_size_override("font_size", 12)
	title.add_theme_color_override("font_color", Color(1.0, 0.6, 0.1))
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(title)

	_fps_lbl = Label.new()
	_fps_lbl.add_theme_font_size_override("font_size", 12)
	_fps_lbl.add_theme_color_override("font_color", Color(0.9, 0.9, 0.9))
	vbox.add_child(_fps_lbl)

	_info_lbl = Label.new()
	_info_lbl.add_theme_font_size_override("font_size", 12)
	_info_lbl.add_theme_color_override("font_color", Color(0.75, 0.95, 0.75))
	vbox.add_child(_info_lbl)

	vbox.add_child(HSeparator.new())

	_biome_btn = Button.new()
	_biome_btn.text = "Biome Overlay: OFF"
	_biome_btn.add_theme_font_size_override("font_size", 12)
	_biome_btn.pressed.connect(_on_biome_btn_pressed)
	vbox.add_child(_biome_btn)

	var biome_update_btn := Button.new()
	biome_update_btn.text = "Update Biomes"
	biome_update_btn.add_theme_font_size_override("font_size", 12)
	biome_update_btn.pressed.connect(_on_biome_update_pressed)
	vbox.add_child(biome_update_btn)

	vbox.add_child(HSeparator.new())

	_spawn_btn = Button.new()
	_spawn_btn.text = "Spawn Human"
	_spawn_btn.add_theme_font_size_override("font_size", 12)
	_spawn_btn.pressed.connect(_on_spawn_human_pressed)
	vbox.add_child(_spawn_btn)

	_hud.visible = false
