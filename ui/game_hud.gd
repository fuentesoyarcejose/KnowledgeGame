extends CanvasLayer
class_name GameHUD

var _day_night:      DayNightCycle  = null
var _build_grid:     BuildGrid      = null
var _colony_manager: ColonyManager  = null
var _camera_base:    CameraBase     = null

var _time_lbl:  Label  = null
var _day_lbl:   Label  = null
var _pop_btn:   Button = null
var _build_btn: Button = null
var _road_btn:  Button = null

var _colony_panel:        PanelContainer = null
var _colony_list:         VBoxContainer  = null
var _colony_refresh_timer: float         = 0.0

var _build_submenu: PanelContainer = null

var _terrain_panel:      PanelContainer = null
var _terrain_accept_btn: Button         = null
var _terrain_status_lbl: Label          = null


var _day_count:  int   = 1
var _prev_time:  float = -1.0


func _ready() -> void:
	layer = 10
	_build_ui()
	call_deferred("_connect_nodes")


func _connect_nodes() -> void:
	_day_night      = _find_typed(get_tree().get_root(), DayNightCycle) as DayNightCycle
	_build_grid     = _find_typed(get_tree().get_root(), BuildGrid)     as BuildGrid
	_colony_manager = _find_typed(get_tree().get_root(), ColonyManager) as ColonyManager
	_camera_base    = _find_typed(get_tree().get_root(), CameraBase)    as CameraBase
	if _build_grid:
		_build_grid.mode_changed.connect(_on_mode_changed)
		_build_grid.terrain_pending_changed.connect(_on_terrain_pending_changed)


func _process(delta: float) -> void:
	if not _day_night:
		return
	var t := _day_night.time_of_day
	if _prev_time >= 0.0 and _prev_time > 0.9 and t < 0.1:
		_day_count += 1
	_prev_time = t
	var hours:   int = int(t * 24.0)
	var minutes: int = int(fmod(t * 24.0 * 60.0, 60.0))
	if _time_lbl:
		_time_lbl.text = "%02d:%02d" % [hours, minutes]
	if _day_lbl:
		_day_lbl.text = "Day %d" % _day_count
	if _pop_btn and _colony_manager:
		var count := _colony_manager.get_humans().size()
		_pop_btn.text = "%d citizen%s" % [count, "s" if count != 1 else ""]
	if _colony_panel and _colony_panel.visible:
		_colony_refresh_timer -= delta
		if _colony_refresh_timer <= 0.0:
			_colony_refresh_timer = 1.0
			_rebuild_colony_list()


# ── UI construction ───────────────────────────────────────────────────────────

func _build_ui() -> void:
	_build_top_bar()
	_build_bottom_bar()
	_build_colony_panel()
	_build_build_submenu()
	_build_terrain_panel()


func _build_top_bar() -> void:
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_WIDE)
	panel.offset_top    = 8
	panel.offset_bottom = 8
	panel.offset_left   = 0
	panel.offset_right  = 0
	panel.mouse_filter  = Control.MOUSE_FILTER_IGNORE
	panel.self_modulate = Color(0, 0, 0, 0)   # invisible panel, no background
	add_child(panel)

	# Centered container
	var center := CenterContainer.new()
	center.set_anchors_preset(Control.PRESET_TOP_WIDE)
	center.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	center.mouse_filter = Control.MOUSE_FILTER_IGNORE
	panel.add_child(center)

	var bg := PanelContainer.new()
	bg.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var style := StyleBoxFlat.new()
	style.bg_color         = Color(0.05, 0.05, 0.08, 0.72)
	style.corner_radius_top_left     = 8
	style.corner_radius_top_right    = 8
	style.corner_radius_bottom_left  = 8
	style.corner_radius_bottom_right = 8
	style.content_margin_left   = 16
	style.content_margin_right  = 16
	style.content_margin_top    = 6
	style.content_margin_bottom = 6
	bg.add_theme_stylebox_override("panel", style)
	center.add_child(bg)

	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 10)
	hbox.mouse_filter = Control.MOUSE_FILTER_IGNORE
	bg.add_child(hbox)

	_day_lbl = Label.new()
	_day_lbl.text = "Day 1"
	_day_lbl.add_theme_font_size_override("font_size", 14)
	_day_lbl.add_theme_color_override("font_color", Color(0.85, 0.80, 0.60))
	_day_lbl.mouse_filter = Control.MOUSE_FILTER_IGNORE
	hbox.add_child(_day_lbl)

	var sep := Label.new()
	sep.text = "·"
	sep.add_theme_font_size_override("font_size", 14)
	sep.add_theme_color_override("font_color", Color(0.5, 0.5, 0.5))
	sep.mouse_filter = Control.MOUSE_FILTER_IGNORE
	hbox.add_child(sep)

	_time_lbl = Label.new()
	_time_lbl.text = "00:00"
	_time_lbl.add_theme_font_size_override("font_size", 14)
	_time_lbl.add_theme_color_override("font_color", Color(0.95, 0.95, 0.95))
	_time_lbl.mouse_filter = Control.MOUSE_FILTER_IGNORE
	hbox.add_child(_time_lbl)

	var sep2 := Label.new()
	sep2.text = "·"
	sep2.add_theme_font_size_override("font_size", 14)
	sep2.add_theme_color_override("font_color", Color(0.5, 0.5, 0.5))
	sep2.mouse_filter = Control.MOUSE_FILTER_IGNORE
	hbox.add_child(sep2)

	_pop_btn = Button.new()
	_pop_btn.text = "0 citizens"
	_pop_btn.flat = true
	_pop_btn.add_theme_font_size_override("font_size", 14)
	_pop_btn.add_theme_color_override("font_color",       Color(0.70, 0.88, 0.70))
	_pop_btn.add_theme_color_override("font_hover_color", Color(1.00, 1.00, 1.00))
	_pop_btn.pressed.connect(_toggle_colony_panel)
	hbox.add_child(_pop_btn)



func _build_bottom_bar() -> void:
	var center := CenterContainer.new()
	center.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	center.offset_top    = -80
	center.offset_bottom = -12
	center.mouse_filter  = Control.MOUSE_FILTER_IGNORE
	add_child(center)

	var bg := PanelContainer.new()
	bg.mouse_filter = Control.MOUSE_FILTER_PASS
	var style := StyleBoxFlat.new()
	style.bg_color         = Color(0.05, 0.05, 0.08, 0.82)
	style.corner_radius_top_left     = 10
	style.corner_radius_top_right    = 10
	style.corner_radius_bottom_left  = 10
	style.corner_radius_bottom_right = 10
	style.content_margin_left   = 12
	style.content_margin_right  = 12
	style.content_margin_top    = 8
	style.content_margin_bottom = 8
	bg.add_theme_stylebox_override("panel", style)
	center.add_child(bg)

	var hbox := HBoxContainer.new()
	hbox.add_theme_constant_override("separation", 8)
	bg.add_child(hbox)

	_build_btn = _make_toolbar_btn("Buildings\n[B]", Color(0.30, 0.62, 0.30))
	_build_btn.pressed.connect(_on_build_btn_pressed)
	hbox.add_child(_build_btn)

	_road_btn = _make_toolbar_btn("Roads\n[R]", Color(0.45, 0.38, 0.28))
	_road_btn.pressed.connect(_on_road_btn_pressed)
	hbox.add_child(_road_btn)

	hbox.add_child(_make_toolbar_btn("Farms\n[—]",      Color(0.55, 0.55, 0.20), true))
	hbox.add_child(_make_toolbar_btn("Industry\n[—]",   Color(0.50, 0.32, 0.18), true))
	hbox.add_child(_make_toolbar_btn("Services\n[—]",   Color(0.25, 0.42, 0.62), true))


func _build_colony_panel() -> void:
	var style := StyleBoxFlat.new()
	style.bg_color                   = Color(0.05, 0.05, 0.08, 0.92)
	style.corner_radius_top_left     = 8
	style.corner_radius_top_right    = 8
	style.corner_radius_bottom_left  = 8
	style.corner_radius_bottom_right = 8
	style.content_margin_left        = 14
	style.content_margin_right       = 14
	style.content_margin_top         = 10
	style.content_margin_bottom      = 10

	_colony_panel = PanelContainer.new()
	_colony_panel.add_theme_stylebox_override("panel", style)
	_colony_panel.custom_minimum_size = Vector2(260, 0)
	_colony_panel.visible = false
	add_child(_colony_panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	_colony_panel.add_child(vbox)

	var header := Label.new()
	header.text = "Citizens"
	header.add_theme_font_size_override("font_size", 15)
	header.add_theme_color_override("font_color", Color(0.85, 0.80, 0.60))
	header.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(header)

	vbox.add_child(HSeparator.new())

	_colony_list = VBoxContainer.new()
	_colony_list.add_theme_constant_override("separation", 4)
	vbox.add_child(_colony_list)


func _build_build_submenu() -> void:
	var style := StyleBoxFlat.new()
	style.bg_color                   = Color(0.05, 0.05, 0.08, 0.95)
	style.corner_radius_top_left     = 8
	style.corner_radius_top_right    = 8
	style.corner_radius_bottom_left  = 8
	style.corner_radius_bottom_right = 8
	style.content_margin_left        = 8
	style.content_margin_right       = 8
	style.content_margin_top         = 8
	style.content_margin_bottom      = 8
	style.border_width_left   = 1
	style.border_width_right  = 1
	style.border_width_top    = 1
	style.border_width_bottom = 1
	style.border_color        = Color(0.30, 0.62, 0.30, 0.5)

	_build_submenu = PanelContainer.new()
	_build_submenu.add_theme_stylebox_override("panel", style)
	_build_submenu.visible = false
	add_child(_build_submenu)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 4)
	_build_submenu.add_child(vbox)

	var header := Label.new()
	header.text = "Buildings"
	header.add_theme_font_size_override("font_size", 12)
	header.add_theme_color_override("font_color", Color(0.65, 0.65, 0.65))
	header.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(header)

	vbox.add_child(HSeparator.new())

	var plot_btn := _make_submenu_item_btn("Open Plot", Color(0.30, 0.62, 0.30))
	plot_btn.pressed.connect(_on_build_open_plot)
	vbox.add_child(plot_btn)

	var terrain_btn := _make_submenu_item_btn("Open Terrain", Color(0.45, 0.62, 0.30))
	terrain_btn.pressed.connect(_on_build_open_terrain)
	vbox.add_child(terrain_btn)


func _make_submenu_item_btn(label: String, accent: Color) -> Button:
	var btn := Button.new()
	btn.text                = label
	btn.custom_minimum_size = Vector2(150, 34)
	btn.add_theme_font_size_override("font_size", 12)
	btn.add_theme_color_override("font_color",       Color(0.95, 0.95, 0.95))
	btn.add_theme_color_override("font_hover_color", Color(1.00, 1.00, 1.00))

	var normal := StyleBoxFlat.new()
	normal.bg_color                  = Color(accent.r * 0.30, accent.g * 0.30, accent.b * 0.30, 0.80)
	normal.corner_radius_top_left    = 5
	normal.corner_radius_top_right   = 5
	normal.corner_radius_bottom_left = 5
	normal.corner_radius_bottom_right = 5
	normal.content_margin_left   = 10
	normal.content_margin_right  = 10
	normal.content_margin_top    = 4
	normal.content_margin_bottom = 4

	var hover := StyleBoxFlat.new()
	hover.bg_color                   = Color(accent.r * 0.55, accent.g * 0.55, accent.b * 0.55, 0.95)
	hover.corner_radius_top_left     = 5
	hover.corner_radius_top_right    = 5
	hover.corner_radius_bottom_left  = 5
	hover.corner_radius_bottom_right = 5
	hover.content_margin_left   = 10
	hover.content_margin_right  = 10
	hover.content_margin_top    = 4
	hover.content_margin_bottom = 4

	btn.add_theme_stylebox_override("normal", normal)
	btn.add_theme_stylebox_override("hover",  hover)
	btn.add_theme_stylebox_override("pressed", hover)
	return btn


func _build_terrain_panel() -> void:
	var style := StyleBoxFlat.new()
	style.bg_color                   = Color(0.05, 0.05, 0.08, 0.92)
	style.corner_radius_top_left     = 8
	style.corner_radius_top_right    = 8
	style.corner_radius_bottom_left  = 8
	style.corner_radius_bottom_right = 8
	style.border_width_left   = 1
	style.border_width_right  = 1
	style.border_width_top    = 1
	style.border_width_bottom = 1
	style.border_color        = Color(0.45, 0.62, 0.30, 0.5)
	style.content_margin_left   = 14
	style.content_margin_right  = 14
	style.content_margin_top    = 10
	style.content_margin_bottom = 10

	_terrain_panel = PanelContainer.new()
	_terrain_panel.add_theme_stylebox_override("panel", style)
	_terrain_panel.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	_terrain_panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	_terrain_panel.grow_vertical   = Control.GROW_DIRECTION_BEGIN
	_terrain_panel.offset_right    = -12
	_terrain_panel.offset_bottom   = -92
	_terrain_panel.visible         = false
	add_child(_terrain_panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	_terrain_panel.add_child(vbox)

	var title := Label.new()
	title.text = "Terrain Definition"
	title.add_theme_font_size_override("font_size", 13)
	title.add_theme_color_override("font_color", Color(0.85, 0.80, 0.60))
	title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	vbox.add_child(title)

	vbox.add_child(HSeparator.new())

	_terrain_status_lbl = Label.new()
	_terrain_status_lbl.text = "Drag to draw areas on the terrain"
	_terrain_status_lbl.add_theme_font_size_override("font_size", 11)
	_terrain_status_lbl.add_theme_color_override("font_color", Color(0.65, 0.65, 0.65))
	_terrain_status_lbl.custom_minimum_size = Vector2(190, 0)
	_terrain_status_lbl.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	vbox.add_child(_terrain_status_lbl)

	_terrain_accept_btn = _make_submenu_item_btn("Accept Terrain", Color(0.30, 0.72, 0.30))
	_terrain_accept_btn.disabled = true
	_terrain_accept_btn.pressed.connect(_on_terrain_accept_pressed)
	vbox.add_child(_terrain_accept_btn)

	var cancel_btn := _make_submenu_item_btn("Cancel", Color(0.55, 0.25, 0.25))
	cancel_btn.pressed.connect(func() -> void:
		if _build_grid:
			_build_grid.set_mode(BuildGrid.Mode.NONE)
	)
	vbox.add_child(cancel_btn)


func _position_build_submenu() -> void:
	if not _build_submenu or not _build_btn:
		return
	_build_submenu.reset_size()
	var btn_rect := _build_btn.get_global_rect()
	var sub_size := _build_submenu.size
	_build_submenu.position = Vector2(
		btn_rect.position.x + (btn_rect.size.x - sub_size.x) * 0.5,
		btn_rect.position.y - sub_size.y - 8.0
	)


func _toggle_colony_panel() -> void:
	if not _colony_panel:
		return
	_colony_panel.visible = not _colony_panel.visible
	if _colony_panel.visible:
		_colony_refresh_timer = 0.0
		_rebuild_colony_list()


func _rebuild_colony_list() -> void:
	for child in _colony_list.get_children():
		child.queue_free()

	if not _colony_manager:
		return

	var state_names  := ["Idle", "Walking", "Working", "Eating", "Sleeping"]
	var state_colors := [
		Color(0.55, 0.80, 0.55),
		Color(0.55, 0.70, 0.95),
		Color(0.95, 0.80, 0.30),
		Color(0.95, 0.55, 0.25),
		Color(0.70, 0.70, 0.95),
	]

	for h: Human in _colony_manager.get_humans():
		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 8)
		_colony_list.add_child(row)

		var name_btn := Button.new()
		name_btn.text = h.human_name
		name_btn.flat = true
		name_btn.alignment = HORIZONTAL_ALIGNMENT_LEFT
		name_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		name_btn.add_theme_font_size_override("font_size", 13)
		name_btn.add_theme_color_override("font_color",       Color(0.95, 0.95, 0.95))
		name_btn.add_theme_color_override("font_hover_color", Color(0.55, 0.85, 1.00))
		var captured := h
		name_btn.pressed.connect(func() -> void:
			if _camera_base and is_instance_valid(captured):
				_camera_base.pan_to(captured.global_position)
		)
		row.add_child(name_btn)

		var state_lbl := Label.new()
		state_lbl.text = state_names[h.state]
		state_lbl.add_theme_font_size_override("font_size", 13)
		state_lbl.add_theme_color_override("font_color", state_colors[h.state])
		row.add_child(state_lbl)

	# Reposition centered below the top bar each time content changes
	await get_tree().process_frame
	if is_instance_valid(_colony_panel):
		_colony_panel.reset_size()
		var vp := get_viewport().get_visible_rect().size
		_colony_panel.position = Vector2(
			(vp.x - _colony_panel.size.x) * 0.5,
			48.0
		)


func _make_toolbar_btn(label: String, accent: Color, disabled: bool = false) -> Button:
	var btn := Button.new()
	btn.text                = label
	btn.custom_minimum_size = Vector2(72, 52)
	btn.toggle_mode         = true
	btn.disabled            = disabled
	btn.add_theme_font_size_override("font_size", 11)

	var normal := StyleBoxFlat.new()
	normal.bg_color         = Color(accent.r * 0.35, accent.g * 0.35, accent.b * 0.35, 0.85)
	normal.corner_radius_top_left     = 6
	normal.corner_radius_top_right    = 6
	normal.corner_radius_bottom_left  = 6
	normal.corner_radius_bottom_right = 6
	normal.content_margin_left   = 6
	normal.content_margin_right  = 6
	normal.content_margin_top    = 4
	normal.content_margin_bottom = 4

	var pressed_style := StyleBoxFlat.new()
	pressed_style.bg_color         = Color(accent.r * 0.7, accent.g * 0.7, accent.b * 0.7, 1.0)
	pressed_style.corner_radius_top_left     = 6
	pressed_style.corner_radius_top_right    = 6
	pressed_style.corner_radius_bottom_left  = 6
	pressed_style.corner_radius_bottom_right = 6
	pressed_style.border_width_bottom = 2
	pressed_style.border_width_top    = 2
	pressed_style.border_width_left   = 2
	pressed_style.border_width_right  = 2
	pressed_style.border_color        = accent.lightened(0.3)
	pressed_style.content_margin_left   = 6
	pressed_style.content_margin_right  = 6
	pressed_style.content_margin_top    = 4
	pressed_style.content_margin_bottom = 4

	var hover_style := StyleBoxFlat.new()
	hover_style.bg_color         = Color(accent.r * 0.5, accent.g * 0.5, accent.b * 0.5, 0.95)
	hover_style.corner_radius_top_left     = 6
	hover_style.corner_radius_top_right    = 6
	hover_style.corner_radius_bottom_left  = 6
	hover_style.corner_radius_bottom_right = 6
	hover_style.content_margin_left   = 6
	hover_style.content_margin_right  = 6
	hover_style.content_margin_top    = 4
	hover_style.content_margin_bottom = 4

	btn.add_theme_stylebox_override("normal",        normal)
	btn.add_theme_stylebox_override("pressed",       pressed_style)
	btn.add_theme_stylebox_override("hover",         hover_style)
	btn.add_theme_stylebox_override("hover_pressed", pressed_style)

	if disabled:
		btn.self_modulate = Color(1, 1, 1, 0.4)

	return btn


# ── Callbacks ─────────────────────────────────────────────────────────────────

func _on_build_btn_pressed() -> void:
	# toggle_mode updates button_pressed before pressed fires
	if not _build_btn.button_pressed:
		# User clicked to deactivate
		if _build_submenu:
			_build_submenu.visible = false
		if _build_grid:
			_build_grid.set_mode(BuildGrid.Mode.NONE)
		return
	# User clicked to open submenu
	if _build_submenu:
		_build_submenu.visible = true
		await get_tree().process_frame
		if is_instance_valid(_build_submenu) and _build_submenu.visible:
			_position_build_submenu()


func _on_build_open_plot() -> void:
	if _build_submenu:
		_build_submenu.visible = false
	if _build_grid:
		_build_grid.set_mode(BuildGrid.Mode.BUILD)


func _on_build_open_terrain() -> void:
	if _build_submenu:
		_build_submenu.visible = false
	if _build_grid:
		_build_grid.set_mode(BuildGrid.Mode.TERRAIN)


func _on_road_btn_pressed() -> void:
	if _build_grid:
		var m := BuildGrid.Mode.NONE if _build_grid.get_mode() == BuildGrid.Mode.ROAD \
			   else BuildGrid.Mode.ROAD
		_build_grid.set_mode(m)


func _on_mode_changed(mode: int) -> void:
	var in_build    := mode == BuildGrid.Mode.BUILD or mode == BuildGrid.Mode.TERRAIN
	var in_terrain  := mode == BuildGrid.Mode.TERRAIN
	if _build_btn:
		_build_btn.button_pressed = in_build
	if not in_build and _build_submenu:
		_build_submenu.visible = false
	if _terrain_panel:
		_terrain_panel.visible = in_terrain
	if not in_terrain and _terrain_accept_btn:
		_terrain_accept_btn.disabled = true
	if _road_btn:
		_road_btn.button_pressed = (mode == BuildGrid.Mode.ROAD)


func _on_terrain_pending_changed(has_cells: bool, is_connected: bool) -> void:
	if not _terrain_accept_btn or not _terrain_status_lbl:
		return
	if not has_cells:
		_terrain_status_lbl.text = "Drag to draw areas on the terrain"
		_terrain_status_lbl.add_theme_color_override("font_color", Color(0.65, 0.65, 0.65))
		_terrain_accept_btn.disabled = true
	elif is_connected:
		_terrain_status_lbl.text = "Areas connected — ready to accept"
		_terrain_status_lbl.add_theme_color_override("font_color", Color(0.50, 0.88, 0.50))
		_terrain_accept_btn.disabled = false
	else:
		_terrain_status_lbl.text = "Areas are disconnected — connect them to accept"
		_terrain_status_lbl.add_theme_color_override("font_color", Color(0.95, 0.45, 0.30))
		_terrain_accept_btn.disabled = true


func _on_terrain_accept_pressed() -> void:
	if _build_grid:
		_build_grid.accept_terrain()


# ── Helpers ───────────────────────────────────────────────────────────────────

func _find_typed(node: Node, type) -> Node:
	if is_instance_of(node, type):
		return node
	for child in node.get_children():
		var found := _find_typed(child, type)
		if found:
			return found
	return null
