extends PanelContainer

signal deselected
signal area_select_requested
signal remove_requested
signal cut_mark_toggled
signal rotate_requested
signal terrain_linked(zone: Dictionary)
signal terrain_unlinked

@onready var _header:        Label         = $Margin/VBox/Header
@onready var _info_box:      VBoxContainer = $Margin/VBox/InfoBox
@onready var _sep2:          HSeparator    = $Margin/VBox/Sep2
@onready var _mark_cut_btn:  Button        = $Margin/VBox/MarkCutBtn
@onready var _remove_btn:    Button        = $Margin/VBox/RemoveBtn
@onready var _area_btn:      Button        = $Margin/VBox/AreaBtn

var _last_prop:    Node3D      = null
var _last_terrain: TerrainMesh = null
var _last_area:    AreaData    = null


func _ready() -> void:
	Debug.toggled.connect(_on_debug_toggled)


func _on_debug_toggled(dbg: bool) -> void:
	_remove_btn.visible = dbg and visible


func show_for(prop: Node3D, terrain: TerrainMesh, area: AreaData = null) -> void:
	_last_prop    = prop
	_last_terrain = terrain
	_last_area    = area
	# Header
	var prop_layer := prop.get_parent() as PropLayer
	if prop_layer:
		var label: String
		if prop_layer.display_name != "":
			label = prop_layer.display_name
		elif prop_layer.prop_scene:
			label = prop_layer.prop_scene.resource_path.get_file().get_basename().capitalize()
		elif not prop_layer.prop_scenes.is_empty():
			label = prop_layer.prop_scenes[0].resource_path.get_file().get_basename().capitalize()
		else:
			label = prop.name
		_header.text = label + "  #" + str(prop.get_index() + 1)
	else:
		_header.text = prop.name

	# Clear previous rows
	for child in _info_box.get_children():
		child.queue_free()

	var pos := prop.global_position
	_add_row("Position", "(%d,  %d)" % [int(pos.x), int(pos.z)])

	if terrain:
		var biome: int = terrain.get_biome(pos.x, pos.z)
		var keys  := TerrainMesh.Biome.keys()
		var label: String = keys[biome].capitalize() if biome < keys.size() else "Unknown"
		_add_row("Biome", label)

	if prop.has_method("get_selection_info"):
		var extra: Dictionary = prop.call("get_selection_info")
		for key: String in extra:
			_add_row(key, str(extra[key]))

	if prop is EmptyPlot and not (prop as EmptyPlot).is_requested():
		_add_applicant_section(prop as EmptyPlot)

	_sep2.visible = _info_box.get_child_count() > 0

	if prop.has_method("toggle_cut_mark"):
		var marked: bool = prop.get("marked_for_cutting")
		_mark_cut_btn.text = "Cancel Cutting" if marked else "Mark for Cutting"
		_mark_cut_btn.visible = true
	else:
		_mark_cut_btn.visible = false

	_remove_btn.visible = Debug.enabled

	if area:
		_area_btn.text    = "View " + area.area_name
		_area_btn.visible = true
	else:
		_area_btn.visible = false

	visible = true


func clear() -> void:
	for child in _info_box.get_children():
		child.queue_free()
	_header.text         = ""
	_mark_cut_btn.visible = false
	_remove_btn.visible  = false
	_area_btn.visible    = false
	visible              = false


func _on_mark_cut_btn_pressed() -> void:
	emit_signal("cut_mark_toggled")


func _on_remove_btn_pressed() -> void:
	emit_signal("remove_requested")


func _on_area_btn_pressed() -> void:
	emit_signal("area_select_requested")


func _on_deselect_pressed() -> void:
	emit_signal("deselected")


# ── Helpers ───────────────────────────────────────────────────────────────────

func add_plot_controls(plot: EmptyPlot, adjacent_zones: Array) -> void:
	_info_box.add_child(HSeparator.new())

	# Terrain link section
	_info_box.add_child(HSeparator.new())

	var link_header := Label.new()
	link_header.text = "Link Terrain"
	link_header.add_theme_font_size_override("font_size", 13)
	link_header.add_theme_color_override("font_color", Color(0.72, 0.88, 0.60))
	_info_box.add_child(link_header)

	if not plot.linked_zone.is_empty():
		var linked_row := HBoxContainer.new()
		linked_row.add_theme_constant_override("separation", 6)
		_info_box.add_child(linked_row)

		var linked_lbl := Label.new()
		linked_lbl.text = plot.linked_zone.get("label", "Zone")
		linked_lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		linked_lbl.add_theme_font_size_override("font_size", 12)
		linked_lbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
		linked_row.add_child(linked_lbl)

		var unlink_btn := Button.new()
		unlink_btn.text = "Unlink"
		unlink_btn.add_theme_font_size_override("font_size", 11)
		unlink_btn.add_theme_color_override("font_color", Color(1.0, 0.55, 0.55))
		unlink_btn.pressed.connect(func() -> void: emit_signal("terrain_unlinked"))
		linked_row.add_child(unlink_btn)
	elif adjacent_zones.is_empty():
		var no_zone_lbl := Label.new()
		no_zone_lbl.text = "No adjacent terrain zones"
		no_zone_lbl.add_theme_font_size_override("font_size", 12)
		no_zone_lbl.add_theme_color_override("font_color", Color(0.55, 0.55, 0.55))
		_info_box.add_child(no_zone_lbl)
	else:
		for zone: Dictionary in adjacent_zones:
			var zone_row := HBoxContainer.new()
			zone_row.add_theme_constant_override("separation", 6)
			_info_box.add_child(zone_row)

			var zone_lbl := Label.new()
			zone_lbl.text = zone.get("label", "Zone")
			zone_lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			zone_lbl.add_theme_font_size_override("font_size", 12)
			zone_lbl.add_theme_color_override("font_color", Color(0.85, 0.85, 0.85))
			zone_row.add_child(zone_lbl)

			var link_btn := Button.new()
			link_btn.text = "Link"
			link_btn.add_theme_font_size_override("font_size", 11)
			link_btn.add_theme_color_override("font_color", Color(0.50, 1.00, 0.50))
			var captured := zone
			link_btn.pressed.connect(func() -> void: emit_signal("terrain_linked", captured))
			zone_row.add_child(link_btn)


func _add_applicant_section(plot: EmptyPlot) -> void:
	var apps := plot.get_applicants()
	if apps.is_empty():
		return

	var sep := HSeparator.new()
	_info_box.add_child(sep)

	var header := Label.new()
	header.text = "Applicants"
	header.add_theme_font_size_override("font_size", 13)
	header.add_theme_color_override("font_color", Color(0.65, 0.82, 1.0))
	_info_box.add_child(header)

	for a: Dictionary in apps:
		var human: Human = a.human
		var p: String    = a.purpose
		if not is_instance_valid(human):
			continue

		var row := HBoxContainer.new()
		row.add_theme_constant_override("separation", 6)
		_info_box.add_child(row)

		var lbl := Label.new()
		lbl.text = "%s  (%s)" % [human.human_name, p]
		lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		lbl.add_theme_font_size_override("font_size", 13)
		row.add_child(lbl)

		var accept_btn := Button.new()
		accept_btn.text = "Accept"
		accept_btn.add_theme_font_size_override("font_size", 11)
		accept_btn.add_theme_color_override("font_color", Color(0.5, 1.0, 0.5))
		var captured := human
		accept_btn.pressed.connect(func() -> void:
			plot.accept_applicant(captured)
			if _last_prop and is_instance_valid(_last_prop):
				show_for(_last_prop, _last_terrain, _last_area)
		)
		row.add_child(accept_btn)

		var decline_btn := Button.new()
		decline_btn.text = "Decline"
		decline_btn.add_theme_font_size_override("font_size", 11)
		decline_btn.add_theme_color_override("font_color", Color(1.0, 0.55, 0.55))
		decline_btn.pressed.connect(func() -> void:
			plot.remove_applicant(captured)
			if _last_prop and is_instance_valid(_last_prop):
				show_for(_last_prop, _last_terrain, _last_area)
		)
		row.add_child(decline_btn)


func _add_row(key: String, value: String) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 6)

	var k := Label.new()
	k.text = key + ":"
	k.custom_minimum_size = Vector2(80, 0)
	k.add_theme_font_size_override("font_size", 13)
	k.self_modulate = Color(0.75, 0.75, 0.75)

	var v := Label.new()
	v.text = value
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	v.add_theme_font_size_override("font_size", 13)

	row.add_child(k)
	row.add_child(v)
	_info_box.add_child(row)
