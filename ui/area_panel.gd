extends PanelContainer

signal closed
signal district_requested
signal name_changed(new_name: String)

@onready var _name_edit: LineEdit      = $Margin/VBox/NameEdit
@onready var _type_val:  Label         = $Margin/VBox/Rows/TypeRow/Val
@onready var _size_val:  Label         = $Margin/VBox/Rows/SizeRow/Val
@onready var _props_val: Label         = $Margin/VBox/Rows/PropsRow/Val
@onready var _district_btn: Button     = $Margin/VBox/DistrictBtn
@onready var _color_rect: ColorRect    = $Margin/VBox/ColorBar


func show_for(area: AreaData) -> void:
	_name_edit.text       = area.area_name
	_type_val.text        = AreaData.TYPE_LABEL.get(area.area_type, "Unknown")
	_size_val.text        = str(area.cell_count) + " cells"
	_color_rect.color     = area.color
	_color_rect.color.a   = 1.0
	_district_btn.visible = area.area_type != AreaData.Type.DISTRICT

	if area.prop_types.is_empty():
		_props_val.text = str(area.prop_count)
	else:
		var lines: Array = []
		for t in area.prop_types:
			lines.append(str(t) + ": " + str(area.prop_types[t]))
		_props_val.text = "\n".join(lines)

	visible = true


func _on_close_pressed() -> void:
	emit_signal("closed")


func _on_district_pressed() -> void:
	emit_signal("district_requested")


func _on_name_submitted(text: String) -> void:
	emit_signal("name_changed", text)


func _on_name_focus_exited() -> void:
	emit_signal("name_changed", _name_edit.text)
