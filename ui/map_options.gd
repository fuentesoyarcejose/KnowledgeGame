extends Control

const HEIGHT_PRESETS: Array[float] = [20.0, 40.0, 64.0, 90.0, 128.0]
const HEIGHT_LABELS:  Array[String] = ["Flat", "Low", "Normal", "High", "Mountainous"]

@onready var _seed_input:    LineEdit = $CenterContainer/VBox/SeedRow/SeedInput
@onready var _rivers_slider: HSlider  = $CenterContainer/VBox/RiversRow/RiversSlider
@onready var _rivers_value:  Label    = $CenterContainer/VBox/RiversRow/RiversValue
@onready var _height_slider: HSlider  = $CenterContainer/VBox/HeightRow/HeightSlider
@onready var _height_value:  Label    = $CenterContainer/VBox/HeightRow/HeightValue


func _ready() -> void:
	_seed_input.text = str(randi())
	_rivers_value.text = str(int(_rivers_slider.value))
	_height_value.text = HEIGHT_LABELS[int(_height_slider.value)]


func _on_random_pressed() -> void:
	_seed_input.text = str(randi())


func _on_rivers_value_changed(value: float) -> void:
	_rivers_value.text = str(int(value))


func _on_height_value_changed(value: float) -> void:
	_height_value.text = HEIGHT_LABELS[int(value)]


func _on_back_pressed() -> void:
	get_tree().change_scene_to_file("res://ui/main_menu.tscn")


func _on_generate_pressed() -> void:
	var raw := _seed_input.text.strip_edges()
	var map_seed := raw.to_int() if raw.is_valid_int() else randi()
	get_tree().set_meta("map_seed",        map_seed)
	get_tree().set_meta("map_river_count", int(_rivers_slider.value))
	get_tree().set_meta("map_height",      HEIGHT_PRESETS[int(_height_slider.value)])
	get_tree().change_scene_to_file("res://ui/loading_screen.tscn")
