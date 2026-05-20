extends Control

const TARGET_SCENE := "res://scenes/terrain_generation.tscn"

@onready var progress_bar: ProgressBar = $CenterContainer/VBoxContainer/ProgressBar
@onready var status_label: Label       = $CenterContainer/VBoxContainer/Status

var _dot_timer: float = 0.0
var _dot_count: int   = 0


func _ready() -> void:
	ResourceLoader.load_threaded_request(TARGET_SCENE)


func _process(delta: float) -> void:
	_dot_timer += delta
	if _dot_timer >= 0.4:
		_dot_timer  = 0.0
		_dot_count  = (_dot_count + 1) % 4
		status_label.text = "Loading" + ".".repeat(_dot_count)

	var progress := []
	var status   := ResourceLoader.load_threaded_get_status(TARGET_SCENE, progress)

	match status:
		ResourceLoader.THREAD_LOAD_IN_PROGRESS:
			progress_bar.value = progress[0] * 100.0
		ResourceLoader.THREAD_LOAD_LOADED:
			progress_bar.value = 100.0
			var packed: PackedScene = ResourceLoader.load_threaded_get(TARGET_SCENE)
			get_tree().change_scene_to_packed(packed)
		ResourceLoader.THREAD_LOAD_FAILED:
			status_label.text = "Failed to load scene!"
