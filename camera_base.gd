extends Node3D

# ── Movement ──────────────────────────────────────────────────────────────────
@export_range(1.0, 200.0, 1.0) var cameraMoveSpeed: float = 100.0
@export_range(1.0, 360.0, 1.0) var cameraRotateSpeed: float = 90.0

# ── Zoom ──────────────────────────────────────────────────────────────────────
@export_range(1.0, 50.0, 0.5)   var arm_min: float = 5.0
@export_range(10.0, 500.0, 1.0) var arm_max: float = 500.0
# Fraction of current distance added/removed per scroll tick (logarithmic feel)
@export_range(0.05, 0.5, 0.01)  var zoom_factor: float = 0.15
# How fast the arm catches up to the target (higher = snappier)
@export_range(1.0, 30.0, 0.5)   var zoom_smooth: float = 12.0

# ── Pitch ─────────────────────────────────────────────────────────────────────
@export_range(-90.0, 0.0, 1.0) var default_pitch: float = -45.0

# ── Mouse rotation ────────────────────────────────────────────────────────────
@export_range(0.01, 2.0, 0.01) var mouse_rotate_sensitivity: float = 0.3

# ── Flags ─────────────────────────────────────────────────────────────────────
var cameraCanProcess:  bool = true
var cameraCanMoveBase: bool = true
var cameraCanZoom:     bool = true

# ── Nodes ─────────────────────────────────────────────────────────────────────
@onready var cameraSocket: Node3D   = $CameraSocket
@onready var camera:       Camera3D = $CameraSocket/Camera3D

# ── State ─────────────────────────────────────────────────────────────────────
var _target_arm:   float = 0.0
var _current_arm:  float = 0.0
var _mmb_held:     bool  = false
var _current_pitch: float = 0.0
var _terrain: TerrainMesh = null


func _ready() -> void:
	_terrain       = get_parent().get_node_or_null("TerrainMesh") as TerrainMesh
	_current_pitch = default_pitch
	_target_arm    = sqrt(arm_min * arm_max)  # geometric midpoint
	_current_arm   = _target_arm
	_apply_zoom()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed:
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP:
				_target_arm = clamp(_target_arm * (1.0 - zoom_factor), arm_min, arm_max)
			MOUSE_BUTTON_WHEEL_DOWN:
				_target_arm = clamp(_target_arm * (1.0 + zoom_factor), arm_min, arm_max)
			MOUSE_BUTTON_MIDDLE:
				_mmb_held = true
	if event is InputEventMouseButton and not event.pressed:
		if event.button_index == MOUSE_BUTTON_MIDDLE:
			_mmb_held = false

	if event is InputEventMouseMotion and _mmb_held:
		rotation_degrees.y -= event.relative.x * mouse_rotate_sensitivity
		_current_pitch = clamp(
			_current_pitch - event.relative.y * mouse_rotate_sensitivity,
			-90.0, 0.0)


func _process(delta: float) -> void:
	if not cameraCanProcess:
		return
	_camera_pan(delta)
	_camera_rotate(delta)
	_camera_zoom(delta)


func _camera_pan(delta: float) -> void:
	if not cameraCanMoveBase:
		return
	var dir := Vector3.ZERO
	if Input.is_action_pressed("CameraForward"):  dir -= transform.basis.z
	if Input.is_action_pressed("CameraBackward"): dir += transform.basis.z
	if Input.is_action_pressed("CameraLeft"):     dir -= transform.basis.x
	if Input.is_action_pressed("CameraRight"):    dir += transform.basis.x
	if dir.length_squared() > 0.0:
		var speed := cameraMoveSpeed * (2.0 if Input.is_key_pressed(KEY_SHIFT) else 1.0)
		position += dir.normalized() * speed * delta
	if _terrain:
		var play_half := (_terrain.terrain_size - _terrain.play_area_margin) * 0.5
		position.x = clamp(position.x, -play_half, play_half)
		position.z = clamp(position.z, -play_half, play_half)


func _camera_rotate(delta: float) -> void:
	if Input.is_action_pressed("CameraRotateLeft"):
		rotation_degrees.y += cameraRotateSpeed * delta
	if Input.is_action_pressed("CameraRotateRight"):
		rotation_degrees.y -= cameraRotateSpeed * delta


func _camera_zoom(delta: float) -> void:
	if not cameraCanZoom:
		return
	_current_arm = lerpf(_current_arm, _target_arm, zoom_smooth * delta)
	_apply_zoom()


func _apply_zoom() -> void:
	camera.position = Vector3(0.0, 0.0, _current_arm)
	cameraSocket.rotation_degrees.x = _current_pitch
