extends Node3D

@export_range(0,100,1) var cameraMoveSpeed: float = 20.0

var cameraZoomDirection: float = 0.0
@export_range(0,100,1) var cameraZoomSpeed: float = 40.0
@export_range(0,100,1) var cameraZoomMin: float = 10.0
@export_range(0,100,1) var cameraZoomMax: float = 100.0
@export_range(0,2,1) var cameraZoomSpeedDamp: float = 0.9


#Flags
var cameraCanProcess: bool = true
var cameraCanMoveBase: bool = true
var cameraCanZoom: bool = true


#Nodes
@onready var cameraSocket: Node3D = $CameraSocket
@onready var camera: Camera3D = $CameraSocket/Camera3D




# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if !cameraCanProcess:
		return
	cameraBaseMovement(delta)
	cameraZoomUpdate(delta)


func _unhandled_input(event: InputEvent) -> void:
	# camera Zoom
	if event.is_action("CameraZoomIn"):
		cameraZoomDirection = -1.0
	if event.is_action("CameraZoomOut"):
		cameraZoomDirection = 1.0

func cameraBaseMovement(delta: float) -> void:
	if !cameraCanMoveBase:
		return
	var velocityDirection: Vector3 = Vector3.ZERO

	if Input.is_action_pressed("CameraForward"): velocityDirection -= transform.basis.z
	if Input.is_action_pressed("CameraBackward"): velocityDirection += transform.basis.z
	if Input.is_action_pressed("CameraLeft"): velocityDirection -= transform.basis.x
	if Input.is_action_pressed("CameraRight"): velocityDirection += transform.basis.x

	position += velocityDirection.normalized() * cameraMoveSpeed * delta

func cameraZoomUpdate(delta:float) -> void:
	if !cameraCanZoom:
		return
	var zoomDir: Vector3 = camera.global_transform.basis.z
	position += zoomDir * cameraZoomSpeed * cameraZoomDirection * delta
	cameraZoomDirection *= cameraZoomSpeedDamp




