extends CharacterBody3D
class_name Human

signal needs_changed(human: Human)

const WALK_SPEED    := 4.0
const GRAVITY       := -20.0
const HUNGER_RATE   := 0.004   # per second
const ENERGY_RATE   := 0.003
const EAT_RATE      := 0.12
const SLEEP_RATE    := 0.08
const HUNGER_EAT    := 0.75    # start eating above this
const ENERGY_SLEEP  := 0.30    # start sleeping below this

enum State { IDLE, MOVE, WORK, EAT, SLEEP }

var human_name: String = ""
var hunger:  float = 0.0   # 0=full, 1=starving
var energy:  float = 1.0   # 0=exhausted, 1=full
var state:   State = State.IDLE

var _terrain:               TerrainMesh = null
var _target_pos:            Vector3     = Vector3.ZERO
var _has_target:            bool        = false
var _work_timer:            float       = 0.0
var _idle_timer:            float       = 0.0
var _pending_work_duration: float       = 0.0
var _on_work_complete:      Callable    = Callable()

@onready var _body_mesh:  MeshInstance3D = $Body
@onready var _head_mesh:  MeshInstance3D = $Head
@onready var _name_label: Label3D        = $NameLabel


func _ready() -> void:
	floor_snap_length = 0.5
	_idle_timer = randf_range(1.0, 4.0)


func setup(terrain: TerrainMesh, spawn_pos: Vector3, hname: String) -> void:
	_terrain    = terrain
	global_position = spawn_pos
	human_name  = hname
	if _name_label:
		_name_label.text = hname


func _physics_process(delta: float) -> void:
	_update_needs(delta)
	_update_state(delta)
	_apply_movement(delta)
	_update_visuals()


# ── Needs ──────────────────────────────────────────────────────────────────────

func _update_needs(delta: float) -> void:
	match state:
		State.SLEEP:
			energy  = minf(energy  + SLEEP_RATE * delta, 1.0)
			hunger  = minf(hunger  + HUNGER_RATE * delta * 0.3, 1.0)
		State.EAT:
			hunger  = maxf(hunger  - EAT_RATE   * delta, 0.0)
			energy  = minf(energy  + SLEEP_RATE * delta * 0.3, 1.0)
		_:
			hunger  = minf(hunger  + HUNGER_RATE * delta, 1.0)
			energy  = maxf(energy  - ENERGY_RATE * delta, 0.0)
	needs_changed.emit(self)


# ── State machine ──────────────────────────────────────────────────────────────

func _update_state(delta: float) -> void:
	match state:
		State.IDLE:    _tick_idle(delta)
		State.MOVE:    _tick_move()
		State.WORK:    _tick_work(delta)
		State.EAT:     _tick_eat(delta)
		State.SLEEP:   _tick_sleep(delta)


func _tick_idle(delta: float) -> void:
	if hunger >= HUNGER_EAT:
		_set_state(State.EAT)
		return
	if energy <= ENERGY_SLEEP:
		_set_state(State.SLEEP)
		return
	_idle_timer -= delta
	if _idle_timer <= 0.0:
		_wander()
		_idle_timer = randf_range(6.0, 14.0)


func _tick_move() -> void:
	if not _has_target:
		_arrive()
		return
	var flat_dist := Vector2(global_position.x - _target_pos.x,
	                         global_position.z - _target_pos.z).length()
	if flat_dist < 0.4:
		_has_target = false
		_arrive()


func _arrive() -> void:
	if _pending_work_duration > 0.0:
		_work_timer            = _pending_work_duration
		_pending_work_duration = 0.0
		state                  = State.WORK
	else:
		_set_state(State.IDLE)


func _tick_work(delta: float) -> void:
	_work_timer -= delta
	if _work_timer <= 0.0:
		if _on_work_complete.is_valid():
			_on_work_complete.call()
		_on_work_complete = Callable()
		_set_state(State.IDLE)
	elif hunger >= HUNGER_EAT or energy <= ENERGY_SLEEP:
		_on_work_complete = Callable()
		_set_state(State.IDLE)


func _tick_eat(delta: float) -> void:
	if hunger <= 0.05:
		_set_state(State.IDLE)


func _tick_sleep(delta: float) -> void:
	if energy >= 0.95:
		_set_state(State.IDLE)


func _set_state(s: State) -> void:
	state       = s
	_has_target = false


# ── Public API ─────────────────────────────────────────────────────────────────

func assign_move(target: Vector3) -> void:
	_target_pos            = _snap_to_ground(target)
	_has_target            = true
	_pending_work_duration = 0.0
	_on_work_complete      = Callable()
	state                  = State.MOVE


func assign_work(at: Vector3, duration: float, on_complete: Callable = Callable()) -> void:
	_target_pos            = _snap_to_ground(at)
	_has_target            = true
	_pending_work_duration = duration
	_on_work_complete      = on_complete
	state                  = State.MOVE


func is_available() -> bool:
	return state == State.IDLE and hunger < HUNGER_EAT and energy > ENERGY_SLEEP


func get_selection_info() -> Dictionary:
	var state_names := ["Idle", "Walking", "Working", "Eating", "Sleeping"]
	return {
		"Name":   human_name,
		"State":  state_names[state],
		"Hunger": "%d%%" % int(hunger * 100),
		"Energy": "%d%%" % int(energy * 100),
	}


# ── Movement ───────────────────────────────────────────────────────────────────

func _apply_movement(delta: float) -> void:
	var v := velocity
	if is_on_floor():
		v.y = 0.0
	else:
		v.y += GRAVITY * delta

	if _has_target and state == State.MOVE:
		var dir := Vector3(_target_pos.x - global_position.x,
		                   0.0,
		                   _target_pos.z - global_position.z)
		var len := dir.length()
		if len > 0.1:
			dir /= len
			v.x = dir.x * WALK_SPEED
			v.z = dir.z * WALK_SPEED
			look_at(global_position + Vector3(dir.x, 0, dir.z), Vector3.UP)
		else:
			v.x = 0.0
			v.z = 0.0
	else:
		v.x = move_toward(v.x, 0.0, WALK_SPEED)
		v.z = move_toward(v.z, 0.0, WALK_SPEED)

	velocity = v
	move_and_slide()

	if _terrain and is_on_floor():
		var gh := _terrain.get_height(global_position.x, global_position.z)
		if global_position.y < gh:
			global_position.y = gh

	if _terrain and state == State.MOVE:
		var biome := _terrain.get_biome(global_position.x, global_position.z)
		if biome == TerrainMesh.Biome.WATER or biome == TerrainMesh.Biome.SHORE:
			velocity.x = 0.0
			velocity.z = 0.0
			_set_state(State.IDLE)
			_idle_timer = 0.0


func _wander() -> void:
	if not _terrain:
		return
	var half := _terrain.terrain_size * 0.5 - 20.0
	var rx   := global_position.x + randf_range(-40.0, 40.0)
	var rz   := global_position.z + randf_range(-40.0, 40.0)
	rx = clampf(rx, -half, half)
	rz = clampf(rz, -half, half)
	var target := _snap_to_ground(Vector3(rx, 0, rz))
	if _terrain.get_biome(target.x, target.z) in [
			TerrainMesh.Biome.WATER, TerrainMesh.Biome.SHORE,
			TerrainMesh.Biome.ROCKY, TerrainMesh.Biome.CLIFF]:
		return
	assign_move(target)


func _snap_to_ground(pos: Vector3) -> Vector3:
	if _terrain:
		pos.y = _terrain.get_height(pos.x, pos.z) + 0.5
	return pos


# ── Visuals ────────────────────────────────────────────────────────────────────

func _update_visuals() -> void:
	var col: Color
	match state:
		State.IDLE:   col = Color(0.55, 0.80, 0.55)
		State.MOVE:   col = Color(0.55, 0.70, 0.95)
		State.WORK:   col = Color(0.95, 0.80, 0.30)
		State.EAT:    col = Color(0.95, 0.55, 0.25)
		State.SLEEP:  col = Color(0.55, 0.55, 0.80)

	if hunger > 0.9 or energy < 0.1:
		col = col.lerp(Color(1.0, 0.2, 0.2), 0.5)

	if _body_mesh:
		var mat := _body_mesh.get_surface_override_material(0) as StandardMaterial3D
		if mat == null:
			mat = StandardMaterial3D.new()
			_body_mesh.set_surface_override_material(0, mat)
		mat.albedo_color = col

	if _name_label:
		var state_names := ["IDLE", "WALK", "WORK", "EAT", "SLEEP"]
		_name_label.text = human_name + "\n" + state_names[state]
