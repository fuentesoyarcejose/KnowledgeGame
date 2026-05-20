extends Node
class_name ColonyManager

const HUMAN_NAMES := [
	"Aldric", "Brynn", "Cael", "Deva", "Edric", "Fira",
	"Gareth", "Hana", "Ivan", "Jora", "Kael", "Lysa",
	"Maren", "Noel", "Oryn", "Petra", "Quinn", "Reva",
]

const HUMAN_SCENE := preload("res://colony/human.tscn")

@export_range(1, 20, 1) var starting_humans: int = 4

const CUT_WORK_DURATION  := 5.0
const DISPATCH_INTERVAL  := 2.0
const REQUEST_INTERVAL   := 15.0

var _terrain:          TerrainMesh      = null
var _humans:           Array[Human]     = []
var _name_pool:        Array[String]    = []
var _cut_assignments:  Dictionary = {}   # tree instance_id -> Human
var _plot_requests:    Dictionary = {}   # human instance_id -> EmptyPlot (applied or approved)
var _dispatch_timer:   float      = 0.0
var _request_timer:    float      = 5.0


func _ready() -> void:
	if Engine.is_editor_hint():
		return
	_name_pool.assign(HUMAN_NAMES)
	_name_pool.shuffle()
	call_deferred("_find_terrain_and_spawn")


func _find_terrain_and_spawn() -> void:
	_terrain = get_parent().get_node_or_null("TerrainMesh") as TerrainMesh
	if not _terrain:
		return
	if not _terrain.terrain_generated.is_connected(_on_terrain_regenerated):
		_terrain.terrain_generated.connect(_on_terrain_regenerated)
	var _dbg := get_node_or_null("/root/Debug")
	if _dbg:
		_dbg.set("_colony_manager", self)
	_spawn_starting_humans()


func _process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	_dispatch_timer -= delta
	if _dispatch_timer <= 0.0:
		_dispatch_timer = DISPATCH_INTERVAL
		_dispatch_cut_orders()
	_request_timer -= delta
	if _request_timer <= 0.0:
		_request_timer = REQUEST_INTERVAL
		_dispatch_requests()


func _on_terrain_regenerated() -> void:
	_cut_assignments.clear()
	_plot_requests.clear()
	for h in _humans:
		if is_instance_valid(h):
			h.queue_free()
	_humans.clear()
	_name_pool.assign(HUMAN_NAMES)
	_name_pool.shuffle()
	_spawn_starting_humans()


func _spawn_starting_humans() -> void:
	for i in starting_humans:
		spawn_human()


# ── Public API ─────────────────────────────────────────────────────────────────

func spawn_human(pos: Vector3 = Vector3(999999, 0, 999999)) -> Human:
	var h: Human = HUMAN_SCENE.instantiate()
	get_parent().add_child(h)

	var name_str := _pick_name()
	var spawn    := pos
	if spawn.x > 99999.0:
		spawn = _find_safe_spawn()

	h.setup(_terrain, spawn, name_str)
	_humans.append(h)
	return h


func get_humans() -> Array[Human]:
	_humans = _humans.filter(func(h): return is_instance_valid(h))
	return _humans


# ── Cut-order dispatch ─────────────────────────────────────────────────────────

func _dispatch_cut_orders() -> void:
	if not _terrain:
		return
	for child in _terrain.get_children():
		if not child is PropLayer:
			continue
		for prop in (child as PropLayer).get_children():
			if not (prop is Node3D) or not is_instance_valid(prop):
				continue
			if not prop.get("marked_for_cutting"):
				continue
			var tree_id := prop.get_instance_id()
			if _cut_assignments.has(tree_id):
				var h: Human = _cut_assignments[tree_id]
				if is_instance_valid(h) and h.state != Human.State.IDLE:
					continue
				_cut_assignments.erase(tree_id)
			var human := _find_available_human()
			if human == null:
				return
			_assign_cut(human, prop as Node3D)


func _find_available_human() -> Human:
	for h in get_humans():
		if h.is_available():
			return h
	return null


func _assign_cut(human: Human, tree: Node3D) -> void:
	var tree_id := tree.get_instance_id()
	_cut_assignments[tree_id] = human
	human.assign_work(tree.global_position, CUT_WORK_DURATION, func() -> void:
		_cut_assignments.erase(tree_id)
		if is_instance_valid(tree) and tree.get("marked_for_cutting"):
			tree.cut_down()
	)


# ── Plot-request dispatch ──────────────────────────────────────────────────────

func _dispatch_requests() -> void:
	# Clean stale entries: human removed from plot's applicants or plot freed
	var stale: Array = []
	for hid: int in _plot_requests:
		var plot: EmptyPlot = _plot_requests[hid]
		if not is_instance_valid(plot):
			stale.append(hid)
			continue
		var still_there := false
		if plot.is_requested() and is_instance_valid(plot.requested_by) \
				and plot.requested_by.get_instance_id() == hid:
			still_there = true
		else:
			for a: Dictionary in plot.get_applicants():
				if (a.human as Human).get_instance_id() == hid:
					still_there = true
					break
		if not still_there:
			stale.append(hid)
	for hid in stale:
		_plot_requests.erase(hid)

	# Pick an unclaimed plot to apply to
	var plots := get_tree().get_nodes_in_group("empty_plots")
	var target: EmptyPlot = null
	plots.shuffle()
	for node in plots:
		var plot := node as EmptyPlot
		if is_instance_valid(plot) and not plot.is_requested():
			target = plot
			break
	if target == null:
		return

	var human := _find_human_for_request()
	if human == null:
		return

	var purposes := ["Home", "Workshop"]
	_plot_requests[human.get_instance_id()] = target
	target.add_applicant(human, purposes[randi() % 2])


func _find_human_for_request() -> Human:
	for h in get_humans():
		if h.is_available() and not _plot_requests.has(h.get_instance_id()):
			return h
	return null


# ── Helpers ────────────────────────────────────────────────────────────────────

func _find_safe_spawn() -> Vector3:
	if not _terrain:
		return Vector3.ZERO
	var half := _terrain.terrain_size * 0.5 - 30.0
	var rng  := RandomNumberGenerator.new()
	rng.randomize()
	for _try in 40:
		var x := rng.randf_range(-half * 0.3, half * 0.3)
		var z := rng.randf_range(-half * 0.3, half * 0.3)
		var biome := _terrain.get_biome(x, z)
		if biome in [TerrainMesh.Biome.WATER, TerrainMesh.Biome.SHORE,
					 TerrainMesh.Biome.ROCKY, TerrainMesh.Biome.CLIFF]:
			continue
		return Vector3(x, _terrain.get_height(x, z) + 0.5, z)
	return Vector3(0, _terrain.ground_level + 0.5, 0)


func _pick_name() -> String:
	if _name_pool.is_empty():
		_name_pool = HUMAN_NAMES.duplicate()
		_name_pool.shuffle()
	return _name_pool.pop_back()
