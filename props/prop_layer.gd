@tool
extends Node3D
class_name PropLayer

@export var target_biome:  TerrainMesh.Biome = TerrainMesh.Biome.FOREST
## Label shown in the selection panel when a prop is clicked.
@export var display_name:  String            = ""
## Pool of scene variants. One is picked at random per placement.
## If empty, falls back to the legacy prop_scene slot below.
@export var prop_scenes:   Array[PackedScene] = []
@export var prop_scene:    PackedScene   # legacy single-scene slot
@export_range(10, 1000, 10)  var max_instances: int = 100
@export var min_spacing: float = 10.0
@export_range(0.1, 5.0, 0.05) var scale_min: float = 0.8
@export_range(0.1, 5.0, 0.05) var scale_max: float = 1.3
@export var align_to_normal: bool = false

var _instances: Array[Node] = []

# Maximum positional jitter applied within each grid cell (≤ GRID_STEP/2).
const _JITTER_FACTOR := 0.45


func refresh(terrain: TerrainMesh) -> void:
	for inst in _instances:
		if is_instance_valid(inst):
			inst.queue_free()
	_instances.clear()

	if prop_scenes.is_empty() and prop_scene == null:
		return

	var rng := RandomNumberGenerator.new()
	rng.seed = terrain.seed + name.hash()

	var gs        := BuildGrid.GRID_STEP
	var half      := terrain.terrain_size * 0.5
	var cell_half := int(half / gs)
	# Minimum occupied-cell radius for spacing enforcement.
	var space_r   := ceili(min_spacing / gs)
	var jitter    := gs * _JITTER_FACTOR

	# Enumerate all cells inside terrain bounds then shuffle once.
	var candidates: Array[Vector2i] = []
	for r in range(-cell_half, cell_half + 1):
		for c in range(-cell_half, cell_half + 1):
			candidates.append(Vector2i(c, r))

	for i in range(candidates.size() - 1, 0, -1):
		var j   := rng.randi() % (i + 1)
		var tmp := candidates[i]
		candidates[i] = candidates[j]
		candidates[j] = tmp

	# Grid-cell dictionary: O(1) spacing lookup instead of O(placed_count) scan.
	var placed: Dictionary = {}

	for cell in candidates:
		if _instances.size() >= max_instances:
			break

		# Spacing: reject if any already-placed cell is within space_r.
		var too_close := false
		for dr in range(-space_r, space_r + 1):
			for dc in range(-space_r, space_r + 1):
				if placed.has(Vector2i(cell.x + dc, cell.y + dr)):
					too_close = true
					break
			if too_close:
				break
		if too_close:
			continue

		# Biome check at cell centre.
		var cx := float(cell.x) * gs
		var cz := float(cell.y) * gs
		var biome := terrain.get_biome(cx, cz)
		if biome != target_biome:
			continue
		if biome == TerrainMesh.Biome.WATER or biome == TerrainMesh.Biome.SHORE:
			continue

		# Density filter: reject isolated biome pixels (forest edge spikes).
		if target_biome == TerrainMesh.Biome.FOREST:
			var r := maxf(min_spacing, 20.0)
			var nb_count := 0
			for nb: Vector2 in [Vector2(r, 0), Vector2(-r, 0), Vector2(0, r), Vector2(0, -r)]:
				if terrain.get_biome(cx + nb.x, cz + nb.y) == TerrainMesh.Biome.FOREST:
					nb_count += 1
			if nb_count < 2:
				continue

		placed[cell] = true

		# Final world position: cell centre + sub-cell jitter.
		var wx := cx + rng.randf_range(-jitter, jitter)
		var wz := cz + rng.randf_range(-jitter, jitter)

		var scene := _pick_scene(rng)
		if scene == null:
			continue

		var instance: Node3D = scene.instantiate()
		add_child(instance)
		_instances.append(instance)

		var sc    := rng.randf_range(scale_min, scale_max)
		var rot_y := rng.randf_range(0.0, TAU)
		var basis: Basis
		if align_to_normal:
			var up    := terrain.get_normal(wx, wz).normalized()
			var right := up.cross(Vector3.FORWARD)
			if right.length_squared() < 0.001:
				right = up.cross(Vector3.RIGHT)
			right = right.normalized()
			basis = Basis(right, up, -right.cross(up).normalized())
			basis = basis * Basis(Vector3.UP, rot_y)
		else:
			basis = Basis(Vector3.UP, rot_y)
		basis = basis.scaled(Vector3(sc, sc, sc))
		instance.transform = Transform3D(basis, Vector3(wx, terrain.get_height(wx, wz), wz))


func remove_instance(instance: Node) -> void:
	_instances.erase(instance)
	instance.queue_free()


func _pick_scene(rng: RandomNumberGenerator) -> PackedScene:
	if not prop_scenes.is_empty():
		return prop_scenes[rng.randi() % prop_scenes.size()]
	return prop_scene
