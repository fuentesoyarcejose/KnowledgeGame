extends Node3D

## Animations to choose from. Each tree picks one at random on spawn.
## Leave empty to auto-select any track containing "wind", "idle", or "sway".
@export var animations: Array[String] = []

var marked_for_cutting: bool = false


func _ready() -> void:
	var player := _find_animation_player(self)
	if not player:
		return
	var available := Array(player.get_animation_list())
	if available.is_empty():
		return

	# Build candidate list from the export array, keeping only names that exist.
	var candidates: Array[String] = []
	for a in animations:
		if a in available:
			candidates.append(a)

	# Fallback: auto-pick tracks with wind/idle/sway keywords.
	if candidates.is_empty():
		for n: String in available:
			if "wind" in n.to_lower() or "idle" in n.to_lower() or "sway" in n.to_lower():
				candidates.append(n)

	# Last resort: use everything.
	if candidates.is_empty():
		candidates.assign(available)

	player.play(candidates[randi() % candidates.size()])


func toggle_cut_mark() -> void:
	marked_for_cutting = not marked_for_cutting


func get_selection_info() -> Dictionary:
	return {"Cut": "Yes" if marked_for_cutting else "No"}


func cut_down() -> void:
	var layer := get_parent() as PropLayer
	if layer:
		var terrain := layer.get_parent() as TerrainMesh
		if terrain:
			terrain.modify_forest(global_position.x, global_position.z, 15.0, -1.5)
		layer.remove_instance(self)


func _find_animation_player(node: Node) -> AnimationPlayer:
	if node is AnimationPlayer:
		return node
	for child in node.get_children():
		var found := _find_animation_player(child)
		if found:
			return found
	return null
