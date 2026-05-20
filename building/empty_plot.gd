extends Node3D
class_name EmptyPlot

const PLOT_CELLS := 4
const CELL_SIZE  := 5.0   # must match BuildGrid.GRID_STEP

enum Direction { NORTH, EAST, SOUTH, WEST }

var origin_cell:  Vector2i          = Vector2i.ZERO
var direction:    Direction         = Direction.SOUTH
var linked_zone:  Dictionary        = {}   # BuildGrid zone dict {id, label, cells, mesh}, or {}
var requested_by: Human             = null
var purpose:      String            = ""
var applicants:   Array[Dictionary] = []   # [{human: Human, purpose: String}]

var _floor_mat: StandardMaterial3D = null
var _label:     Label3D            = null
var _dir_arrow: MeshInstance3D     = null


func setup(cell: Vector2i, terrain: TerrainMesh, dir: Direction = Direction.SOUTH) -> void:
	origin_cell = cell
	direction   = dir
	name        = "4x4 Open Plot"
	add_to_group("empty_plots")

	var plot_w := PLOT_CELLS * CELL_SIZE
	var cx     := (float(cell.x) + PLOT_CELLS * 0.5) * CELL_SIZE
	var cz     := (float(cell.y) + PLOT_CELLS * 0.5) * CELL_SIZE
	global_position = Vector3(cx, terrain.get_height(cx, cz), cz)

	_build_floor(plot_w)
	_build_outline(plot_w)
	_build_collider(plot_w)
	_build_label()
	_build_direction_arrow()
	set_direction(direction)


# ── Visuals ────────────────────────────────────────────────────────────────────

func _build_floor(plot_w: float) -> void:
	var box      := BoxMesh.new()
	box.size     =  Vector3(plot_w, 0.12, plot_w)
	_floor_mat   = StandardMaterial3D.new()
	_floor_mat.albedo_color = Color(0.78, 0.68, 0.46, 0.40)
	_floor_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_floor_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var mi               := MeshInstance3D.new()
	mi.mesh              = box
	mi.material_override = _floor_mat
	mi.position          = Vector3(0, 0.06, 0)
	add_child(mi)


func _build_outline(plot_w: float) -> void:
	var hs    := plot_w * 0.5
	var y     := 0.14
	var col   := Color(0.95, 0.82, 0.40)
	var verts := PackedVector3Array()
	var cols  := PackedColorArray()
	var corners := [
		Vector3(-hs, y, -hs), Vector3( hs, y, -hs),
		Vector3( hs, y,  hs), Vector3(-hs, y,  hs),
	]
	for i in 4:
		verts.append(corners[i]);           cols.append(col)
		verts.append(corners[(i + 1) % 4]); cols.append(col)

	var arrays := []; arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	arrays[ArrayMesh.ARRAY_COLOR]  = cols
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.shading_mode               = BaseMaterial3D.SHADING_MODE_UNSHADED
	arr_mesh.surface_set_material(0, mat)

	var mi      := MeshInstance3D.new()
	mi.mesh     = arr_mesh
	add_child(mi)


func _build_collider(plot_w: float) -> void:
	var box      := BoxShape3D.new()
	box.size     = Vector3(plot_w, 1.0, plot_w)
	var cshape   := CollisionShape3D.new()
	cshape.shape    = box
	cshape.position = Vector3(0, 0.5, 0)
	var body     := StaticBody3D.new()
	body.add_child(cshape)
	add_child(body)


func _build_label() -> void:
	_label            = Label3D.new()
	_label.text       = "4×4 Open Plot"
	_label.font_size  = 52
	_label.pixel_size = 0.04
	_label.modulate   = Color(0.95, 0.82, 0.40)
	_label.position   = Vector3(0, 1.8, 0)
	_label.billboard  = BaseMaterial3D.BILLBOARD_ENABLED
	_label.no_depth_test = true
	_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	add_child(_label)


func _build_direction_arrow() -> void:
	var hs  := PLOT_CELLS * CELL_SIZE * 0.5  # 10.0
	var y   := 0.22
	var col := Color(0.40, 0.85, 1.00)
	# Arrow shaft + head pointing in -Z (North); rotated by set_direction()
	var tip  := Vector3(0.0,  y, -(hs - 0.8))
	var base := Vector3(0.0,  y, -2.5)
	var verts := PackedVector3Array([
		base, tip,
		tip, Vector3(-1.8, y, -(hs - 4.0)),
		tip, Vector3( 1.8, y, -(hs - 4.0)),
	])
	var cols := PackedColorArray()
	for _i in verts.size(): cols.append(col)

	var arrays := []; arrays.resize(ArrayMesh.ARRAY_MAX)
	arrays[ArrayMesh.ARRAY_VERTEX] = verts
	arrays[ArrayMesh.ARRAY_COLOR]  = cols
	var arr_mesh := ArrayMesh.new()
	arr_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
	var mat := StandardMaterial3D.new()
	mat.vertex_color_use_as_albedo = true
	mat.shading_mode               = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.no_depth_test              = true
	arr_mesh.surface_set_material(0, mat)

	_dir_arrow      = MeshInstance3D.new()
	_dir_arrow.mesh = arr_mesh
	add_child(_dir_arrow)


func _update_visuals() -> void:
	var n_apps := get_applicants().size()
	if _floor_mat:
		if is_requested():
			_floor_mat.albedo_color = Color(0.46, 0.72, 0.52, 0.45)
		elif n_apps > 0:
			_floor_mat.albedo_color = Color(0.46, 0.60, 0.80, 0.40)
		else:
			_floor_mat.albedo_color = Color(0.78, 0.68, 0.46, 0.40)
	if _label:
		if is_requested():
			_label.text     = "%s\n(%s)" % [requested_by.human_name, purpose]
			_label.modulate = Color(0.55, 0.92, 0.55)
		elif n_apps > 0:
			_label.text     = "4×4 Open Plot\n(%d applicant%s)" % [n_apps, "s" if n_apps != 1 else ""]
			_label.modulate = Color(0.65, 0.82, 1.0)
		else:
			_label.text     = "4×4 Open Plot"
			_label.modulate = Color(0.95, 0.82, 0.40)


# ── Public API ─────────────────────────────────────────────────────────────────

func set_direction(d: Direction) -> void:
	direction = d
	if _dir_arrow:
		# d=0 N→0, d=1 E→PI/2, d=2 S→PI, d=3 W→3PI/2 (same as -PI/2 visually)
		_dir_arrow.rotation.y = d * (PI / 2.0)


func link_terrain(zone: Dictionary) -> void:
	linked_zone = zone
	_update_visuals()


func unlink_terrain() -> void:
	linked_zone = {}
	_update_visuals()


func request(human: Human, p: String) -> void:
	requested_by = human
	purpose      = p
	_update_visuals()


func cancel_request() -> void:
	requested_by = null
	purpose      = ""
	_update_visuals()


func is_requested() -> bool:
	return requested_by != null and is_instance_valid(requested_by)


func get_applicants() -> Array[Dictionary]:
	applicants = applicants.filter(
		func(a: Dictionary) -> bool: return is_instance_valid(a.human)
	)
	return applicants


func add_applicant(human: Human, p: String) -> void:
	applicants.append({"human": human, "purpose": p})
	_update_visuals()


func remove_applicant(human: Human) -> void:
	applicants = applicants.filter(
		func(a: Dictionary) -> bool: return a.human != human
	)
	_update_visuals()


func accept_applicant(human: Human) -> void:
	for a: Dictionary in applicants:
		if a.human == human:
			applicants.clear()
			request(human, a.purpose)
			return


func get_cells() -> Array:
	var out: Array = []
	for r in PLOT_CELLS:
		for c in PLOT_CELLS:
			out.append(Vector2i(origin_cell.x + c, origin_cell.y + r))
	return out


func get_selection_info() -> Dictionary:
	var dir_names := ["North ↑", "East →", "South ↓", "West ←"]
	var info: Dictionary = {"Exit": dir_names[direction]}
	if not linked_zone.is_empty():
		info["Terrain"] = linked_zone.get("label", "Zone")
	if is_requested():
		info["Claimed by"] = requested_by.human_name
		info["Purpose"]    = purpose
		return info
	var n := get_applicants().size()
	info["Status"]     = "Available"
	info["Applicants"] = str(n) if n > 0 else "None"
	return info
