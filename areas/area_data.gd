class_name AreaData
extends RefCounted

enum Type { PLAINS, FOREST, RIVER, MOUNTAIN, DISTRICT }

const TYPE_COLOR: Dictionary = {
	0: Color(0.68, 0.76, 0.36),   # PLAINS
	1: Color(0.15, 0.50, 0.18),   # FOREST
	2: Color(0.20, 0.42, 0.80),   # RIVER
	3: Color(0.55, 0.52, 0.47),   # MOUNTAIN
	4: Color(0.80, 0.44, 0.12),   # DISTRICT
}

const TYPE_LABEL: Dictionary = {
	0: "Plains",
	1: "Forest",
	2: "River",
	3: "Mountain",
	4: "District",
}

var id:         int     = 0
var area_name:  String  = ""
var area_type:  Type    = Type.PLAINS
var color:      Color   = Color.WHITE
var centroid:   Vector2 = Vector2.ZERO  # world XZ
var cell_count:  int        = 0
var prop_count:  int        = 0
var prop_types:  Dictionary = {}  # type_name -> count
