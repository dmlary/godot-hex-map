extends Camera3D

@export var hex_map: HexMapTiled
@export var label_cells := true
@export var marker: Node3D

@onready var raycast: RayCast3D = $RayCast3D
@onready var label: Label = %DebugLabel

var map: AStar3D;
var points: Dictionary;
var cursor_cell;
var lines = [];

func _ready() -> void:
	map = AStar3D.new()
	points = {};

	for cell in hex_map.get_used_cells():
		map.add_point(cell.as_int(), hex_map.cell_id_to_local(cell))

	for cell in hex_map.get_used_cells():
		for neighbor in cell.get_neighbors():
			if hex_map.get_cell_item(neighbor) != -1:
				map.connect_points(cell.as_int(), neighbor.as_int())


	# label all of the cells
	if label_cells == true:
		for cell in hex_map.get_used_cells():
			var above = cell
			above.y += 1
			if hex_map.get_cell_item(above) != -1:
				continue

			var coords = hex_map.cell_id_to_local(cell);
			var clabel := Label3D.new();
			clabel.no_depth_test = true
			clabel.text = str(cell)
			coords.y += 0.5
			clabel.position = coords
			clabel.billboard = BaseMaterial3D.BILLBOARD_ENABLED
			clabel.font_size = 64
			clabel.outline_size = 0
			clabel.modulate = Color.BLACK
			get_parent().add_child.call_deferred(clabel)

func _process(_delta: float) -> void:
	var mouse_pos: Vector2 = get_viewport().get_mouse_position()
	raycast.target_position = project_local_ray_normal(mouse_pos) * 100.0
	raycast.force_raycast_update()

	if !raycast.is_colliding():
		label.text = "not colliding"
		cursor_cell = null
		return

	var collider = raycast.get_collider()
	if collider is not HexMapNode:
		return

	var collison_point = raycast.get_collision_point()
	var local = collider.to_local(collison_point)
	var cell = hex_map.local_to_cell_id(local)
	if cell == cursor_cell:
		return
	cursor_cell = cell;

	var item = hex_map.get_cell_item(cell)
	var cell2 = hex_map.local_to_cell_id(collison_point)
	label.text = "collision %s\nlocal %s\ncell %s %s\nitem %d" % [ collison_point, local, cell, cell2, item]

	var point = points.get(cell)
	if point == null:
		return

	var path := map.get_point_path(marker_point(), points.get(cell))
	label.text += "\npath: %s" % [path]
	draw_path(path)

func marker_point() -> int:
	# grab the marker y and search down until we find a cell
	var pos = marker.position
	while true:
		# XXX infinite loop during shutdown
		var cell = hex_map.local_to_map(pos)
		var point = points.get(cell)
		if point:
			return point
		pos.y -= 1.0
	return 0

func draw_path(path: PackedVector3Array):
	for line in lines:
		line.queue_free()
	lines.clear()

	print("drawing", path)
	for i in path.size()-1:
		var a = path[i]
		var b = path[i+1]
		var line = draw_line(hex_map.map_to_local(a) + Vector3(0, 2, 0), hex_map.map_to_local(b) + Vector3(0, 2, 0))
		hex_map.add_child(line)
		lines.append(line)

func draw_line(pos1: Vector3, pos2: Vector3) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	var immediate_mesh := ImmediateMesh.new()
	var material := ORMMaterial3D.new()

	mesh_instance.mesh = immediate_mesh
	mesh_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF

	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINES, material)
	immediate_mesh.surface_add_vertex(pos1)
	immediate_mesh.surface_add_vertex(pos2)
	immediate_mesh.surface_end()

	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = Color.DARK_RED

	return mesh_instance


func _input(event: InputEvent) -> void:
	if event is InputEventPanGesture:
		fov = clampf(fov + event.delta.y, 15.0, 75.0);
	elif event.is_action("zoom_in"):
		fov = clampf(fov - 2.0, 15.0, 75.0);
	elif event.is_action("zoom_out"):
		fov = clampf(fov + 2.0, 15.0, 75.0);
	elif event.is_action("place_marker") && cursor_cell:
		var pos = hex_map.map_to_local(cursor_cell)
		pos.y += 1.5
		marker.position = pos
