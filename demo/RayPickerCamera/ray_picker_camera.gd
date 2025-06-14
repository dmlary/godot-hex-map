extends Camera3D

@export var hex_map: HexMapTiled
@export var label_cells := true
@export var marker: Node3D

@onready var raycast: RayCast3D = $RayCast3D
@onready var label: Label = %DebugLabel

var map: AStar3D
var points: Dictionary
var cursor_cell: HexMapCellId
var marker_cell: HexMapCellId :
    set(value):
        marker_cell = value.duplicate()
        if marker == null:
            return
        # grab the cell above the marker cell to place the marker
        var cell = value.duplicate()
        cell.y += 1
        marker.position = hex_map.get_cell_center(cell)

var lines := []

func _ready() -> void:
    map = AStar3D.new()
    points = {};

    # iterate once to add each cell to the astar
    for cell_vec in hex_map.get_cell_vecs():
        var cell := HexMapCellId.from_vec(cell_vec)
        # don't add cells that have a cell on top of them
        if hex_map.has(cell.up()):
            continue

        map.add_point(cell.as_int(), hex_map.get_cell_center(cell))

    # iterate a second time to connect cells with their neighbors
    for cell_vec in hex_map.get_cell_vecs():
        var cell := HexMapCellId.from_vec(cell_vec)
        var cell_int := cell.as_int()
        if not map.has_point(cell_int):
            continue
        for neighbor in cell.get_neighbors():
            var neighbor_int := neighbor.as_int()
            if not map.has_point(neighbor_int):
                continue
            map.connect_points(cell_int, neighbor_int)

    # label all of the cells
    if label_cells == true:
        var max_neighbour_search_height = 5
        var label_string = "q:{q}, r:{r}, y:{y}"

        for cell_vec in hex_map.get_cell_vecs():
            var cell := HexMapCellId.from_vec(cell_vec)
            var is_top_cell = true
            var current_search_height = 0
            var above = cell
            while is_top_cell && current_search_height <= max_neighbour_search_height:
                above = above.up()
                is_top_cell = !hex_map.has(above)
                current_search_height += 1

            if !is_top_cell:
                continue

            var coords_above = hex_map.get_cell_center(cell.up());
            var clabel := Label3D.new();
            clabel.no_depth_test = true
            clabel.text = label_string.format({ "q": cell.q, "r": cell.r, "y": cell.y })
            clabel.position = coords_above
            clabel.billboard = BaseMaterial3D.BILLBOARD_ENABLED
            clabel.font_size = 48
            clabel.outline_size = 3
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
    var cell = hex_map.get_cell_id(local)
    if cell.equals(cursor_cell):
        return
    cursor_cell = cell;

    var item = hex_map.get_cell(cell)
    label.text = "collision %s\nlocal %s\ncell %s \nitem %s" % [ collison_point, local, cell, item]

    if marker_cell == null:
        return
    if not map.has_point(cell.as_int()):
        return
    var path := map.get_point_path(marker_cell.as_int(), cell.as_int())

    label.text += "\npath: %s" % [path]
    draw_path(path)


func draw_path(path: PackedVector3Array):
    for line in lines:
        line.queue_free()
    lines.clear()

    for i in path.size()-1:
        var a = path[i]
        var b = path[i+1]
        var line = draw_line(a + Vector3(0, 2, 0), b + Vector3(0, 2, 0))
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
        marker_cell = cursor_cell
