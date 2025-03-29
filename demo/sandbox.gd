@tool
extends Node3D

@export var hex_map: HexMapTiled

# OperatorEvaluatorEqualObject::evaluate()
#	*
# godot::RefCounted does not pass operator= down to underlying type

var lines = [];

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


func _ready():
	var global := Transform3D(Basis(Vector3(0, 1, 0), PI/2)) # 90 degrees
	var local := Transform3D(Basis(), Vector3(10, 0, 0)) # x += 10
	var cell := Transform3D(Basis(), Vector3(0, 0, 5)) # z += 5
	var mesh := Transform3D(Basis(), Vector3(0, 1, 0)) # y += 1

	print("global ", global)
	print("local ", local)
	print("cell ", cell)
	print("mesh ", mesh)

	print("g * l * c * m = ", global * local * cell * mesh)
	var t := cell * mesh
	print("g * l * (c * m) = ", global * local * t)
	var d := global * local * cell * mesh
	print("global.inverse() * d = ", global.inverse() * d)
	print("local.inverse() * global.inverse() * d = ", local.inverse() * global.inverse() * d)


	# print("HexMap baked meshes ", hex_map.get_bake_meshes())
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func _on_hex_map_cells_changed(cells: Array) -> void:
	# print("Sandbox HexMap emitted `cells_changed`: ", cells)
	pass
