@tool

extends VBoxContainer

const cell_type_button: PackedScene = preload("int_node_cell_type_button.tscn")
const cell_type_form: PackedScene = preload("int_node_cell_type_form.tscn")

signal types_changed
signal selection_changed(value)

@export var int_map: HexMapInt :
	set(value):
		if int_map != value:
			int_map = value
			rebuild_type_list()
@export var selected_type: int = -1 :
	set(value):
		if value != selected_type:
			selected_type = value
			emit_signal("selection_changed", value)
var context_menu_node: Node

func rebuild_type_list() -> void:
	for child in %CellTypes.get_children():
		child.queue_free()
	var types = int_map.get_cell_types()
	types.sort_custom(func(a, b): return a.name.naturalnocasecmp_to(b.name) < 0)
	for type in types:
		var cell = cell_type_button.instantiate()
		cell.value = type.value
		cell.type = type.name
		cell.color = type.color
		cell.pressed.connect(select.bind(type.value))
		%CellTypes.add_child(cell)

func select(value: int) -> void:
	for child in %CellTypes.get_children():
		if child.value != value:
			child.selected = false
	selected_type = value

func open_context_menu(pos: Vector2) -> void:
	var type_node
	for child in %CellTypes.get_children():
		if child.hovered:
			type_node = child
			break
	if !type_node:
		return

	context_menu_node = type_node
	%ContextMenu.position = pos
	%ContextMenu.visible = true

func context_menu_visibility_changed() -> void:
	var visible = %ContextMenu.visible
	for child in %CellTypes.get_children():
		child.frozen = visible

func context_menu_index_pressed(index: int) -> void:
	match index:
		0:
			edit_type(context_menu_node)
		1:
			int_map.remove_cell_type(context_menu_node.value)
			rebuild_type_list()
		_:
			push_error("Unknown context menu index ", index)

func cell_type_form_submitted(value: int, name: String, color: Color) -> void:
	if value < 0:
		int_map.add_cell_type(name, color)
	else:
		int_map.update_cell_type(value, name, color)
	types_changed.emit()
	rebuild_type_list()

func append_new_type_form() -> void:
	var form := cell_type_form.instantiate()
	form.color = Color.from_hsv(
			randf(),
			randf_range(0.2, 0.6),
			randf_range(0.9, 1.0))
	form.submitted.connect(cell_type_form_submitted)
	%CellTypes.add_child(form)
	form.grab_focus()

func edit_type(type_node: Node) -> void:
	var form := cell_type_form.instantiate()
	form.value = type_node.value
	form.type = type_node.type
	form.color = type_node.color
	form.submitted.connect(cell_type_form_submitted)
	form.cancelled.connect(rebuild_type_list)
	type_node.add_sibling(form)
	type_node.queue_free()
	form.grab_focus()
	
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	%ContextMenu.visibility_changed.connect(context_menu_visibility_changed)
	%ContextMenu.id_pressed.connect(context_menu_index_pressed)
	%AddTypeFormButton.pressed.connect(append_new_type_form)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		if event.is_pressed() && event.button_index == MOUSE_BUTTON_RIGHT:
			open_context_menu(event.position)
