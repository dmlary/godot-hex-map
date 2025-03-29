@tool

class_name HexMapIntMapCellTypeForm

extends HBoxContainer

signal submitted(value:int, type: String, color: Color)
signal cancelled

@export var value := -1
@export var type : String :
	set(value):
		type = value
		$Name.text = value
@export var color : Color :
	set(value):
		color = value
		$Color.color = color

func grab_focus():
	$Name.grab_focus()
	$Name.select_all()

func submit_form(_text: String = "") -> void:
	var name = $Name.text.strip_edges()
	if name.is_empty():
		push_error("HexMapIntNode cell type must have a non-empty name")
		return

	submitted.emit(value, name, $Color.color)
	queue_free()

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	$Name.text_submitted.connect(submit_form)
	$AddButton.pressed.connect(submit_form)
	$CloseButton.pressed.connect(func(): cancelled.emit(); queue_free())

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
