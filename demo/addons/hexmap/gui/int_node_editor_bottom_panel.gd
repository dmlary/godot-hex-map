@tool

extends VBoxContainer

signal types_changed

@export var int_map: HexMapInt :
	set(value):
		if int_map != value:
			%TypeSelector.int_map = value
			int_map = value

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
