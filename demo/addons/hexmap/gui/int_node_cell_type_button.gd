@tool

extends PanelContainer

signal pressed

@export var value: int :
	set(v):
		value = v
		%ValueLabel.text = "(%d)" % value

@export var type: String = "name":
	set(value):
		type = value
		%Label.text = value
@export var color: Color:
	set(value):
		color = value
		%Color.color = value 
@export var selected := false :
	set(value):
		if value:
			add_theme_stylebox_override("panel", get_theme_stylebox("focus"))
		else:
			remove_theme_stylebox_override("panel")
		selected = value

@export var frozen := false :
	set(value):
		frozen = value
		if !frozen:
			hovered = false

var hovered := false : set = set_hover

func set_hover(state: bool) -> void:
	hovered = state
	if selected or frozen:
		return
	if state:
		add_theme_stylebox_override("panel", get_theme_stylebox("hovered"))
	else:
		remove_theme_stylebox_override("panel")

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	mouse_entered.connect(set_hover.bind(true))
	mouse_exited.connect(set_hover.bind(false))

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func _input(event: InputEvent) -> void:
	if event is not InputEventMouseButton:
		return
	if not hovered:
		return
	if not event.is_pressed():
		return
	if not event.button_index == MOUSE_BUTTON_LEFT:
		return
	accept_event()
	selected = true
	emit_signal("pressed")
