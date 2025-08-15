@tool
extends HBoxContainer

@export var editor_plugin: EditorPlugin:
    set(value):
        if editor_plugin != null:
            editor_plugin.edit_plane_axis_changed \
                    .disconnect(_on_edit_plane_axis_changed)
            editor_plugin.edit_plane_depth_changed \
                    .disconnect(_on_edit_plane_depth_changed)
            editor_plugin.cursor_active_changed \
                    .connect(_on_editor_plugin_cursor_active_changed)
            editor_plugin.selection_active_changed \
                    .connect(_on_editor_plugin_selection_active_changed)
        editor_plugin = value
        if editor_plugin == null:
            return

        editor_plugin.edit_plane_axis_changed \
                .connect(_on_edit_plane_axis_changed)
        editor_plugin.edit_plane_depth_changed \
                .connect(_on_edit_plane_depth_changed)
        editor_plugin.cursor_active_changed \
                .connect(_on_editor_plugin_cursor_active_changed)
        editor_plugin.selection_active_changed \
                .connect(_on_editor_plugin_selection_active_changed)

        axis = editor_plugin.edit_plane_axis
        depth = editor_plugin.edit_plane_depth
        _on_editor_plugin_cursor_active_changed(
                editor_plugin.is_cursor_active())
        _on_editor_plugin_selection_active_changed(
                editor_plugin.is_selection_active())

@export var axis := -1:
    set(value):
        if value == axis:
            return

        axis = value
        editor_plugin.edit_plane_axis = value

        if not is_node_ready():
            await ready
        %AxisDropdown.selected = value

@export var depth: int:
    set(value):
        if value == depth:
            return

        depth = value
        editor_plugin.edit_plane_depth = value

        if not is_node_ready():
            await ready
        %DepthSpinBox.value = value

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    %CCWButton.pressed.connect(_on_cursor_rotate_button_pressed.bind(1))
    %CWButton.pressed.connect(_on_cursor_rotate_button_pressed.bind(-1))
    %MoveButton.pressed.connect(_on_move_selection_button_pressed)
    %CopyButton.pressed.connect(_on_copy_selection_button_pressed)
    %FillButton.pressed.connect(_on_fill_selection_button_pressed)
    %ClearButton.pressed.connect(_on_clear_selection_button_pressed)
    %AxisDropdown.item_selected.connect(func(v): axis = v)
    %DepthSpinBox.value_changed.connect(func(v): depth = v)
    %DepthLabelButton.pressed.connect(func(): depth = 0)
    %BuildupBrushButton.toggled.connect(_on_buildup_brush_button_toggled)

func _on_edit_plane_axis_changed(value: int) -> void:
    axis = value

func _on_edit_plane_depth_changed(value: int) -> void:
    depth = value

func _on_editor_plugin_cursor_active_changed(active: bool) -> void:
    %CCWButton.disabled = !active
    %CWButton.disabled = !active
    %FillButton.disabled = (editor_plugin.paint_value == -1 or !editor_plugin.is_selection_active())

func _on_editor_plugin_selection_active_changed(active: bool) -> void:
    %CopyButton.disabled = !active
    %MoveButton.disabled = !active
    %ClearButton.disabled = !active
    %FillButton.disabled = (editor_plugin.paint_value == -1 or !active)

func _on_cursor_rotate_button_pressed(steps: int) -> void:
    if editor_plugin:
        editor_plugin.cursor_rotate(steps)

func _on_move_selection_button_pressed() -> void:
    if editor_plugin:
        editor_plugin.selection_move()

func _on_copy_selection_button_pressed() -> void:
    if editor_plugin:
        editor_plugin.selection_clone()

func _on_fill_selection_button_pressed() -> void:
    if editor_plugin:
        editor_plugin.selection_fill()

func _on_clear_selection_button_pressed() -> void:
    if editor_plugin:
        editor_plugin.selection_clear()

func _on_buildup_brush_button_toggled(state: bool) -> void:
    if editor_plugin:
        editor_plugin.set_buildup_brush(state)
