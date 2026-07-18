@tool

extends Control

enum Tool { PAINT, ERASE, SELECT }
enum DisplayMode { THUMBNAIL, LIST }

@export var node: HexMapTiled :
    set(value) :
        if value == node:
            return

        if node != null:
            node.mesh_library_changed.disconnect(_on_node_mesh_library_changed)

        node = value
        node.mesh_library_changed.connect(_on_node_mesh_library_changed)
        _on_node_mesh_library_changed()

@export var editor_plugin: EditorPlugin :
    set(value):
        if editor_plugin != null:
            # disconnect existing EditorPlugin
            editor_plugin.paint_value_changed \
                    .connect(_on_editor_plugin_paint_value_changed)
            pass

        editor_plugin = value

        # set up all the needed connections
        editor_plugin.paint_value_changed \
                .connect(_on_editor_plugin_paint_value_changed)

        %CommonToolbar.editor_plugin = editor_plugin

var paint_value := -1 :
    set(value):
        if value == paint_value:
            return
        paint_value = value
        %MeshPalette.selected_id = value

var mesh_id := -1

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
    %FilterLineEdit.text_changed.connect(func(v): %MeshPalette.filter = v)
    %MeshPalette.compact_view = %ThumbnailButton.button_pressed
    %ThumbnailButton.toggled.connect(func(v): %MeshPalette.compact_view = v)
    %MeshPalette.selected.connect(func(v): editor_plugin.paint_value = v)
    %ZoomWidget.changed.connect(_on_zoom_widget_changed)
    _on_zoom_widget_changed(1.0)

func _on_node_mesh_library_changed() -> void:
    %MeshPalette.clear()
    var mesh_library := node.mesh_library
    for id in mesh_library.get_item_list():
        var preview = mesh_library.get_item_preview(id)
        if not preview:
            preview = EditorInterface.make_mesh_previews([mesh_library.get_item_mesh(id)], 128)[0]
        %MeshPalette.add_item({
            "id": id,
            "preview": preview,
            "desc": mesh_library.get_item_name(id),
        })

func _on_zoom_widget_changed(scale: float) -> void:
    %MeshPalette.preview_size = Vector2(64, 64) * scale

func _on_editor_plugin_paint_value_changed(value: int) -> void:
    paint_value = value
