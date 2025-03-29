#ifdef TOOLS_ENABLED

#pragma once

#include <godot_cpp/classes/box_container.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/confirmation_dialog.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/h_box_container.hpp>
#include <godot_cpp/classes/h_slider.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include <godot_cpp/classes/item_list.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/classes/menu_button.hpp>
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/packed_scene.hpp>
#include <godot_cpp/classes/panel.hpp>
#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/slider.hpp>
#include <godot_cpp/classes/spin_box.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/v_box_container.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "../tiled_node.h"
#include "core/editor/editor_cursor.h"
#include "core/editor/selection_manager.h"
#include "editor_control.h"

using namespace godot;

// XXX move palette to bottom panel for HexGrid
class HexMapTiledNodeEditorPlugin : public EditorPlugin {
    GDCLASS(HexMapTiledNodeEditorPlugin, EditorPlugin);

    HexMapTiledNode *hex_map = nullptr;
    Ref<MeshLibrary> mesh_library = nullptr;
    EditorControl *editor_control = nullptr;
    Control *bottom_panel = nullptr;
    EditorCursor *editor_cursor = nullptr;

    SelectionManager *selection_manager = nullptr;
    Vector3 selection_anchor;
    Vector<HexMapCellId> last_selection;

    // Tried using UndoRedo with MERGE_ALL to maintain the list of cells
    // painted/erased while the mouse moved.  This does not work properly
    // because UndoRedo::create_action() will only merge the action if the
    // last update happened within 800ms.  If the delay is greater than 800ms,
    // it will create a new action instead of merging.
    //
    // https://github.com/godotengine/godot-proposals/issues/8527
    //
    // So we have to track updated tiles ourselves.  This vector is also used
    // by move to save off the original cell positions for cancel, undo/redo.
    struct CellChange {
        HexMapCellId cell_id;
        int orig_tile = 0;
        HexMapTiledNode::TileOrientation orig_orientation;
        int new_tile = 0;
        HexMapTiledNode::TileOrientation new_orientation;
    };
    Vector<CellChange> cells_changed;

    enum InputState {
        INPUT_STATE_DEFAULT,
        INPUT_STATE_PAINTING,
        INPUT_STATE_ERASING,
        INPUT_STATE_SELECTING,
        INPUT_STATE_MOVING,
        INPUT_STATE_CLONING,
    };
    InputState input_state = INPUT_STATE_DEFAULT;

    void commit_cell_changes(String);

    // callbacks for HexMap signals
    void update_mesh_library();
    void hex_space_changed();

    // callbacks used by signals from EditorControl
    void tile_changed(int p_mesh_id);
    void plane_changed(int p_axis);
    void axis_changed(int p_axis);
    void cursor_changed(Variant p_orientation);

    // manage selection
    void deselect_cells();
    void _deselect_cells();
    void _select_cell(Vector3i);
    void _set_selection_v(Array);

    // perform actions on selected cells
    void selection_fill();
    void selection_clear();
    void copy_selection_to_cursor();
    void selection_move();
    void selection_move_cancel();
    void selection_move_apply();
    void selection_clone();
    void selection_clone_cancel();
    void selection_clone_apply();

protected:
    void _notification(int p_what);
    static void _bind_methods();

public:
    virtual void _make_visible(bool visible) override;
    virtual bool _handles(Object *object) const override;
    virtual void _edit(Object *p_object) override;
    virtual int32_t _forward_3d_gui_input(Camera3D *viewport_camera,
            const Ref<InputEvent> &event) override;

    HexMapTiledNodeEditorPlugin();
    ~HexMapTiledNodeEditorPlugin();
};

#endif
