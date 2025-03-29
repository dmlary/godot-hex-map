#include "godot_cpp/classes/input_event_key.hpp"
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

#include "core/cell_id.h"
#include "core/editor/editor_cursor.h"
#include "core/editor/selection_manager.h"
#include "core/hex_map_node.h"
#include "core/tile_orientation.h"

using namespace godot;

// XXX move palette to bottom panel for HexGrid
class HexMapNodeEditorPlugin : public EditorPlugin {
    GDCLASS(HexMapNodeEditorPlugin, EditorPlugin);

public:
    /// subclasses should also override this to show ui elements, and call this
    /// implementation
    virtual void _make_visible(bool visible) override;

    // XXX subclasses override this
    // virtual bool _handles(Object *object) const override;
    virtual void _edit(Object *p_object) override;
    virtual int32_t _forward_3d_gui_input(Camera3D *viewport_camera,
            const Ref<InputEvent> &event) override;

    HexMapNodeEditorPlugin();
    ~HexMapNodeEditorPlugin();

protected:
    void _notification(int p_what);
    static void _bind_methods();

    /// add setting entry for a keyboard shortcut
    void add_editor_shortcut(const String &path, const String &name, Key);

    /// set cursor orientation
    void cursor_set_orientation(HexMapTileOrientation);

    /// rotate the cursor in 60 degree steps around the y-axis
    /// @param steps negative values rotate clockwise, positive
    /// counter-clockwise
    ///
    /// this function will call cursor_set_orientation() for any change
    void cursor_rotate(int steps);

    /// flip the cursor 180 degrees around the x-axis
    ///
    /// this function will call cursor_set_orientation() for any change
    void cursor_flip();

    /// set edit axis
    void edit_plane_set_axis(EditorCursor::EditAxis);

    /// set edit axis depth
    void edit_plane_set_depth(int);

    /// Handle common keypress shortcuts for navigating the spatial editor
    ///
    /// Note: If a selection is active, EditorPlugin will consume the first
    /// escape keypress.  All following escape keypresses will be sent to this
    /// function.
    virtual EditorPlugin::AfterGUIInput handle_keypress(Ref<InputEventKey>);

    // callbacks for HexMap signals
    void hex_space_changed();

    void plane_changed(int p_axis);
    void axis_changed(int p_axis);

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

    /// node being edited
    HexMapNode *hex_map = nullptr;

    EditorCursor *editor_cursor = nullptr;

    /// selection manager, contains & controls the list of currently selected
    /// tiles
    SelectionManager *selection_manager = nullptr;

    /// begining poing of an ongoing selection
    Vector3 selection_anchor;
    /// Array of Vector3i encoded HexMapCellIds representing last active
    /// selection
    Array last_selection;
    /// Array of previous cell contents; used during move selection
    Array move_source_cells;

private:
    // Tried using UndoRedo with MERGE_ALL to maintain the list of cells
    // painted/erased while the mouse moved.  This does not work properly
    // because UndoRedo::create_action() will only merge the action if the
    // last update happened within 800ms.  If the delay is greater than 800ms,
    // it will create a new action instead of merging.
    //
    // https://github.com/godotengine/godot-proposals/issues/8527
    //
    // So we have to track updated tiles ourselves.
    struct CellChange {
        HexMapCellId cell_id;
        int orig_tile = HexMapNode::INVALID_CELL_ITEM;
        HexMapTileOrientation orig_orientation;
        int new_tile = HexMapNode::INVALID_CELL_ITEM;
        HexMapTileOrientation new_orientation;
    };
    Vector<CellChange> cells_changed;

    /// Input handling states
    enum InputState {
        INPUT_STATE_DEFAULT,
        INPUT_STATE_PAINTING,
        INPUT_STATE_ERASING,
        INPUT_STATE_SELECTING,
        INPUT_STATE_MOVING,
        INPUT_STATE_CLONING,
    };

    /// current input handling state
    InputState input_state = INPUT_STATE_DEFAULT;

    /// active edit axis
    EditorCursor::EditAxis edit_axis = EditorCursor::EditAxis::AXIS_Y;

    /// saved edit axis depths so the axis can change but return to the same
    /// depth when switched back
    Array edit_axis_depth;

    /// GDScript set_orientation handler; calls true cursor_set_orientation
    void cursor_set_orientation(int);

    /// used by PAINTING and ERASING to commit changes via undo/redo
    void commit_cell_changes(String desc);

    /// save the editor state into the metadata of the node provided
    void write_editor_state(HexMapNode *) const;

    /// load the editor state from the metadata of the provided node
    ///
    /// If no state is found in the node, the state will be initialized to sane
    /// values.
    void read_editor_state(const HexMapNode *);

    /// rebuild the cursor from the UI state
    virtual void rebuild_cursor();
};

#endif
