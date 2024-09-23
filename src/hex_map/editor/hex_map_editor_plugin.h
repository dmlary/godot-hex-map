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
#include <godot_cpp/classes/panel.hpp>
#include <godot_cpp/classes/slider.hpp>
#include <godot_cpp/classes/spin_box.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/v_box_container.hpp>

#include "../hex_map.h"
#include "editor_control.h"
#include "editor_cursor.h"
#include "godot_cpp/variant/dictionary.hpp"
#include "mesh_library_palette.h"
#include "selection_manager.h"

using namespace godot;

// XXX move palette to bottom panel for HexGrid

// input states:
//	- erase_only; default state
//		- erase_or_paint: tile selected
//		- erasing: right down
//		- selecting: shift+left down
//		- move: EditorControl signal (has selection)
//		- clone: EditorControl signal (has selection)
//	- erase_or_paint:
//		- erase_only: tile selection cleared
//		- erase_only: escape pressed
//		- painting: left down
//		- erasing: right down
//		- selecting: shift+left down
//		- move: EditorControl signal (has selection)
//		- clone: EditorControl signal (has selection)
//	- painting: actively painting tiles with mouse movement
//		- erase_or_paint: left mouse up
//	- erasing: actively erasing tiles with mouse movement
//		- erase_or_paint: right mouse up && tile selected
//		- erase_only: right mouse up && no tile selected
//	- selecting:
//		- erase_or_paint: left mouse up && tile selected
//		- erase_or_paint: escape down && tile selected
//		- erase_only: left mouse up && no tile selected
//		- erase_only: escape down && no tile selected
//	- move:
//		- erase_or_paint: left mouse up && tile selected
//		- erase_or_paint: escape down && tile selected
//		- erase_only: left mouse up && no tile selected
//		- erase_only: escape down && no tile selected
//	- clone
//		- erase_or_paint: left mouse up && tile selected
//		- erase_or_paint: escape down && tile selected
//		- erase_only: left mouse up && no tile selected
//		- erase_only: escape down && no tile selected
//
// input handling
// - return pass if no hex_map
// - input contains a mouse position `InputEventMouse.get_position()`
//		- update editor_cursor position
//		- (selecting) update selection
//		- (painting) paint tile if changed
//		- (erase) erase tile
// - input is a mouse button event
//		- SHIFT + wheel-up: plane up
//		- SHIFT + wheel-down: plane down
//		- ALT+CTRL + pan gesture: plane up/down
//		- left-button down:
//			- (tile) begin painting tiles
//			- (move) place tiles, reset cursor
//			- (clone) place tiles, reset cursor
//			- SHIFT: begin selection
//				- save active selection for undo-redo
//				- hide cursor tiles
//				- set selection begin/end anc selection active
//				- update menu to enable selection options
//			- CTRL: pick tile
//				- get cell item & orientation
//				- set cell item in mesh palette
//				- set cursor orientation
//		- left-button up:
//			- (painting) complete painting tiles
//				- create undo-redo action for changes (MergeMode::MERGE_ALL)
//			- (selecting) complete selection
//				- create undo/redo with the last selection
//				- show the editor cursor
//		- right-button down:
//			- begin clear tiles
//		- right-button up:
//			- (erasing) complete erasing tiles
//				- create undo-redo action for changes (MergeMode::MERGE_ALL)
//	- input is a key event
//		- escape down
//			- (selecting) cancel selection
//				- clear current selection
//				- restore the previous selection if one was active
//			- (move)
//				- restore the tiles to their origin
//				- reset cursor to mesh palette selection
//			- (clone)
//				- reset cursor to mesh palette selection
//			- (painting) ignore; need to preserve mesh tile
//			- (erase) ignore
//			- (active selection) clear selection
//			- (tile) clear tile selection
//		- other keys
//			- editor_control->handle_keypress()
//

// Figure out how to add EditorSetting for HexMapEditorPlugin
// Break out MeshLibraryPicker (VBox)
// Break out ContextMenu (HBox); signals back for control
//
// Signals or function calls to EditorPlugins?
//
// XXX BUG while shift-click dragging, if you drag outside of the pane, will
// translate the node we're editing. -- fixed

// EditorPlugin
//	- has get_editor_interface()
//	- needs EditorInterface for shortcuts & settings
//	- handle viewpane input
//		- mouse movement move cursor
//		- clicks place tiles, do selection
//	- 3d view output
//		- draw grid
//		- draw cursor
//	- input handling
//		- input modes
//			- paint
//				- only when tile selected
//			- clear
//				- right-click if in paint mode
//			- selecting
//			- move
//				- update cursor for selection
//				- clear selected tiles
//				- clear selection
//				- cancel restores cleared tiles, and selection
//			- duplicate
//				- with selection
//					- save duplicate last source
//					- cancel restores selection; clears last source
//				- without selection & without last source
//					- disabled
//				- without selection & with last source
//					- cancel just resets the cursor state
//
// SelectionManager
//	- set_begin(pos)
//	- set_end(pos)
//	- show()
//	- hide()
//	- is_visible()
//	- get_cells()
//	- get_begin()
//	- get_end()
//
//
// Merge GridManager & TileCursor?
//	- move_raycast(from, direction)
//		- calculate edit plane intercept and get cellid from hexmap
//		- move cursor & grid based on pointer cell
//	- set_axis(axis)
//		- switch edit plane
//	- set_depth(depth)
//		- translate edit plane
//	- redraw()
//		- translate cursor meshes
//		- called after setting tile meshes
//	- set_tile(cell, tile, orientation)
//		- set a tile in cursor
//	- clear_tiles()
//	- set_orientation(orientation)
//		- rotate the cursor tiles as a group
//	- get_tiles()
//		- get all cursor cells with current cell id, tile, and orientation
//
//
// GridManager
//	- resize(float radius, float height)
//	- hide()/show()
//	- set_axis()
//	- set_depth()
//	- bool get_ray_intersect(Vector3 from, Vector3 normal, Vector3 *point)
//
//	- allocate mesh
//	- add points to the mesh based on cell size & axis
//		- mesh_clear() first
//	- allocate mesh instance
//	- hide/show mesh instance
//	- reposition the grid based on the pointer
//	- set grid depth
//
// TileCursor
//	- hide/show
//	- clear tiles
//		- clear()
//	- draw single tile
//		- set_tile(pos, index, rotation)
//	- draw multiple tiles
//		- set_tile(pos, index, rotation)
//	- rotate/flip cursor
//	- move cursor
//		- set_pointer_cell()
//		- update hex coord for cursor
//		- translate cursor tile instances
//	- get cursor location relative list of cells
//		- List<CellId, index, rotation> get_tiles()
//		- cursor.tiles.each { |tile| grid_map.set(tile) }
//	- internals
//		- need MeshLibrary and HexMap for positioning
//			- can do all with just HexMap
//		- multimesh may be overkill here
//		- set mesh layer higher than hexgrid layer
//		- dynamically calculate center during `set_tile()`?
//	- XXX need to figure out cursor positioning vs pointer
//		- center entire cursor on pointer cell
//			- can look like some cells deleted when move selected
//			- could be fixed with a highlight box around cursor tiles?
//		- position relative to pointer cell
//			- move selected leaves cells where they are; may appear to not be
//			  working, except for the cleared selection
//
// MeshLibraryPicker (VBox)
//	- Needs Ref<Theme> to get icons
//	- Hbox
//		- LineEdit (filter)
//		- Button (thumbnail)
//		- Button (list)
//  - HSlider (thumbnail size)
//	- ItemList (mesh library palette)
//
// ContextMenu
//	- to get settings/shortcuts need EditorSettings || ProjectSettings
//	- Floor/Layer
//	- Axis setting
//		- maintain per-axis floor here; just signal plane_changed(axis, floor)
//	- Tile/Selection rotation; cursor_changed(orientation?)
//	- Deselect; "select_none"
//	- Fill Selection; "selection_fill"
//	- Copy Selection; "selection_copy"
//	- Cut Selection;  "selection_cut"
//	- enable/disable selection-based menu options
//
// Signals
//  - HexMap
//		- cell_size_changed -> redraw grids, cursor
//		- cell_shape_changed -> redraw grids, cursor
//		- mesh_library_changed -> update palette
//		- map_changed(pos. old, new)
//  - MeshLibrary changed -> update palette
//	- MeshLibraryPicker
//		- Search Box
//			- text_changed -> filter ItemList
//			- gui_input -> up/down/page_up/page_down redirect to ItemList for
//			  selection
//		- Thumbnail Button "pressed" -> set ItemList display mode
//		- List Button "pressed" -> set ItemList display mode
//		- Size Slider "value_changed" -> set ItemList icon size
//		- ItemList "gui_input" -> ctrl + mouse wheel size adjustments
//		- item_selected(index) -> set cursor model
//	- ContextMenu
//		- Floor
//			- value_changed -> to EditorPlugin
//			- mouse_exited -> release floor edit focus
//		- Options Dropdown Menu
//			- id_pressed -> process, then issue "changed"
//		- changed(axis, floor)
//
class HexMapEditorPlugin : public EditorPlugin {
	GDCLASS(HexMapEditorPlugin, EditorPlugin);

	HexMap *hex_map = nullptr;
	Ref<MeshLibrary> mesh_library = nullptr;
	MeshLibraryPalette *mesh_palette = nullptr;
	EditorControl *editor_control = nullptr;
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
		TileOrientation orig_orientation;
		int new_tile = 0;
		TileOrientation new_orientation;
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

	double accumulated_floor_delta = 0.0; // used for touch/drag input

	void commit_cell_changes(String);

	void _update_mesh_library();

	void cell_size_changed(Vector3 cell_size);
	// callbacks used by signals from EditorControl
	void tile_changed(int p_mesh_id);
	void plane_changed(int p_axis);
	void axis_changed(int p_axis);
	void cursor_changed(Variant p_orientation);

	// manage selection
	void deselect_cells();
	void _deselect_cells();
	void _select_cell(Vector3i);

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

	HexMapEditorPlugin();
	~HexMapEditorPlugin();
};

#endif
