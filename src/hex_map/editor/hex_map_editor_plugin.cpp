#include "hex_map/hex_map_cell_id.h"
#include "profiling.h"
#ifdef TOOLS_ENABLED
#include "editor_control.h"
#include "editor_cursor.h"
#include "godot_cpp/classes/global_constants.hpp"
#include "godot_cpp/classes/input_event_mouse.hpp"
#include "godot_cpp/classes/undo_redo.hpp"
#include "godot_cpp/core/error_macros.hpp"
#include "godot_cpp/variant/callable_method_pointer.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include "hex_map_editor_plugin.h"
#include "selection_manager.h"

#include <cassert>
#include <cstdint>
#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/control.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/editor_undo_redo_manager.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/input_event_mouse_button.hpp>
#include <godot_cpp/classes/input_event_mouse_motion.hpp>
#include <godot_cpp/classes/input_event_pan_gesture.hpp>
#include <godot_cpp/classes/line_edit.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/popup_menu.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shortcut.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/v_separator.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

// XXX selecting too many cells then deleting causes cpu spin/hang
// XXX middle click in orthographic axis view moves camera; block that
void HexMapEditorPlugin::commit_cell_changes(String desc) {
	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action(desc);
	for (const CellChange &change : cells_changed) {
		undo_redo->add_do_method(hex_map,
				"set_cell_item",
				change.cell_id,
				change.new_tile,
				change.new_orientation);
	}
	auto end = --cells_changed.begin();
	for (auto iter = --cells_changed.end(); iter != end; --iter) {
		const CellChange &change = *iter;
		undo_redo->add_undo_method(hex_map,
				"set_cell_item",
				change.cell_id,
				change.orig_tile,
				change.orig_orientation);
	}
	undo_redo->commit_action();

	cells_changed.clear();
}

void HexMapEditorPlugin::deselect_cells() {
	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action("HexMap: deselect cells");

	undo_redo->add_do_method(this, "deselect_cells");

	// clear and select cells
	undo_redo->add_undo_method(this, "deselect_cells");
	for (const HexMapCellId &cell : selection_manager->get_cells()) {
		undo_redo->add_undo_method(this, "select_cell", (Vector3i)cell);
	}

	undo_redo->commit_action();
}

void HexMapEditorPlugin::_deselect_cells() {
	selection_manager->clear();
	editor_control->update_selection_menu(false);
}

void HexMapEditorPlugin::_select_cell(Vector3i cell) {
	selection_manager->set_cell(cell);
	// XXX slow, only need to do this once
	editor_control->update_selection_menu(true);
}

void HexMapEditorPlugin::selection_clear() {
	ERR_FAIL_COND_MSG(selection_manager->is_empty(), "no cells selected");

	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action("HexMap: clear selected tiles");

	for (const HexMapCellId cell : selection_manager->get_cells()) {
		undo_redo->add_do_method(
				hex_map, "set_cell_item", cell, HexMap::INVALID_CELL_ITEM);
		undo_redo->add_undo_method(hex_map,
				"set_cell_item",
				cell,
				hex_map->get_cell_item(cell),
				hex_map->get_cell_item_orientation(cell));
	}

	undo_redo->commit_action();
}

void HexMapEditorPlugin::selection_fill() {
	ERR_FAIL_COND_MSG(selection_manager->is_empty(), "no cells selected");

	List<EditorCursor::CursorCell> tiles = editor_cursor->get_tiles();
	ERR_FAIL_COND_MSG(
			tiles.size() > 1, "cursor has more than one tile; need one");
	ERR_FAIL_COND_MSG(tiles.size() == 0, "cursor has no tiles; need one");
	EditorCursor::CursorCell &tile = tiles[0];

	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action("HexMap: fill selected tiles");

	Array cells;
	for (const HexMapCellId cell : selection_manager->get_cells()) {
		cells.push_back((Vector3i)cell);
		undo_redo->add_undo_method(hex_map,
				"set_cell_item_v",
				cell,
				hex_map->get_cell_item(cell),
				hex_map->get_cell_item_orientation(cell));
	}
	undo_redo->add_do_method(
			hex_map, "fill_cells", cells, tile.tile, tile.orientation);

	auto prof = profiling_begin("commit action");
	auto start = std::chrono::high_resolution_clock::now();
	undo_redo->commit_action();
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = end - start;
	std::cout << "fill cells took " << elapsed.count() << " seconds"
			  << std::endl;

	const int iters = 10000000;
	Variant v = Vector3i();
	start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < iters; i++) {
		Vector3i x = v;
	}
	end = std::chrono::high_resolution_clock::now();
	elapsed = end - start;
	std::cout << "Variant -> Vector3i took " << elapsed.count() << " seconds"
			  << std::endl;

	Vector3i x = v;
	start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < iters; i++) {
		HexMapCellId c = x;
	}
	end = std::chrono::high_resolution_clock::now();
	elapsed = end - start;
	std::cout << "Vector -> HexMapCellId took " << elapsed.count()
			  << " seconds" << std::endl;

	start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < iters; i++) {
		HexMapCellId c = (Vector3i)v;
	}
	end = std::chrono::high_resolution_clock::now();
	elapsed = end - start;
	std::cout << "Variant -> Variant -> HexMapCellId took " << elapsed.count()
			  << " seconds" << std::endl;
}

void HexMapEditorPlugin::copy_selection_to_cursor() {
	ERR_FAIL_COND(editor_cursor == nullptr);

	// make the current pointer cell the origin cell in the cursor.
	editor_cursor->clear_tiles();
	editor_control->reset_orientation();

	Vector<HexMapCellId> cells = selection_manager->get_cells();
	if (cells.is_empty()) {
		return;
	}
	HexMapCellId center = selection_manager->get_center();

	for (const HexMapCellId &cell : cells) {
		int tile = hex_map->get_cell_item(cell);
		if (tile == HexMap::INVALID_CELL_ITEM) {
			continue;
		}
		editor_cursor->set_tile(
				cell - center, tile, hex_map->get_cell_item_orientation(cell));
	}
	editor_cursor->update(true);
	input_state = INPUT_STATE_MOVING;
}

void HexMapEditorPlugin::selection_clone() {
	ERR_FAIL_COND_MSG(input_state != INPUT_STATE_DEFAULT,
			"HexMap: cannot clone cells during another process");
	ERR_FAIL_COND_MSG(selection_manager->is_empty(),
			"HexMap: no cells selected; cannot clone");

	copy_selection_to_cursor();

	// save off selection for cancel, then clear it
	last_selection = selection_manager->get_cells();
	_deselect_cells();

	input_state = INPUT_STATE_CLONING;
}

void HexMapEditorPlugin::selection_clone_cancel() {
	selection_manager->set_cells(last_selection);
	last_selection.clear();

	editor_control->update_selection_menu(true);
	editor_control->reset_orientation();

	editor_cursor->clear_tiles();
	editor_cursor->set_tile(HexMapCellId(), mesh_palette->get_mesh());
	editor_cursor->update(true);
}

void HexMapEditorPlugin::selection_clone_apply() {
	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action("HexMap: clone selected tiles",
			godot::UndoRedo::MERGE_DISABLE,
			hex_map);
	undo_redo->add_do_method(this, "deselect_cells");

	for (const auto &cell : editor_cursor->get_tiles()) {
		undo_redo->add_do_method(
				this, "select_cell", (Vector3i)cell.cell_id_live);
		undo_redo->add_do_method(hex_map,
				"set_cell_item",
				cell.cell_id_live,
				cell.tile,
				cell.orientation);
		undo_redo->add_undo_method(hex_map,
				"set_cell_item",
				cell.cell_id_live,
				hex_map->get_cell_item(cell.cell_id_live),
				hex_map->get_cell_item_orientation(cell.cell_id_live));
	}

	undo_redo->add_undo_method(this, "deselect_cells");
	for (const HexMapCellId &cell_id : last_selection) {
		undo_redo->add_undo_method(this, "select_cell", (Vector3i)cell_id);
	}
	last_selection.clear();

	undo_redo->commit_action();

	editor_control->reset_orientation();
	editor_cursor->clear_tiles();
	editor_cursor->set_tile(HexMapCellId(), mesh_palette->get_mesh());
	editor_cursor->update(true);
}

void HexMapEditorPlugin::selection_move() {
	ERR_FAIL_COND_MSG(input_state != INPUT_STATE_DEFAULT,
			"HexMap: cannot move cells during another process");
	ERR_FAIL_COND_MSG(selection_manager->is_empty(),
			"HexMap: no cells selected; cannot move");
	assert(cells_changed.is_empty() && "cell_changed contains stale values");

	copy_selection_to_cursor();
	last_selection = selection_manager->get_cells();

	for (const Vector3i cell_id : selection_manager->get_cells()) {
		cells_changed.push_back(CellChange{
				.cell_id = cell_id,
				.orig_tile = hex_map->get_cell_item(cell_id),
				.orig_orientation =
						hex_map->get_cell_item_orientation(cell_id),
		});
		hex_map->set_cell_item(cell_id, HexMap::INVALID_CELL_ITEM);
	}
	_deselect_cells();

	input_state = INPUT_STATE_MOVING;
}

void HexMapEditorPlugin::selection_move_cancel() {
	for (const CellChange &change : cells_changed) {
		hex_map->set_cell_item(
				change.cell_id, change.orig_tile, change.orig_orientation);
	}
	cells_changed.clear();

	selection_manager->set_cells(last_selection);
	last_selection.clear();

	editor_control->update_selection_menu(true);
	editor_control->reset_orientation();

	editor_cursor->clear_tiles();
	editor_cursor->set_tile(HexMapCellId(), mesh_palette->get_mesh());
	editor_cursor->update(true);
}

void HexMapEditorPlugin::selection_move_apply() {
	EditorUndoRedoManager *undo_redo = get_undo_redo();
	undo_redo->create_action("HexMap: move selected tiles",
			godot::UndoRedo::MERGE_DISABLE,
			hex_map);

	undo_redo->add_do_method(this, "deselect_cells");

	// clear original cells first
	for (const CellChange &change : cells_changed) {
		undo_redo->add_do_method(hex_map,
				"set_cell_item",
				change.cell_id,
				HexMap::INVALID_CELL_ITEM);
	}

	// set the new cells, and start the undo restoring the new cells
	for (const auto &cell : editor_cursor->get_tiles()) {
		undo_redo->add_do_method(this, "select_cell", cell.cell_id_live);
		undo_redo->add_do_method(hex_map,
				"set_cell_item",
				cell.cell_id_live,
				cell.tile,
				cell.orientation);
		undo_redo->add_undo_method(hex_map,
				"set_cell_item",
				cell.cell_id_live,
				hex_map->get_cell_item(cell.cell_id_live),
				hex_map->get_cell_item_orientation(cell.cell_id_live));
	}

	// undo the clearing of the original cells
	for (const CellChange &change : cells_changed) {
		undo_redo->add_undo_method(hex_map,
				"set_cell_item",
				change.cell_id,
				change.orig_tile,
				change.orig_orientation);
	}
	cells_changed.clear();

	// update selection undo/redo
	undo_redo->add_undo_method(this, "deselect_cells");
	for (const HexMapCellId &cell_id : last_selection) {
		undo_redo->add_undo_method(this, "select_cell", (Vector3i)cell_id);
	}
	last_selection.clear();

	undo_redo->commit_action();

	editor_control->reset_orientation();
	editor_cursor->clear_tiles();
	editor_cursor->set_tile(HexMapCellId(), mesh_palette->get_mesh());
	editor_cursor->update(true);
}

bool HexMapEditorPlugin::_handles(Object *p_object) const {
	return p_object->is_class("HexMap");
}

void HexMapEditorPlugin::_make_visible(bool p_visible) {
	if (p_visible) {
		editor_control->show();
		mesh_palette->show();
		set_process(true);
	} else {
		editor_control->hide();
		mesh_palette->hide();
		set_process(false);
	}
}

int32_t HexMapEditorPlugin::_forward_3d_gui_input(Camera3D *p_camera,
		const Ref<InputEvent> &p_event) {
	if (!hex_map) {
		return EditorPlugin::AFTER_GUI_INPUT_PASS;
	}

	// try to handle any key event as a shortcut first
	//
	// XXX need to make sure keypress events aren't passed through while we're
	// performing an action.  This can happen right now with paint, move, etc
	// by hitting `G` becuase there's no selection.
	//
	Ref<InputEventKey> key_event = p_event;
	if (key_event.is_valid() && editor_control->handle_keypress(key_event)) {
		return AFTER_GUI_INPUT_STOP;
	}
	bool escape_pressed = key_event.is_valid() &&
			key_event->get_keycode() == Key::KEY_ESCAPE &&
			key_event->is_pressed();

	// if it's any type of mouse event, update the editor cursor
	Ref<InputEventMouse> mouse_event = p_event;
	if (mouse_event.is_valid()) {
		editor_cursor->update(p_camera, mouse_event->get_position(), nullptr);
	}

	// grab the common mouse button up/downs to simplify the switch statement
	// below
	Ref<InputEventMouseButton> mouse_button_event = p_event;
	bool mouse_left_pressed = mouse_button_event.is_valid() &&
			mouse_button_event->get_button_index() ==
					MouseButton::MOUSE_BUTTON_LEFT &&
			mouse_button_event->is_pressed();
	bool mouse_left_released = mouse_button_event.is_valid() &&
			mouse_button_event->get_button_index() ==
					MouseButton::MOUSE_BUTTON_LEFT &&
			mouse_button_event->is_released();
	bool mouse_right_pressed = mouse_button_event.is_valid() &&
			mouse_button_event->get_button_index() ==
					MouseButton::MOUSE_BUTTON_RIGHT &&
			mouse_button_event->is_pressed();
	bool mouse_right_released = mouse_button_event.is_valid() &&
			mouse_button_event->get_button_index() ==
					MouseButton::MOUSE_BUTTON_RIGHT &&
			mouse_button_event->is_released();
	bool shift_pressed = mouse_button_event.is_valid() &&
			mouse_button_event->is_shift_pressed();

	switch (input_state) {
		// transitions:
		// - SELECTING (shift + left button down)
		// - PAINTING (left button down)
		// - ERASING (right button down)
		// actions:
		// - clear selection (escape && have selection)
		// - clear cursor (escape)
		case INPUT_STATE_DEFAULT:
			if (mouse_left_pressed && shift_pressed) {
				editor_cursor->hide();
				input_state = INPUT_STATE_SELECTING;
				last_selection = selection_manager->get_cells();
				selection_anchor = editor_cursor->get_pos();
				selection_manager->clear();
				selection_manager->set_cell(editor_cursor->get_cell());
				return AFTER_GUI_INPUT_STOP;
			} else if (mouse_left_pressed && !editor_cursor->is_empty()) {
				input_state = INPUT_STATE_PAINTING;
				return AFTER_GUI_INPUT_STOP;
			} else if (mouse_right_pressed) {
				editor_cursor->hide();
				input_state = INPUT_STATE_ERASING;
				return AFTER_GUI_INPUT_STOP;
			} else if (escape_pressed) {
				if (!selection_manager->is_empty()) {
					deselect_cells();
				} else {
					mesh_palette->clear_selection();
				}
				return AFTER_GUI_INPUT_STOP;
			}
			break;

		// transitions:
		// - DEFAULT (left button up)
		// actions:
		// - change cells, and add them to the cells_changed list
		// - commit changes on mouse up
		case INPUT_STATE_PAINTING: {
			HexMapCellId cell_id = editor_cursor->get_cell();
			int tile = hex_map->get_cell_item(cell_id);
			int orientation = hex_map->get_cell_item_orientation(cell_id);

			EditorCursor::CursorCell cell = editor_cursor->get_first_tile();
			if (cell.tile != tile || cell.orientation != orientation) {
				cells_changed.push_back(CellChange{
						.cell_id = cell_id,
						.orig_tile = tile,
						.orig_orientation = orientation,
						.new_tile = cell.tile,
						.new_orientation = cell.orientation,
				});
				hex_map->set_cell_item(cell_id, cell.tile, cell.orientation);
			}
			if (mouse_left_released) {
				commit_cell_changes("HexMap: paint cells");
				input_state = INPUT_STATE_DEFAULT;
			}
			break;
		}
		// transitions:
		// - DEFAULT (left button up)
		// actions:
		// actions:
		// - change cells, and add them to the cells_changed list
		// - commit changes on mouse up
		case INPUT_STATE_ERASING: {
			HexMapCellId cell_id = editor_cursor->get_cell();
			int tile = hex_map->get_cell_item(cell_id);
			if (tile != -1) {
				cells_changed.push_back(CellChange{
						.cell_id = cell_id,
						.orig_tile = tile,
						.orig_orientation =
								hex_map->get_cell_item_orientation(cell_id),
						.new_tile = -1,
				});
				hex_map->set_cell_item(cell_id, HexMap::INVALID_CELL_ITEM);
			}
			if (mouse_right_released) {
				commit_cell_changes("HexMap: erase cells");
				input_state = INPUT_STATE_DEFAULT;
				editor_cursor->show();
			}
			break;
		}
		// transitions:
		// - DEFAULT (left button up)
		// - DEFAULT (escape); clear selection
		// actions:
		// - update selection (mouse event)
		case INPUT_STATE_SELECTING:
			if (mouse_event.is_valid()) {
				// XXX selection along Q/S axis currently selects cube instead
				// of plane.
				auto cells = hex_map->local_region_to_map(
						selection_anchor, editor_cursor->get_pos());
				selection_manager->clear();
				selection_manager->set_cells(cells);
			}
			if (mouse_left_released) {
				EditorUndoRedoManager *undo_redo = get_undo_redo();
				undo_redo->create_action("HexMap: select cells");
				// do will set the selection
				undo_redo->add_do_method(this, "deselect_cells");
				for (const HexMapCellId &cell :
						selection_manager->get_cells()) {
					undo_redo->add_do_method(
							this, "select_cell", (Vector3i)cell);
				}
				// undo will restore last selection
				undo_redo->add_undo_method(this, "deselect_cells");
				for (const HexMapCellId &cell : last_selection) {
					undo_redo->add_undo_method(
							this, "select_cell", (Vector3i)cell);
				}
				undo_redo->commit_action();

				editor_cursor->show();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			} else if (escape_pressed) {
				selection_manager->clear();
				selection_manager->set_cells(last_selection);
				editor_cursor->show();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			}
			break;
		case INPUT_STATE_MOVING:
			if (mouse_left_pressed) {
				selection_move_apply();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			} else if (escape_pressed) {
				selection_move_cancel();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			}
			break;
		case INPUT_STATE_CLONING:
			if (mouse_left_pressed) {
				selection_clone_apply();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			} else if (escape_pressed) {
				selection_clone_cancel();
				input_state = INPUT_STATE_DEFAULT;
				return AFTER_GUI_INPUT_STOP;
			}
			break;
	}

	return EditorPlugin::AFTER_GUI_INPUT_PASS;
}

void HexMapEditorPlugin::_update_mesh_library() {
	ERR_FAIL_NULL(hex_map);

	mesh_library = hex_map->get_mesh_library();
	mesh_palette->set_mesh_library(mesh_library);

	if (editor_cursor) {
		editor_control->reset_orientation();
		editor_cursor->clear_tiles();
	}
}

void HexMapEditorPlugin::_edit(Object *p_object) {
	if (hex_map) {
		// save off the current floors
		hex_map->set_meta("_editor_floors_", editor_control->get_planes());

		// disconnect signals
		hex_map->disconnect("cell_size_changed",
				callable_mp(this, &HexMapEditorPlugin::cell_size_changed));
		hex_map->disconnect("mesh_library_changed",
				callable_mp(this, &HexMapEditorPlugin::_update_mesh_library));

		// clear the mesh library
		mesh_library = Ref<MeshLibrary>();
		mesh_palette->set_mesh_library(mesh_library);

		// reset the selection menu
		editor_control->update_selection_menu(false);

		// clear a couple of other state tracking variables
		last_selection.clear();
		cells_changed.clear();

		// we use delete here because EditorCursor is not a Godot Object
		// subclass, so its destructor is not called with memfree().
		delete editor_cursor;
		editor_cursor = nullptr;

		delete selection_manager;
		selection_manager = nullptr;
	}

	hex_map = Object::cast_to<HexMap>(p_object);

	if (!hex_map) {
		set_process(false);
		return;
	}

	// not a godot Object subclass, so `new` instead of `memnew()`
	editor_cursor = new EditorCursor(hex_map);
	selection_manager = new SelectionManager(hex_map);
	Array floors = hex_map->get_meta("_editor_floors_", Array());
	if (floors.size() == 4) {
		editor_control->set_planes(floors);
	}

	set_process(true);

	hex_map->connect("cell_size_changed",
			callable_mp(this, &HexMapEditorPlugin::cell_size_changed));
	hex_map->connect("mesh_library_changed",
			callable_mp(this, &HexMapEditorPlugin::_update_mesh_library));
	_update_mesh_library();
}

void HexMapEditorPlugin::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_PROCESS: {
			// we cache the global transform of the HexMap.  When the global
			// transform changes, we'll update our visuals accordingly.
			static Transform3D cached_transform;
			ERR_FAIL_COND_MSG(hex_map == nullptr,
					"HexMapEditorPlugin process without HexMap");

			// if the transform of the HexMap node has been changed, update
			// the grid.
			Transform3D transform = hex_map->get_global_transform();
			if (transform != cached_transform) {
				cached_transform = transform;
				editor_cursor->update(true);
				selection_manager->redraw_selection();
			}
		} break;

		case NOTIFICATION_APPLICATION_FOCUS_OUT: {
			// If the user switches applications while the mouse button is
			// pressed, the state will remain pressed when they return to the
			// editor.  This will result in drawing tiles when the mouse button
			// isn't pressed.
			//
			// Workaround is to fire a release event when editor loses focus.
			if (input_state == INPUT_STATE_PAINTING) {
				Ref<InputEventMouseButton> release;
				release.instantiate();
				release->set_button_index(MouseButton::MOUSE_BUTTON_LEFT);
				_forward_3d_gui_input(nullptr, release);
			} else if (input_state == INPUT_STATE_ERASING) {
				Ref<InputEventMouseButton> release;
				release.instantiate();
				release->set_button_index(MouseButton::MOUSE_BUTTON_RIGHT);
				_forward_3d_gui_input(nullptr, release);
			}
		} break;
	}
}

void HexMapEditorPlugin::cell_size_changed(Vector3 cell_size) {
	ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
	editor_cursor->cell_size_changed();
	selection_manager->redraw_selection();
}

void HexMapEditorPlugin::tile_changed(int p_mesh_id) {
	ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
	editor_cursor->clear_tiles();
	if (p_mesh_id != -1) {
		editor_cursor->set_tile(Vector3i(), p_mesh_id);
		editor_cursor->update(true);
	}
	editor_control->reset_orientation();
}

void HexMapEditorPlugin::plane_changed(int p_plane) {
	ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
	editor_cursor->set_depth(p_plane);
}

void HexMapEditorPlugin::axis_changed(int p_axis) {
	ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
	editor_cursor->set_axis((EditorControl::EditAxis)p_axis);
}

void HexMapEditorPlugin::cursor_changed(Variant p_orientation) {
	ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
	editor_cursor->set_orientation(p_orientation);
}

void HexMapEditorPlugin::_bind_methods() {
	ClassDB::bind_method(
			D_METHOD("deselect_cells"), &HexMapEditorPlugin::_deselect_cells);
	ClassDB::bind_method(D_METHOD("select_cell", "cell"),
			&HexMapEditorPlugin::_select_cell);
}

HexMapEditorPlugin::HexMapEditorPlugin() {
	// palette
	mesh_palette = memnew(MeshLibraryPalette);
	mesh_palette->hide();
	mesh_palette->connect("mesh_changed",
			callable_mp(this, &HexMapEditorPlugin::tile_changed));
	add_control_to_container(
			CONTAINER_SPATIAL_EDITOR_SIDE_RIGHT, mesh_palette);

	// context menu in spatial editor
	editor_control = memnew(EditorControl);
	editor_control->hide();
	editor_control->connect("plane_changed",
			callable_mp(this, &HexMapEditorPlugin::plane_changed));
	editor_control->connect("axis_changed",
			callable_mp(this, &HexMapEditorPlugin::axis_changed));
	editor_control->connect("cursor_orientation_changed",
			callable_mp(this, &HexMapEditorPlugin::cursor_changed));
	editor_control->connect("deselect",
			callable_mp(this, &HexMapEditorPlugin::deselect_cells));
	editor_control->connect("selection_fill",
			callable_mp(this, &HexMapEditorPlugin::selection_fill));
	editor_control->connect("selection_clear",
			callable_mp(this, &HexMapEditorPlugin::selection_clear));
	editor_control->connect("selection_move",
			callable_mp(this, &HexMapEditorPlugin::selection_move));
	editor_control->connect("selection_clone",
			callable_mp(this, &HexMapEditorPlugin::selection_clone));
	add_control_to_container(CONTAINER_SPATIAL_EDITOR_MENU, editor_control);
}

HexMapEditorPlugin::~HexMapEditorPlugin() {
	ERR_FAIL_NULL(RenderingServer::get_singleton());

	if (editor_cursor != nullptr) {
		delete editor_cursor;
	}
	if (selection_manager != nullptr) {
		delete selection_manager;
	}
	// editor_control & mesh_palette are cleaned up automatically.
}
#endif
