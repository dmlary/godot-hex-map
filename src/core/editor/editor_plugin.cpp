#include "godot_cpp/variant/dictionary.hpp"
#ifdef TOOLS_ENABLED

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
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/input_event_mouse.hpp>
#include <godot_cpp/classes/input_event_mouse_button.hpp>
#include <godot_cpp/classes/input_event_mouse_motion.hpp>
#include <godot_cpp/classes/input_event_pan_gesture.hpp>
#include <godot_cpp/classes/line_edit.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/packed_scene.hpp>
#include <godot_cpp/classes/popup_menu.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shortcut.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/undo_redo.hpp>
#include <godot_cpp/classes/v_separator.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector3i.hpp>

#include "core/cell_id.h"
#include "core/editor/editor_cursor.h"
#include "core/hex_map_node.h"
#include "core/tile_orientation.h"
#include "editor_plugin.h"
#include "helpers.h"
#include "profiling.h"

using EditAxis = EditorCursor::EditAxis;

// XXX selecting too many cells then deleting causes cpu spin/hang
// XXX middle click in orthographic axis view moves camera; block that
void HexMapNodeEditorPlugin::commit_cell_changes(String desc) {
    auto change_count = cells_changed.size();
    Array do_list, undo_list;

    do_list.resize(change_count * 3);
    undo_list.resize(change_count * 3);
    for (int i = 0; i < change_count; i++) {
        const CellChange &change = cells_changed[i];
        int base = i * 3;
        do_list[base] = (Vector3i)change.cell_id;
        do_list[base + 1] = change.new_tile;
        do_list[base + 2] = change.new_orientation;
        undo_list[base] = (Vector3i)change.cell_id;
        undo_list[base + 1] = change.orig_tile;
        undo_list[base + 2] = change.orig_orientation;
    }

    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action(desc);
    undo_redo->add_do_method(hex_map, "set_cells", do_list);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_list);
    undo_redo->commit_action();

    cells_changed.clear();
}

void HexMapNodeEditorPlugin::deselect_cells() {
    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action("HexMap: deselect cells");

    undo_redo->add_do_method(this, "deselect_cells");

    // clear and select cells
    undo_redo->add_undo_method(
            this, "set_selection_v", selection_manager->get_cells_v());

    undo_redo->commit_action();
}

void HexMapNodeEditorPlugin::_deselect_cells() {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");
    selection_manager->clear();
    emit_signal("selection_changed", false);
}

void HexMapNodeEditorPlugin::_select_cell(Vector3i cell) {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");
    selection_manager->add_cell(cell);
    emit_signal("selection_changed", true);
}

void HexMapNodeEditorPlugin::_set_selection_v(Array cells) {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");

    selection_manager->set_cells(cells);
    emit_signal("selection_changed", !cells.is_empty());
}

void HexMapNodeEditorPlugin::selection_clear() {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");

    auto cells = selection_manager->get_cells_v();

    // build the set_cells() argument to restore the original cells
    Array undo_list = hex_map->get_cells(cells);

    // build the set_cells() argument to clear the cells
    Array do_list;
    int cell_count = cells.size();
    do_list.resize(cell_count * 3);
    for (int i = 0; i < cell_count; i++) {
        int base = i * 3;
        do_list[base] = cells[i];
        do_list[base + 1] = HexMapNode::INVALID_CELL_ITEM;
        do_list[base + 2] = 0;
    }

    // add it to undo/redo and commit it
    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action("HexMap: clear selected tiles");
    undo_redo->add_do_method(hex_map, "set_cells", do_list);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_list);
    auto prof = profiling_begin("clear: commit action");
    undo_redo->commit_action();
}

void HexMapNodeEditorPlugin::selection_fill() {
    ERR_FAIL_COND_MSG(selection_manager->is_empty(), "no cells selected");

    ERR_FAIL_COND_MSG(editor_cursor->size() > 1,
            "cursor has more than one tile; need one");
    ERR_FAIL_COND_MSG(
            editor_cursor->size() == 0, "cursor has no tiles; need one");

    EditorCursor::CellState tile = editor_cursor->get_only_cell_state();
    auto cells = selection_manager->get_cells_v();

    // build the set_cells() argument to restore the original cells
    Array undo_list = hex_map->get_cells(cells);

    // build the set_cells() argument to clear the cells
    Array do_list;
    int cell_count = cells.size();
    do_list.resize(cell_count * 3);
    for (int i = 0; i < cell_count; i++) {
        int base = i * 3;
        do_list[base] = cells[i];
        do_list[base + 1] = tile.index;
        do_list[base + 2] = tile.orientation;
    }

    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action("HexMap: fill selected tiles");
    undo_redo->add_do_method(hex_map, "set_cells", do_list);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_list);
    auto prof = profiling_begin("fill: commit action");
    undo_redo->commit_action();
}

void HexMapNodeEditorPlugin::copy_selection_to_cursor() {
    ERR_FAIL_COND(editor_cursor == nullptr);

    // make the current pointer cell the origin cell in the cursor.
    editor_cursor->clear_tiles();
    cursor_set_orientation(HexMapTileOrientation());

    Vector<HexMapCellId> cells = selection_manager->get_cells();
    if (cells.is_empty()) {
        return;
    }
    HexMapCellId center = selection_manager->get_center();

    for (const HexMapCellId &cell_id : cells) {
        auto [value, orientation] = hex_map->get_cell(cell_id);
        if (value == HexMapNode::INVALID_CELL_ITEM) {
            continue;
        }
        editor_cursor->set_tile(cell_id - center, value, orientation);
    }
    editor_cursor->update(true);
    input_state = INPUT_STATE_MOVING;
}

void HexMapNodeEditorPlugin::selection_clone() {
    ERR_FAIL_COND_MSG(input_state != INPUT_STATE_DEFAULT,
            "HexMap: cannot clone cells during another process");
    ERR_FAIL_COND_MSG(selection_manager->is_empty(),
            "HexMap: no cells selected; cannot clone");

    copy_selection_to_cursor();

    // save off selection for cancel, then clear it
    last_selection = selection_manager->get_cells_v();
    _deselect_cells();

    input_state = INPUT_STATE_CLONING;
}

void HexMapNodeEditorPlugin::selection_clone_cancel() {
    _set_selection_v(last_selection);
    last_selection.clear();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::selection_clone_apply() {
    Array do_list = editor_cursor->get_cells_v();
    Array do_select = editor_cursor->get_cell_ids_v();
    Array undo_list = hex_map->get_cells(do_select);
    Array undo_select = last_selection;
    last_selection = Array();

    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action("HexMap: clone selected tiles",
            godot::UndoRedo::MERGE_DISABLE,
            hex_map);
    undo_redo->add_do_method(hex_map, "set_cells", do_list);
    undo_redo->add_do_method(this, "set_selection_v", do_select);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_list);
    undo_redo->add_undo_method(this, "set_selection_v", undo_select);
    auto prof = profiling_begin("clone: commit action");
    undo_redo->commit_action();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::selection_move() {
    ERR_FAIL_COND_MSG(input_state != INPUT_STATE_DEFAULT,
            "HexMap: cannot move cells during another process");
    ERR_FAIL_COND_MSG(selection_manager->is_empty(),
            "HexMap: no cells selected; cannot move");
    assert(cells_changed.is_empty() && "cell_changed contains stale values");

    copy_selection_to_cursor();

    // save the original cell contents, then clear the cells
    last_selection = selection_manager->get_cells_v();
    move_source_cells = hex_map->get_cells(last_selection);
    for (const HexMapCellId &cell_id : selection_manager->get_cells()) {
        hex_map->set_cell(cell_id, HexMapNode::INVALID_CELL_ITEM);
    }

    // clear selection
    _deselect_cells();

    input_state = INPUT_STATE_MOVING;
}

void HexMapNodeEditorPlugin::selection_move_cancel() {
    hex_map->set_cells(move_source_cells);
    move_source_cells.clear();

    _set_selection_v(last_selection);
    last_selection.clear();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::selection_move_apply() {
    // first phase of redo is to clear the source cells
    int count = move_source_cells.size();
    Array do_set_cells;
    do_set_cells.resize(count * HexMapNode::CellArrayWidth);
    for (int i = 0; i < count; i++) {
        int base = i * HexMapNode::CellArrayWidth;
        do_set_cells[base] = move_source_cells[i];
        do_set_cells[base + 1] = HexMapNode::INVALID_CELL_ITEM;
    }
    // second phase is to set the destination cells
    do_set_cells.append_array(editor_cursor->get_cells_v());
    Array do_select = editor_cursor->get_cell_ids_v();

    // similarly for undo, restore the destination cells, then restore the
    // source cells
    Array undo_set_cells = hex_map->get_cells(editor_cursor->get_cell_ids_v());
    undo_set_cells.append_array(move_source_cells);
    Array undo_select = last_selection;

    move_source_cells = Array();
    last_selection = Array();

    EditorUndoRedoManager *undo_redo = get_undo_redo();
    undo_redo->create_action("HexMap: move selected tiles",
            godot::UndoRedo::MERGE_DISABLE,
            hex_map);
    undo_redo->add_do_method(hex_map, "set_cells", do_set_cells);
    undo_redo->add_do_method(this, "set_selection_v", do_select);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_set_cells);
    undo_redo->add_undo_method(this, "set_selection_v", undo_select);
    auto prof = profiling_begin("move: commit action");
    undo_redo->commit_action();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::rebuild_cursor() {
    editor_cursor->clear_tiles();
    cursor_set_orientation(HexMapTileOrientation());
    emit_signal("cursor_restore");
    editor_cursor->update(true);
}

void HexMapNodeEditorPlugin::_make_visible(bool p_visible) {
    set_process(p_visible);
}

int32_t HexMapNodeEditorPlugin::_forward_3d_gui_input(Camera3D *p_camera,
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

    // The escape key requires special handling.  In some cases this function
    // will consume it to cancel a move/clone, or clear the selection.  In
    // other cases, it will need to be passed to the subclass to clear the
    // current tile selection.  We pull the escape_pressed state here.
    bool escape_pressed = key_event.is_valid() &&
            key_event->get_keycode() == Key::KEY_ESCAPE &&
            key_event->is_pressed();

    // Pass keypress events to handle_keypress() to override editor default
    // keybindings.  The one exception is the escape key; we'll call
    // handle_keypress() with the event later if this function does not consume
    // it.
    if (key_event.is_valid() && key_event->is_pressed() && !escape_pressed &&
            handle_keypress(key_event) == EditorPlugin::AFTER_GUI_INPUT_STOP) {
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }

    // if it is any type of mouse event, update the editor cursor
    // Note: the check for a null camera is necessary because of the event we
    // generate during NOTIFICATION_APPLICATION_FOCUS_OUT
    Ref<InputEventMouse> mouse_event = p_event;
    if (mouse_event.is_valid() && p_camera != nullptr) {
        editor_cursor->update(p_camera, mouse_event->get_position(), nullptr);
    }

    // grab the common mouse button up/downs to simplify the conditions in the
    // switch statement below
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
        // DEFAULT transitions to the following states:
        // - SELECTING (shift + left button down)
        // - PAINTING (left button down)
        // - ERASING (right button down)
        // actions:
        // - clear selection (escape && have selection)
        // - clear cursor (escape)
        case INPUT_STATE_DEFAULT:
            if (mouse_left_pressed && shift_pressed) {
                editor_cursor->set_visibility(false);
                input_state = INPUT_STATE_SELECTING;
                last_selection = selection_manager->get_cells_v();
                selection_anchor = editor_cursor->get_pos();
                selection_manager->clear();
                selection_manager->add_cell(editor_cursor->get_cell());
                return AFTER_GUI_INPUT_STOP;
            } else if (mouse_left_pressed && !editor_cursor->is_empty()) {
                input_state = INPUT_STATE_PAINTING;
                return AFTER_GUI_INPUT_STOP;
            } else if (mouse_right_pressed) {
                editor_cursor->set_visibility(false);
                input_state = INPUT_STATE_ERASING;
                return AFTER_GUI_INPUT_STOP;
            } else if (escape_pressed) {
                if (!selection_manager->is_empty()) {
                    deselect_cells();
                    return AFTER_GUI_INPUT_STOP;
                }
                // allow keypress handler to PASS or STOP the escape key
                return handle_keypress(key_event);
            }
            break;

        // transitions:
        // - DEFAULT (left button up)
        // actions:
        // - change cells, and add them to the cells_changed list
        // - commit changes on mouse up
        case INPUT_STATE_PAINTING: {
            HexMapCellId cell_id = editor_cursor->get_cell();
            auto [tile, orientation] = hex_map->get_cell(cell_id);

            EditorCursor::CellState cell =
                    editor_cursor->get_only_cell_state();
            if (cell.index != tile || cell.orientation != orientation) {
                cells_changed.push_back(CellChange{
                        .cell_id = cell_id,
                        .orig_tile = tile,
                        .orig_orientation = orientation,
                        .new_tile = cell.index,
                        .new_orientation = cell.orientation,
                });
                hex_map->set_cell(cell_id, cell.index, cell.orientation);
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
        // - change cells, and add them to the cells_changed list
        // - commit changes on mouse up
        case INPUT_STATE_ERASING: {
            HexMapCellId cell_id = editor_cursor->get_cell();
            auto [tile, orientation] = hex_map->get_cell(cell_id);
            if (tile != -1) {
                cells_changed.push_back(CellChange{
                        .cell_id = cell_id,
                        .orig_tile = tile,
                        .orig_orientation = orientation,
                        .new_tile = -1,
                });
                hex_map->set_cell(cell_id, HexMapNode::INVALID_CELL_ITEM);
            }
            if (mouse_right_released) {
                commit_cell_changes("HexMap: erase cells");
                input_state = INPUT_STATE_DEFAULT;
                editor_cursor->set_visibility(true);
            }
            break;
        }
        // transitions:
        // - DEFAULT (left button up)
        // - DEFAULT (escape); clear selection
        // actions:
        // - update selection (mouse event)
        case INPUT_STATE_SELECTING:
            if (mouse_event.is_valid() && p_camera != nullptr) {
                // XXX look at moving this into SelectionManager
                Vector3 p1 = selection_anchor, p3 = editor_cursor->get_pos();
                Vector3 p2, p4;
                switch (editor_cursor->get_axis()) {
                    // when working on the Y axis, we want to snap the
                    // selection rectangle to 30 degree angles to better line
                    // up with the hex grid.
                    case EditAxis::AXIS_Y: {
                        real_t snap = Math_PI / 6;
                        real_t rot = p_camera->get_global_rotation().y;
                        real_t delta = snap * round(rot / snap);
                        Vector3 r1 = p1.rotated(Vector3(0, 1, 0), -delta);
                        Vector3 r3 = p3.rotated(Vector3(0, 1, 0), -delta);
                        p2 = Vector3(r1.x, r1.y, r3.z)
                                     .rotated(Vector3(0, 1, 0), delta);
                        p4 = Vector3(r3.x, r1.y, r1.z)
                                     .rotated(Vector3(0, 1, 0), delta);

                    } break;
                    case EditAxis::AXIS_Q:
                    case EditAxis::AXIS_R:
                    case EditAxis::AXIS_S:
                        p2 = Vector3(p1.x, p3.y, p1.z);
                        p4 = Vector3(p3.x, p1.y, p3.z);
                        break;
                }

                auto cells = hex_map->get_space().get_cell_ids_in_local_quad(
                        p1, p2, p3, p4);

                selection_manager->clear();
                selection_manager->set_cells(cells);
                emit_signal("selection_changed", true);
            }
            if (mouse_left_released) {
                auto profile = profiling_begin("complete selection");
                EditorUndoRedoManager *undo_redo = get_undo_redo();
                undo_redo->create_action("HexMap: select cells");
                // do method will apply the existing selection
                undo_redo->add_do_method(this,
                        "set_selection_v",
                        selection_manager->get_cells_v());

                undo_redo->add_undo_method(
                        this, "set_selection_v", last_selection);

                auto prof_commit =
                        profiling_begin("commit selection undo_redo");
                undo_redo->commit_action();

                editor_cursor->set_visibility(true);
                input_state = INPUT_STATE_DEFAULT;
                return AFTER_GUI_INPUT_STOP;
            } else if (escape_pressed) {
                selection_manager->clear();
                selection_manager->set_cells(last_selection);
                editor_cursor->set_visibility(true);
                input_state = INPUT_STATE_DEFAULT;
                return AFTER_GUI_INPUT_STOP;
            }
            break;

        // transitions:
        // - DEFAULT (left click, escape)
        // actions:
        // - apply selection move
        // - restore selected cells (cancel)
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

        // transitions:
        // - DEFAULT (left click, escape)
        // actions:
        // - apply selection clone
        // - cancel selection clone
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

// XXX subclass needs to update editor_cursor MeshLibrary when changes happen
// void HexMapNodeEditorPlugin::update_mesh_library() {
//     ERR_FAIL_NULL(hex_map);
//
//     mesh_library = hex_map->get_mesh_library();
//     bottom_panel->set("mesh_library", mesh_library);
//
//     if (editor_cursor) {
//         editor_cursor->clear_tiles();
//         editor_cursor->set_mesh_library(mesh_library);
//         editor_control->reset_orientation();
//         bottom_panel->call("clear_selection");
//         bottom_panel->call("update_mesh_list");
//     }
// }

void HexMapNodeEditorPlugin::_edit(Object *p_object) {
    if (hex_map) {
        // save off the editor state to the current node
        write_editor_state(hex_map);

        // disconnect signals
        hex_map->disconnect("cell_scale_changed",
                callable_mp(this, &HexMapNodeEditorPlugin::hex_space_changed));
        hex_map->disconnect("mesh_offset_changed",
                callable_mp(this, &HexMapNodeEditorPlugin::hex_space_changed));
        // XXX subclass must handle all MeshLibrary work
        // hex_map->disconnect("mesh_library_changed",
        //         callable_mp(
        //                 this,
        //                 &HexMapNodeEditorPlugin::update_mesh_library));

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

    hex_map = Object::cast_to<HexMapNode>(p_object);

    if (!hex_map) {
        set_process(false);
        return;
    }

    Ref<PackedScene> panel_scene = ResourceLoader::get_singleton()->load(
            "res://addons/hexmap/gui/"
            "hex_map_editor_bottom_panel.tscn");

    RID scenario = hex_map->get_world_3d()->get_scenario();

    selection_manager = new SelectionManager(scenario);
    selection_manager->set_space(hex_map->get_space());

    // not a godot Object subclass, so `new` instead of `memnew()`
    editor_cursor = new EditorCursor(scenario);
    editor_cursor->set_space(hex_map->get_space());
    editor_cursor->set_cells_visibility_callback(
            callable_mp(hex_map, &HexMapNode::set_cells_visibility));

    // load any saved editor state from the node
    read_editor_state(hex_map);
    editor_cursor->set_axis(edit_axis);
    editor_cursor->set_depth(edit_axis_depth[edit_axis]);

    set_process(true);

    hex_map->connect("cell_scale_changed",
            callable_mp(this, &HexMapNodeEditorPlugin::hex_space_changed));
    hex_map->connect("mesh_offset_changed",
            callable_mp(this, &HexMapNodeEditorPlugin::hex_space_changed));

    // XXX the subclass needs to handle this some way
    // hex_map->connect("mesh_library_changed",
    //         callable_mp(this,
    //         &HexMapNodeEditorPlugin::update_mesh_library));
    // update_mesh_library();
}

void HexMapNodeEditorPlugin::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_PROCESS: {
            // we cache the global transform of the HexMap.  When the global
            // transform changes, we'll update our visuals accordingly.
            static Transform3D cached_transform;
            ERR_FAIL_COND_MSG(hex_map == nullptr,
                    "HexMapNodeEditorPlugin process without HexMap");

            // if the transform of the HexMap node has been changed, update
            // the grid.
            Transform3D transform = hex_map->get_global_transform();
            if (transform != cached_transform) {
                cached_transform = transform;
                editor_cursor->set_space(hex_map->get_space());
                selection_manager->set_space(hex_map->get_space());
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

void HexMapNodeEditorPlugin::write_editor_state(HexMapNode *node) const {
    Dictionary state;
    state["__version"] = 1;
    state["edit_axis"] = (int)edit_axis;
    state["edit_axis_depth"] = edit_axis_depth;
    node->set_meta("_editor_state_", state);
}

void HexMapNodeEditorPlugin::read_editor_state(const HexMapNode *node) {
    // set state to sane values first
    edit_axis = EditAxis::AXIS_Y;
    edit_axis_depth.resize(4);
    edit_axis_depth.fill(0);

    Dictionary state = hex_map->get_meta("_editor_state_", Dictionary());
    int version = state.get("__version", 0);
    if (version >= 1) {
        edit_axis = EditAxis((int)state["edit_axis"]);
        edit_axis_depth = state["edit_axis_depth"];
    }
}

void HexMapNodeEditorPlugin::edit_plane_set_axis(EditorCursor::EditAxis axis) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    edit_axis = axis;
    editor_cursor->set_axis(axis);
    emit_signal("edit_plane_changed", axis, edit_axis_depth[axis]);
}

void HexMapNodeEditorPlugin::edit_plane_set_depth(int depth) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    edit_axis_depth[edit_axis] = depth;
    editor_cursor->set_depth(depth);
    emit_signal("edit_plane_changed", edit_axis, depth);
}

void HexMapNodeEditorPlugin::hex_space_changed() {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_space(hex_map->get_space());
    selection_manager->set_space(hex_map->get_space());
}

void HexMapNodeEditorPlugin::plane_changed(int p_plane) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_depth(p_plane);
}

void HexMapNodeEditorPlugin::axis_changed(int p_axis) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_axis((EditAxis)p_axis);
}

void HexMapNodeEditorPlugin::cursor_set_orientation(
        HexMapTileOrientation orientation) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_orientation(orientation);
}

void HexMapNodeEditorPlugin::cursor_set_orientation(int p_orientation) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_orientation((HexMapTileOrientation)p_orientation);
}

void HexMapNodeEditorPlugin::cursor_rotate(int steps) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    HexMapTileOrientation orientation = editor_cursor->get_orientation();
    orientation.rotate(steps);
    cursor_set_orientation(orientation);
}

void HexMapNodeEditorPlugin::cursor_flip() {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    HexMapTileOrientation orientation = editor_cursor->get_orientation();
    orientation.flip();
    cursor_set_orientation(orientation);
}

EditorPlugin::AfterGUIInput HexMapNodeEditorPlugin::handle_keypress(
        Ref<InputEventKey> event) {
    if (ED_IS_SHORTCUT("hex_map/edit_y_plane", event)) {
        // set edit plane to y
        edit_plane_set_axis(EditorCursor::AXIS_Y);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_q_plane", event)) {
        edit_plane_set_axis(EditorCursor::AXIS_Q);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_r_plane", event)) {
        edit_plane_set_axis(EditorCursor::AXIS_R);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_s_plane", event)) {
        edit_plane_set_axis(EditorCursor::AXIS_S);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_increment", event)) {
        int depth = (int)edit_axis_depth[editor_cursor->get_axis()] + 1;
        edit_plane_set_depth(depth);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_decrement", event)) {
        int depth = (int)edit_axis_depth[editor_cursor->get_axis()] - 1;
        edit_plane_set_depth(depth);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/cursor_rotate_cw", event)) {
        cursor_rotate(-1);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/cursor_rotate_ccw", event)) {
        cursor_rotate(1);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/cursor_flip", event)) {
        cursor_flip();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/selection_clear", event)) {
        selection_clear();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/selection_fill", event)) {
        selection_fill();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/selection_move", event)) {
        selection_move();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/selection_clone", event)) {
        selection_clone();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }

    return EditorPlugin::AFTER_GUI_INPUT_PASS;
}

void HexMapNodeEditorPlugin::_bind_methods() {
    ADD_SIGNAL(MethodInfo(
            "selection_changed", PropertyInfo(Variant::BOOL, "active")));
    ADD_SIGNAL(MethodInfo("edit_plane_changed",
            PropertyInfo(Variant::INT,
                    "orientation",
                    PROPERTY_HINT_ENUM,
                    "AxisQ,AxisR,AxisS,AxisY"),
            PropertyInfo(Variant::INT, "depth")));
    ADD_SIGNAL(MethodInfo("cursor_restore"));

    ClassDB::bind_method(D_METHOD("deselect_cells"),
            &HexMapNodeEditorPlugin::_deselect_cells);
    ClassDB::bind_method(D_METHOD("select_cell", "cell"),
            &HexMapNodeEditorPlugin::_select_cell);
    ClassDB::bind_method(D_METHOD("set_selection_v", "cells_vector"),
            &HexMapNodeEditorPlugin::_set_selection_v);
    ClassDB::bind_method(D_METHOD("cursor_set_orientation", "orientation"),
            static_cast<void (HexMapNodeEditorPlugin::*)(int)>(
                    &HexMapNodeEditorPlugin::cursor_set_orientation));
    ClassDB::bind_method(D_METHOD("cursor_rotate", "steps"),
            &HexMapNodeEditorPlugin::cursor_rotate);

    BIND_ENUM_CONSTANT(EditorCursor::AXIS_Q);
    BIND_ENUM_CONSTANT(EditorCursor::AXIS_R);
    BIND_ENUM_CONSTANT(EditorCursor::AXIS_S);
    BIND_ENUM_CONSTANT(EditorCursor::AXIS_Y);
}

void HexMapNodeEditorPlugin::add_editor_shortcut(const String &path,
        const String &name,
        Key keycode) {
    Ref<EditorSettings> editor_settings =
            EditorInterface::get_singleton()->get_editor_settings();

    Array events;
    Ref<InputEventKey> input_event_key;
    if (keycode != KEY_NONE) {
        input_event_key.instantiate();
        input_event_key->set_physical_keycode((Key)(keycode & KEY_CODE_MASK));
        if ((keycode & KEY_MASK_SHIFT) != 0) {
            input_event_key->set_shift_pressed(true);
        }
        if ((keycode & KEY_MASK_ALT) != 0) {
            input_event_key->set_alt_pressed(true);
        }
        if ((keycode & KEY_MASK_CMD_OR_CTRL) != 0) {
            input_event_key->set_command_or_control_autoremap(true);
        } else if ((keycode & KEY_MASK_CTRL) != 0) {
            input_event_key->set_ctrl_pressed(true);
        } else if ((keycode & KEY_MASK_META) != 0) {
            input_event_key->set_meta_pressed(true);
        }
        events.push_back(input_event_key);
    }

    Ref<Shortcut> shortcut = editor_settings->get_setting(path);
    if (shortcut.is_valid()) {
        shortcut->set_name(name);
        shortcut->set_meta("original", events.duplicate(true));
    } else {
        shortcut.instantiate();
        shortcut->set_name(name);
        shortcut->set_events(events);
        shortcut->set_meta("original", events.duplicate(true));
        editor_settings->set_setting(path, shortcut);
    }
}

HexMapNodeEditorPlugin::HexMapNodeEditorPlugin() {
    // Define edit plane shortcuts
    add_editor_shortcut("hex_map/edit_y_plane", "Edit Y Plane", Key::KEY_X);
    add_editor_shortcut("hex_map/edit_q_plane", "Edit Q Plane", Key::KEY_NONE);
    add_editor_shortcut("hex_map/edit_r_plane", "Edit R Plane", Key::KEY_NONE);
    add_editor_shortcut("hex_map/edit_s_plane", "Edit S Plane", Key::KEY_NONE);
    add_editor_shortcut("hex_map/edit_plane_rotate_cw",
            "Rotate Edit Plane About Y-Axis Clockwise",
            Key::KEY_C);
    add_editor_shortcut("hex_map/edit_plane_rotate_ccw",
            "Rotate Edit Plane About Y-Axis Counter-Clockwise",
            Key::KEY_Z);
    add_editor_shortcut("hex_map/edit_plane_increment",
            "Increment Edit Plane",
            Key::KEY_E);
    add_editor_shortcut("hex_map/edit_plane_decrement",
            "Decrement Edit Plane",
            Key::KEY_Q);

    // define editor cursor shortcuts
    add_editor_shortcut("hex_map/cursor_rotate_cw",
            "Rotate Editor Cursor Clockwise",
            Key::KEY_D);
    add_editor_shortcut("hex_map/cursor_rotate_ccw",
            "Rotate Editor Cursor Counter Clockwise",
            Key::KEY_A);
    add_editor_shortcut("hex_map/cursor_clear_rotation",
            "Clear Editor Cursor Rotation",
            Key::KEY_S);
    add_editor_shortcut(
            "hex_map/cursor_flip", "Flip Editor Cursor", Key::KEY_W);

    // selection shortcuts
    add_editor_shortcut("hex_map/selection_clear",
            "Clear Selected Cells",
            Key::KEY_BACKSPACE);
    add_editor_shortcut(
            "hex_map/selection_fill", "Fill Selected Cells", Key::KEY_F);
    add_editor_shortcut("hex_map/selection_clone",
            "Clone Selected Cells",
            Key(KEY_MASK_SHIFT + Key::KEY_D));
    add_editor_shortcut(
            "hex_map/selection_move", "Move Selected Cells", Key::KEY_G);
}

HexMapNodeEditorPlugin::~HexMapNodeEditorPlugin() {
    ERR_FAIL_NULL(RenderingServer::get_singleton());

    if (editor_cursor != nullptr) {
        delete editor_cursor;
    }
    if (selection_manager != nullptr) {
        delete selection_manager;
    }
}
#endif
