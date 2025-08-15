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
#include <godot_cpp/classes/input.hpp>
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
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/undo_redo.hpp>
#include <godot_cpp/classes/v_separator.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/dictionary.hpp>
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

void HexMapNodeEditorPlugin::set_buildup_brush(bool toggled) {
    buildup_mode = toggled;
}

void HexMapNodeEditorPlugin::commit_cell_changes(String desc) {
    auto change_count = cells_changed.size();
    Array do_list, undo_list;

    do_list.resize(change_count * HexMapNode::CELL_ARRAY_WIDTH);
    undo_list.resize(change_count * HexMapNode::CELL_ARRAY_WIDTH);
    for (int i = 0; i < change_count; i++) {
        const CellChange &change = cells_changed[i];
        int base = i * HexMapNode::CELL_ARRAY_WIDTH;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VEC] =
                (Vector3i)change.cell_id;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] = change.new_tile;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_ORIENTATION] =
                change.new_orientation;
        undo_list[base + HexMapNode::CELL_ARRAY_INDEX_VEC] =
                (Vector3i)change.cell_id;
        undo_list[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] =
                change.orig_tile;
        undo_list[base + HexMapNode::CELL_ARRAY_INDEX_ORIENTATION] =
                change.orig_orientation;
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
            this, "set_selection", selection_manager->get_cell_vecs());

    undo_redo->commit_action();
}

void HexMapNodeEditorPlugin::clear_current_selection() {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");
    selection_manager->clear();
    emit_signal("selection_active_changed", false);
}

void HexMapNodeEditorPlugin::set_selection_vecs(Array cells) {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");

    selection_manager->set_cells(cells);
    emit_signal("selection_active_changed", !cells.is_empty());
}

void HexMapNodeEditorPlugin::selection_clear() {
    ERR_FAIL_COND_MSG(selection_manager == nullptr,
            "HexMap: SelectionManager not present");

    auto cells = selection_manager->get_cell_vecs();

    // build the set_cells() argument to restore the original cells
    Array undo_list = hex_map->get_cells(cells);

    // build the set_cells() argument to clear the cells
    Array do_list;
    int cell_count = cells.size();
    do_list.resize(cell_count * HexMapNode::CELL_ARRAY_WIDTH);
    for (int i = 0; i < cell_count; i++) {
        int base = i * HexMapNode::CELL_ARRAY_WIDTH;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VEC] = cells[i];
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] =
                HexMapNode::CELL_VALUE_NONE;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_ORIENTATION] = 0;
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

    auto tile = editor_cursor->get_origin_cell_info();
    ERR_FAIL_COND(tile.value == HexMapNode::CELL_VALUE_NONE);
    auto cells = selection_manager->get_cell_vecs();

    // build the set_cells() argument to restore the original cells
    Array undo_list = hex_map->get_cells(cells);

    // build the set_cells() argument to clear the cells
    Array do_list;
    int cell_count = cells.size();
    do_list.resize(cell_count * HexMapNode::CELL_ARRAY_WIDTH);
    for (int i = 0; i < cell_count; i++) {
        int base = i * HexMapNode::CELL_ARRAY_WIDTH;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VEC] = cells[i];
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] = tile.value;
        do_list[base + HexMapNode::CELL_ARRAY_INDEX_ORIENTATION] =
                tile.orientation;
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

    editor_cursor->clear_tiles();
    cursor_set_orientation(HexMapTileOrientation());

    Vector<HexMapCellId> cells = selection_manager->get_cell_ids();
    if (cells.is_empty()) {
        return;
    }
    // get the center cell id for the selection
    HexMapCellId center = selection_manager->get_center();

    // copy all the cells into the editor cursor, adjusting the selection
    // manager's cell id to be centered on the editor cursor
    for (const HexMapCellId &cell_id : cells) {
        auto [value, orientation] = hex_map->get_cell(cell_id);
        if (value == HexMapNode::CELL_VALUE_NONE) {
            continue;
        }
        editor_cursor->set_tile(cell_id - center, value, orientation);
    }
    editor_cursor->update(true);
    emit_signal("cursor_active_changed", true);
    input_state = INPUT_STATE_MOVING;
}

void HexMapNodeEditorPlugin::selection_clone() {
    ERR_FAIL_COND_MSG(input_state != INPUT_STATE_DEFAULT,
            "HexMap: cannot clone cells during another process");
    ERR_FAIL_COND_MSG(selection_manager->is_empty(),
            "HexMap: no cells selected; cannot clone");

    copy_selection_to_cursor();

    // save off selection for cancel, then clear it
    last_selection = selection_manager->get_cell_vecs();
    clear_current_selection();

    input_state = INPUT_STATE_CLONING;
}

void HexMapNodeEditorPlugin::selection_clone_cancel() {
    set_selection_vecs(last_selection);
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
    undo_redo->add_do_method(this, "set_selection", do_select);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_list);
    undo_redo->add_undo_method(this, "set_selection", undo_select);
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

    // save the original cell contents
    last_selection = selection_manager->get_cell_vecs();
    move_source_cells = hex_map->get_cells(last_selection);

    // clear the cells; we use set_cells() to ensure the "cells_changed" signal
    // is emitted
    Array cells;
    size_t count = selection_manager->size();
    cells.resize(count * HexMapNode::CELL_ARRAY_WIDTH);
    size_t index = 0;
    for (const HexMapCellId &cell_id : selection_manager->get_cell_ids()) {
        size_t base = index * HexMapNode::CELL_ARRAY_WIDTH;
        cells[base] = cell_id;
        cells[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] =
                HexMapNode::CELL_VALUE_NONE;
        index++;
    }
    hex_map->set_cells(cells);

    // clear selection
    clear_current_selection();

    input_state = INPUT_STATE_MOVING;
}

void HexMapNodeEditorPlugin::selection_move_cancel() {
    hex_map->set_cells(move_source_cells);
    move_source_cells.clear();

    set_selection_vecs(last_selection);
    last_selection.clear();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::selection_move_apply() {
    // first phase of redo is to clear the source cells
    int count = move_source_cells.size();
    Array do_set_cells;
    do_set_cells.resize(count * HexMapNode::CELL_ARRAY_WIDTH);
    for (int i = 0; i < count; i++) {
        int base = i * HexMapNode::CELL_ARRAY_WIDTH;
        do_set_cells[base] = move_source_cells[i];
        do_set_cells[base + HexMapNode::CELL_ARRAY_INDEX_VALUE] =
                HexMapNode::CELL_VALUE_NONE;
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
    undo_redo->add_do_method(this, "set_selection", do_select);
    undo_redo->add_undo_method(hex_map, "set_cells", undo_set_cells);
    undo_redo->add_undo_method(this, "set_selection", undo_select);
    auto prof = profiling_begin("move: commit action");
    undo_redo->commit_action();

    rebuild_cursor();
}

void HexMapNodeEditorPlugin::rebuild_cursor() {
    if (editor_cursor == nullptr) {
        return;
    }

    editor_cursor->clear_tiles();
    if (paint_value != HexMapNode::CELL_VALUE_NONE) {
        editor_cursor->set_tile(HexMapCellId(), paint_value);
    }

    cursor_set_orientation(HexMapTileOrientation());
    editor_cursor->update(true);
    emit_signal("cursor_active_changed", editor_cursor->size() > 0);
}

int32_t HexMapNodeEditorPlugin::_forward_3d_gui_input(Camera3D *p_camera,
        const Ref<InputEvent> &p_event) {
    if (!hex_map) {
        return EditorPlugin::AFTER_GUI_INPUT_PASS;
    }

    Ref<InputEventKey> key_event = p_event;

    // The escape key requires special handling later on; just grab the state
    // for it now.
    bool escape_pressed = key_event.is_valid() &&
            key_event->get_keycode() == Key::KEY_ESCAPE &&
            key_event->is_pressed();

    // Pass keypress events to handle_keypress() to override editor default
    // keybindings.  The one exception is the escape key; we'll consume all
    // escape keypresses, and never pass them to handle_keypress().
    if (key_event.is_valid() && key_event->is_pressed() && !escape_pressed &&
            handle_keypress(key_event) == EditorPlugin::AFTER_GUI_INPUT_STOP) {
        assert(bottom_panel != nullptr &&
                "EditorPlugin::bottom_panel not set");
        // must accept the event here, otherwise another control in the
        // viewport tree may attempt to act on it
        bottom_panel->accept_event();
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }

    // if it is any type of mouse event, update the editor cursor
    // Note: the check for a null camera is necessary because of the event we
    // generate during NOTIFICATION_APPLICATION_FOCUS_OUT
    Ref<InputEventMouse> mouse_event = p_event;
    if (mouse_event.is_valid() && p_camera != nullptr) {
        if (buildup_mode) {
            editor_cursor->update(p_camera, mouse_event->get_position(), hex_map, nullptr);
        } else {
            editor_cursor->update(p_camera, mouse_event->get_position(), nullptr);
        }
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
            editor_cursor->set_visible(false);
            emit_signal("cursor_active_changed", false);
            input_state = INPUT_STATE_SELECTING;
            last_selection = selection_manager->get_cell_vecs();
            selection_anchor = editor_cursor->get_pos();
            selection_manager->clear();
            selection_manager->add_cell(editor_cursor->get_cell());
            return AFTER_GUI_INPUT_STOP;
        } else if (mouse_left_pressed) {
            // always consume the left click, otherwise we fall out of the
            // editor
            if (!editor_cursor->is_empty()) {
                input_state = INPUT_STATE_PAINTING;
            }
            return AFTER_GUI_INPUT_STOP;
        } else if (mouse_right_pressed) {
            editor_cursor->set_visible(false);
            emit_signal("cursor_active_changed", false);
            input_state = INPUT_STATE_ERASING;
            return AFTER_GUI_INPUT_STOP;
        } else if (escape_pressed) {
            if (!selection_manager->is_empty()) {
                deselect_cells();
            } else {
                set_paint_value(HexMapNode::CELL_VALUE_NONE);
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
        auto [tile, orientation] = hex_map->get_cell(cell_id);

        auto cell = editor_cursor->get_origin_cell_info();
        ERR_FAIL_COND_V(cell.value == HexMapNode::CELL_VALUE_NONE,
                AFTER_GUI_INPUT_STOP);
        if (cell.value != tile || cell.orientation != orientation) {
            cells_changed.push_back(CellChange{
                    .cell_id = cell_id,
                    .orig_tile = tile,
                    .orig_orientation = orientation,
                    .new_tile = cell.value,
                    .new_orientation = cell.orientation,
            });
            static_assert(HexMapNode::CELL_ARRAY_INDEX_VEC == 0);
            static_assert(HexMapNode::CELL_ARRAY_INDEX_VALUE == 1);
            static_assert(HexMapNode::CELL_ARRAY_INDEX_ORIENTATION == 2);
            hex_map->set_cells(Array::make(
                    (Vector3i)cell_id, cell.value, cell.orientation));
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
            static_assert(HexMapNode::CELL_ARRAY_INDEX_VEC == 0);
            static_assert(HexMapNode::CELL_ARRAY_INDEX_VALUE == 1);
            static_assert(HexMapNode::CELL_ARRAY_INDEX_ORIENTATION == 2);
            hex_map->set_cells(Array::make(cell_id, -1, 0));
        }
        if (mouse_right_released) {
            commit_cell_changes("HexMap: erase cells");
            input_state = INPUT_STATE_DEFAULT;
            editor_cursor->set_visible(true);
            emit_signal("cursor_active_changed", true);
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
            // p1 .. p4 are all points that fall on the edit plane that
            // make up the quad created by dragging the mouse.
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
            emit_signal("selection_active_changed", true);
        }
        if (mouse_left_released) {
            auto profile = profiling_begin("complete selection");
            EditorUndoRedoManager *undo_redo = get_undo_redo();
            undo_redo->create_action("HexMap: select cells");
            // do method will apply the existing selection
            undo_redo->add_do_method(
                    this, "set_selection", selection_manager->get_cell_vecs());

            undo_redo->add_undo_method(this, "set_selection", last_selection);

            auto prof_commit = profiling_begin("commit selection undo_redo");
            undo_redo->commit_action();

            editor_cursor->set_visible(true);
            emit_signal("cursor_active_changed", true);
            input_state = INPUT_STATE_DEFAULT;
            return AFTER_GUI_INPUT_STOP;
        } else if (escape_pressed) {
            selection_manager->clear();
            selection_manager->set_cells(last_selection);
            editor_cursor->set_visible(true);
            emit_signal("cursor_active_changed", true);
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

void HexMapNodeEditorPlugin::_edit(Object *p_object) {
    if (hex_map) {
        // save off the editor state to the current node
        write_editor_state(hex_map);

        // disconnect signals
        hex_map->disconnect("hex_space_changed",
                callable_mp(this,
                        &HexMapNodeEditorPlugin::on_node_hex_space_changed));
        hex_map->disconnect("mesh_offset_changed",
                callable_mp(this,
                        &HexMapNodeEditorPlugin::on_node_hex_space_changed));

        // clear a couple of other state tracking variables
        last_selection.clear();
        cells_changed.clear();

        // we use delete here because EditorCursor is not a Godot Object
        // subclass, so its destructor is not called with memfree().
        delete editor_cursor;
        editor_cursor = nullptr;

        delete selection_manager;
        selection_manager = nullptr;

        if (bottom_panel != nullptr) {
            remove_control_from_bottom_panel(bottom_panel);
            memfree(bottom_panel);
            bottom_panel = nullptr;
        }
    }

    hex_map = Object::cast_to<HexMapNode>(p_object);

    if (!hex_map) {
        set_process(false);
        return;
    }

    RID scenario = hex_map->get_world_3d()->get_scenario();
    assert(scenario.is_valid());

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

    hex_map->connect("hex_space_changed",
            callable_mp(
                    this, &HexMapNodeEditorPlugin::on_node_hex_space_changed));
    hex_map->connect("mesh_offset_changed",
            callable_mp(
                    this, &HexMapNodeEditorPlugin::on_node_hex_space_changed));

    // add & show the panel
    if (bottom_panel_scene.is_valid()) {
        bottom_panel = (Control *)bottom_panel_scene->instantiate();
        add_control_to_bottom_panel(bottom_panel, hex_map->get_class());
        make_bottom_panel_item_visible(bottom_panel);
    }
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

void HexMapNodeEditorPlugin::set_edit_plane_axis(
        EditorCursor::EditAxis value) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    ERR_FAIL_COND(bottom_panel == nullptr);

    if (value == edit_axis) {
        return;
    }
    edit_axis = value;
    editor_cursor->set_axis(value);
    emit_signal("edit_plane_axis_changed", value);
    set_edit_plane_depth(edit_axis_depth[value]);
}

EditorCursor::EditAxis HexMapNodeEditorPlugin::get_edit_plane_axis() const {
    return edit_axis;
}

void HexMapNodeEditorPlugin::set_edit_plane_depth(int value) {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    ERR_FAIL_COND(bottom_panel == nullptr);

    if (value == (int)edit_axis_depth[edit_axis]) {
        return;
    }
    edit_axis_depth[edit_axis] = value;
    editor_cursor->set_depth(value);
    emit_signal("edit_plane_depth_changed", value);
}

int HexMapNodeEditorPlugin::get_edit_plane_depth() const {
    return edit_axis_depth[edit_axis];
}

void HexMapNodeEditorPlugin::set_paint_value(int value) {
    if (value == paint_value) {
        return;
    }
    paint_value = value;

    // rebuild the cursor if it's not populated with move/cloned cells, or
    // actively selecting cells.
    if (input_state != INPUT_STATE_CLONING &&
            input_state != INPUT_STATE_MOVING &&
            input_state != INPUT_STATE_SELECTING) {
        rebuild_cursor();
    }

    emit_signal("paint_value_changed", value);
}

int HexMapNodeEditorPlugin::get_paint_value() const { return paint_value; }

bool HexMapNodeEditorPlugin::is_cursor_active() const {
    return editor_cursor->get_visible() && !editor_cursor->is_empty();
}

bool HexMapNodeEditorPlugin::is_selection_active() const {
    return !selection_manager->is_empty();
}

void HexMapNodeEditorPlugin::on_node_hex_space_changed() {
    ERR_FAIL_COND_MSG(editor_cursor == nullptr, "editor_cursor not present");
    editor_cursor->set_space(hex_map->get_space());
    selection_manager->set_space(hex_map->get_space());
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
        set_edit_plane_axis(EditorCursor::AXIS_Y);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_q_plane", event)) {
        set_edit_plane_axis(EditorCursor::AXIS_Q);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_r_plane", event)) {
        set_edit_plane_axis(EditorCursor::AXIS_R);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_s_plane", event)) {
        set_edit_plane_axis(EditorCursor::AXIS_S);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_rotate_cw", event)) {
        switch (edit_axis) {
        case EditAxis::AXIS_Y:
            set_edit_plane_axis(EditAxis::AXIS_R);
            break;
        case EditAxis::AXIS_Q:
            set_edit_plane_axis(EditAxis::AXIS_S);
            break;
        case EditAxis::AXIS_R:
            set_edit_plane_axis(EditAxis::AXIS_Q);
            break;
        case EditAxis::AXIS_S:
            set_edit_plane_axis(EditAxis::AXIS_R);
            break;
        }
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_rotate_ccw", event)) {
        switch (edit_axis) {
        case EditAxis::AXIS_Y:
            set_edit_plane_axis(EditAxis::AXIS_S);
            break;
        case EditAxis::AXIS_Q:
            set_edit_plane_axis(EditAxis::AXIS_R);
            break;
        case EditAxis::AXIS_R:
            set_edit_plane_axis(EditAxis::AXIS_S);
            break;
        case EditAxis::AXIS_S:
            set_edit_plane_axis(EditAxis::AXIS_Q);
            break;
        }
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_increment", event)) {
        int depth = (int)edit_axis_depth[editor_cursor->get_axis()] + 1;
        set_edit_plane_depth(depth);
        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }
    if (ED_IS_SHORTCUT("hex_map/edit_plane_decrement", event)) {
        int depth = (int)edit_axis_depth[editor_cursor->get_axis()] - 1;
        set_edit_plane_depth(depth);
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
    if (ED_IS_SHORTCUT("hex_map/focus", event)) {
        // Awful hack to recenter the map at the cursor because we cannot move
        // the viewport camera without it jumping back due to LERP[^1].
        // [^1]: https://github.com/godotengine/godot-proposals/issues/12112

        // calculate what cell we want to center, and grab the location of the
        // center of that cell.
        HexMapCellId cell_id = editor_cursor->get_cell();
        Vector3 center = hex_map->get_space().get_cell_center(cell_id);

        // create a transform to map that center point to the origin.  If this
        // is not the current transform of the HexMap, set it as the new
        // transform.  If this is already the transform, clear the transform.
        // This allows a user to hit the shortcut key twice to clear any
        // custom transform from this shortcut.
        Transform3D transform(Basis(), -center);
        if (hex_map->get_transform() != transform) {
            hex_map->set_transform(transform);
        } else {
            hex_map->set_transform(Transform3D());
        }

        // sent the keypress event to center origin
        Ref<InputEventKey> ev;
        ev.instantiate();
        ev->set_keycode(KEY_O);
        ev->set_pressed(true);
        Input::get_singleton()->parse_input_event(ev);

        // move the cursor to the center of the viewport
        // XXX this does not work right when the user is working in another
        // viewport besides index 0.
        auto viewport =
                EditorInterface::get_singleton()->get_editor_viewport_3d();
        Size2 size = viewport->get_visible_rect().size;
        viewport->warp_mouse(Vector2(size.x / 2, size.y / 2));

        return EditorPlugin::AFTER_GUI_INPUT_STOP;
    }

    return EditorPlugin::AFTER_GUI_INPUT_PASS;
}

void HexMapNodeEditorPlugin::set_bottom_panel_scene(Ref<PackedScene> value) {
    ERR_FAIL_COND_MSG(hex_map != nullptr,
            "HexMapNodeEditorPlugin: cannot change bottom panel while "
            "editing");
    bottom_panel_scene = value;
}

void HexMapNodeEditorPlugin::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_edit_plane_axis", "axis"),
            &HexMapNodeEditorPlugin::set_edit_plane_axis);
    ClassDB::bind_method(D_METHOD("get_edit_plane_axis"),
            &HexMapNodeEditorPlugin::get_edit_plane_axis);
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "edit_plane_axis",
                         godot::PROPERTY_HINT_NONE,
                         ""),
            "set_edit_plane_axis",
            "get_edit_plane_axis");
    ADD_SIGNAL(MethodInfo(
            "edit_plane_axis_changed", PropertyInfo(Variant::INT, "axis")));

    ClassDB::bind_method(D_METHOD("set_edit_plane_depth", "depth"),
            &HexMapNodeEditorPlugin::set_edit_plane_depth);
    ClassDB::bind_method(D_METHOD("get_edit_plane_depth"),
            &HexMapNodeEditorPlugin::get_edit_plane_depth);
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "edit_plane_depth",
                         godot::PROPERTY_HINT_NONE,
                         ""),
            "set_edit_plane_depth",
            "get_edit_plane_depth");
    ADD_SIGNAL(MethodInfo(
            "edit_plane_depth_changed", PropertyInfo(Variant::INT, "depth")));

    ClassDB::bind_method(D_METHOD("set_paint_value", "depth"),
            &HexMapNodeEditorPlugin::set_paint_value);
    ClassDB::bind_method(D_METHOD("get_paint_value"),
            &HexMapNodeEditorPlugin::get_paint_value);
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "paint_value",
                         godot::PROPERTY_HINT_NONE,
                         ""),
            "set_paint_value",
            "get_paint_value");
    ADD_SIGNAL(MethodInfo(
            "paint_value_changed", PropertyInfo(Variant::INT, "value")));

    ClassDB::bind_method(D_METHOD("is_cursor_active"),
            &HexMapNodeEditorPlugin::is_cursor_active);
    ADD_SIGNAL(MethodInfo(
            "cursor_active_changed", PropertyInfo(Variant::INT, "active")));

    ClassDB::bind_method(D_METHOD("is_selection_active"),
            &HexMapNodeEditorPlugin::is_selection_active);
    ADD_SIGNAL(MethodInfo(
            "selection_active_changed", PropertyInfo(Variant::INT, "active")));

    ClassDB::bind_method(D_METHOD("cursor_rotate", "steps"),
            &HexMapNodeEditorPlugin::cursor_rotate);
    ClassDB::bind_method(D_METHOD("selection_move"),
            &HexMapNodeEditorPlugin::selection_move);
    ClassDB::bind_method(D_METHOD("selection_clone"),
            &HexMapNodeEditorPlugin::selection_clone);
    ClassDB::bind_method(D_METHOD("selection_fill"),
            &HexMapNodeEditorPlugin::selection_fill);
    ClassDB::bind_method(D_METHOD("selection_clear"),
            &HexMapNodeEditorPlugin::selection_clear);

    ClassDB::bind_method(D_METHOD("deselect_cells"),
            &HexMapNodeEditorPlugin::clear_current_selection);
    ClassDB::bind_method(D_METHOD("set_selection", "cells_vector"),
            &HexMapNodeEditorPlugin::set_selection_vecs);
    ClassDB::bind_method(D_METHOD("cursor_set_orientation", "orientation"),
            static_cast<void (HexMapNodeEditorPlugin::*)(int)>(
                    &HexMapNodeEditorPlugin::cursor_set_orientation));

    ClassDB::bind_method(D_METHOD("set_buildup_brush", "toggled"), 
            &HexMapNodeEditorPlugin::set_buildup_brush);
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
    buildup_mode = false;

    // initialize the depth array
    edit_axis_depth.resize(4);
    edit_axis_depth.fill(0);

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

    add_editor_shortcut("hex_map/focus", "Focus Cursor Cells", Key::KEY_V);
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
