#ifdef TOOLS_ENABLED

#include "editor_control.h"

#include <godot_cpp/classes/box_container.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/classes/line_edit.hpp>
#include <godot_cpp/classes/popup_menu.hpp>
#include <godot_cpp/classes/shortcut.hpp>
#include <godot_cpp/classes/theme.hpp>
#include <godot_cpp/classes/v_separator.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

Ref<Shortcut> EditorControl::editor_shortcut(const String &p_path,
        const String &p_name,
        Key p_keycode,
        bool p_physical) {
    Ref<EditorSettings> editor_settings =
            EditorInterface::get_singleton()->get_editor_settings();

    Array events;
    Ref<InputEventKey> input_event_key;
    if (p_keycode != KEY_NONE) {
        input_event_key.instantiate();
        if (p_physical) {
            input_event_key->set_physical_keycode(
                    (Key)(p_keycode & KEY_CODE_MASK));
        } else {
            input_event_key->set_keycode((Key)(p_keycode & KEY_CODE_MASK));
        }
        if ((p_keycode & KEY_MASK_SHIFT) != 0) {
            input_event_key->set_shift_pressed(true);
        }
        if ((p_keycode & KEY_MASK_ALT) != 0) {
            input_event_key->set_alt_pressed(true);
        }
        if ((p_keycode & KEY_MASK_CMD_OR_CTRL) != 0) {
            input_event_key->set_command_or_control_autoremap(true);
        } else if ((p_keycode & KEY_MASK_CTRL) != 0) {
            input_event_key->set_ctrl_pressed(true);
        } else if ((p_keycode & KEY_MASK_META) != 0) {
            input_event_key->set_meta_pressed(true);
        }
        events.push_back(input_event_key);
    }

    Ref<Shortcut> shortcut = editor_settings->get_setting(p_path);
    if (shortcut.is_valid()) {
        shortcut->set_name(p_name);
        shortcut->set_meta("original", events.duplicate(true));
    } else {
        shortcut.instantiate();
        shortcut->set_name(p_name);
        shortcut->set_events(events);
        shortcut->set_meta("original", events.duplicate(true));
        editor_settings->set_setting(p_path, shortcut);
    }

    return shortcut;
}

void EditorControl::handle_action(int p_action) {
    switch ((Action)p_action) {
        case ACTION_PLANE_DOWN:
            plane_spin_box->set_value(plane_spin_box->get_value() - 1);
            break;
        case ACTION_PLANE_UP:
            plane_spin_box->set_value(plane_spin_box->get_value() + 1);
            break;
        case ACTION_AXIS_Y:
            active_axis = EditAxis::AXIS_Y;
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_AXIS_Q:
            active_axis = EditAxis::AXIS_Q;
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_AXIS_R:
            active_axis = EditAxis::AXIS_R;
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_AXIS_S:
            active_axis = EditAxis::AXIS_S;
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_AXIS_ROTATE_CW:
            switch (active_axis) {
                case EditAxis::AXIS_Y:
                    active_axis = EditAxis::AXIS_R;
                    break;
                case EditAxis::AXIS_Q:
                    active_axis = EditAxis::AXIS_S;
                    break;
                case EditAxis::AXIS_R:
                    active_axis = EditAxis::AXIS_Q;
                    break;
                case EditAxis::AXIS_S:
                    active_axis = EditAxis::AXIS_R;
                    break;
            }
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_AXIS_ROTATE_CCW:
            switch (active_axis) {
                case EditAxis::AXIS_Y:
                    active_axis = EditAxis::AXIS_S;
                    break;
                case EditAxis::AXIS_Q:
                    active_axis = EditAxis::AXIS_R;
                    break;
                case EditAxis::AXIS_R:
                    active_axis = EditAxis::AXIS_S;
                    break;
                case EditAxis::AXIS_S:
                    active_axis = EditAxis::AXIS_Q;
                    break;
            }
            emit_signal("axis_changed", active_axis);
            plane_spin_box->set_value(plane[active_axis]);
            break;
        case ACTION_TILE_RESET:
            cursor_orientation = HexMapTileOrientation::Upright0;
            emit_signal("cursor_orientation_changed", cursor_orientation);
            break;
        case ACTION_TILE_FLIP:
            cursor_orientation.flip();
            emit_signal("cursor_orientation_changed", cursor_orientation);
            break;
        case ACTION_TILE_ROTATE_CW:
            cursor_orientation.rotate(-1);
            emit_signal("cursor_orientation_changed", cursor_orientation);
            break;
        case ACTION_TILE_ROTATE_CCW:
            cursor_orientation.rotate(1);
            emit_signal("cursor_orientation_changed", cursor_orientation);
            break;
        case ACTION_SELECTION_CLEAR:
            emit_signal("selection_clear");
            break;
        case ACTION_SELECTION_FILL:
            emit_signal("selection_fill");
            break;
        case ACTION_SELECTION_MOVE:
            emit_signal("selection_move");
            break;
        case ACTION_SELECTION_CLONE:
            emit_signal("selection_clone");
            break;
        case ACTION_DESELECT:
            emit_signal("deselect");
            break;
    }

    // ensure the axis selection ratio buttons are marked correctly
    PopupMenu *popup = menu_button->get_popup();
    Action selected_axis = (Action)(active_axis + ACTION_AXIS_Y);
    for (int i = ACTION_AXIS_Y; i <= ACTION_AXIS_S; i++) {
        int index = popup->get_item_index(i);
        if (index != 1) {
            popup->set_item_checked(index, i == selected_axis);
        }
    }
}

void EditorControl::set_plane(int p_plane) {
    plane[active_axis] = p_plane;
    emit_signal("plane_changed", p_plane);
}

void EditorControl::_bind_methods() {
    ADD_SIGNAL(
            MethodInfo("plane_changed", PropertyInfo(Variant::INT, "plane")));
    ADD_SIGNAL(MethodInfo("axis_changed", PropertyInfo(Variant::INT, "axis")));
    ADD_SIGNAL(MethodInfo("cursor_orientation_changed",
            PropertyInfo(Variant::INT, "orientation")));
    ADD_SIGNAL(MethodInfo("selection_clear"));
    ADD_SIGNAL(MethodInfo("selection_fill"));
    ADD_SIGNAL(MethodInfo("selection_move"));
    ADD_SIGNAL(MethodInfo("selection_clone"));
    ADD_SIGNAL(MethodInfo("deselect"));
}

bool EditorControl::handle_keypress(Ref<InputEventKey> p_event) {
    ERR_FAIL_COND_V(!p_event.is_valid(), false);
    if (!p_event->is_pressed()) {
        return false;
    }

    PopupMenu *popup = menu_button->get_popup();
    int count = popup->get_item_count();

    for (int i = 0; i < count; i++) {
        // skip disabled items
        if (popup->is_item_disabled(i)) {
            continue;
        }
        const Ref<Shortcut> shortcut = popup->get_item_shortcut(i);
        if (shortcut.is_valid() && shortcut->matches_event(p_event)) {
            accept_event();
            handle_action(popup->get_item_id(i));
            return true;
        }
    }
    return false;
}

void EditorControl::update_selection_menu(bool p_active, bool p_duplicate) {
    PopupMenu *popup = menu_button->get_popup();

    for (int i = ACTION_SELECTION_FILL; i <= ACTION_DESELECT; i++) {
        int index = popup->get_item_index(i);
        if (index != -1) {
            popup->set_item_disabled(index, !p_active);
        }
    }
}

EditorControl::EditorControl() {
    EditorInterface *editor_interface = EditorInterface::get_singleton();
    Ref<Theme> theme = editor_interface->get_editor_theme();

    set_h_size_flags(Control::SIZE_EXPAND_FILL);
    set_alignment(BoxContainer::ALIGNMENT_END);

    plane_label = memnew(Label);
    plane_label->set_text("Floor:");
    add_child(plane_label);

    plane_spin_box = memnew(SpinBox);
    plane_spin_box->set_min(SHRT_MIN);
    plane_spin_box->set_max(SHRT_MAX);
    plane_spin_box->set_step(1);
    plane_spin_box->get_line_edit()->add_theme_constant_override(
            "minimum_character_width", 8);
    plane_spin_box->connect(
            "value_changed", callable_mp(this, &EditorControl::set_plane));
    add_child(plane_spin_box);

    add_child(memnew(VSeparator));

    menu_button = memnew(MenuButton);
    menu_button->set_text("Hex Map");
    menu_button->set_button_icon(theme->get_icon("GridMap", "EditorIcons"));
    add_child(menu_button);

    PopupMenu *popup = menu_button->get_popup();
    popup->connect(
            "id_pressed", callable_mp(this, &EditorControl::handle_action));

    popup->add_shortcut(editor_shortcut("hex_map/previous_floor",
                                "Previous Floor",
                                Key::KEY_Q,
                                true),
            Action::ACTION_PLANE_DOWN);
    popup->add_shortcut(
            editor_shortcut(
                    "hex_map/next_floor", "Next Floor", Key::KEY_E, true),
            Action::ACTION_PLANE_UP);

    popup->add_separator();
    popup->add_radio_check_shortcut(
            editor_shortcut(
                    "hex_map/edit_y_axis", "Edit Y Axis", Key::KEY_X, true),
            Action::ACTION_AXIS_Y);
    popup->add_radio_check_item("Edit Q Axis", ACTION_AXIS_Q);
    popup->add_radio_check_item("Edit R Axis", ACTION_AXIS_R);
    popup->add_radio_check_item("Edit S Axis", ACTION_AXIS_S);
    popup->add_shortcut(editor_shortcut("hex_map/edit_plane_rotate_cw",
                                "Rotate Edit Plane Clockwise",
                                Key::KEY_C,
                                true),
            Action::ACTION_AXIS_ROTATE_CW);
    popup->add_shortcut(editor_shortcut("hex_map/edit_plane_rotate_ccw",
                                "Rotate Edit Plane Counter-Clockwise",
                                Key::KEY_Z,
                                true),
            Action::ACTION_AXIS_ROTATE_CCW);

    popup->add_separator();
    popup->add_shortcut(
            editor_shortcut(
                    "hex_map/tile_flip", "Flip Tile", Key::KEY_W, true),
            Action::ACTION_TILE_FLIP);
    popup->add_shortcut(editor_shortcut("hex_map/tile_rotate_cw",
                                "Rotate Tile Clockwise",
                                Key::KEY_D,
                                true),
            Action::ACTION_TILE_ROTATE_CW);
    popup->add_shortcut(editor_shortcut("hex_map/tile_rotate_ccw",
                                "Rotate Tile Counter-Clockwise",
                                Key::KEY_A,
                                true),
            Action::ACTION_TILE_ROTATE_CCW);
    popup->add_shortcut(editor_shortcut("hex_map/tile_clear_rotation",
                                "Clear Tile Rotation",
                                Key::KEY_S,
                                true),
            Action::ACTION_TILE_RESET);

    popup->add_separator();
    popup->add_shortcut(editor_shortcut("hex_map/selection_fill",
                                "Fill Selected Region",
                                Key::KEY_F,
                                true),
            Action::ACTION_SELECTION_FILL);
    popup->add_shortcut(editor_shortcut("hex_map/selection_clear",
                                "Clear Selected Tiles",
                                Key::KEY_BACKSPACE,
                                true),
            Action::ACTION_SELECTION_CLEAR);
    popup->add_shortcut(editor_shortcut("hex_map/selection_clone",
                                "Clone Selected Tiles",
                                Key(KEY_MASK_SHIFT + KEY_D),
                                true),
            Action::ACTION_SELECTION_CLONE);
    popup->add_shortcut(editor_shortcut("hex_map/selection_move",
                                "Move Selected Tiles",
                                Key::KEY_G,
                                true),
            Action::ACTION_SELECTION_MOVE);
    popup->add_item("Deselect Tiles", Action::ACTION_DESELECT);

    // XXX add popup window for setting maximum pick distance
}

EditorControl::~EditorControl() {}

#endif
