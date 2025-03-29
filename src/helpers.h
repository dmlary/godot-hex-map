#pragma once
#include "godot_cpp/classes/input_event.hpp"
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/shortcut.hpp>

using namespace godot;

/// check if an input event matches a given shortcut
/// Note: this is done as an inlined function because MSVC does not support
/// compound literals
static inline bool editor_interface_shortcut_matches(const char *name,
        godot::Ref<InputEvent> event) {
    Ref<Shortcut> shortcut = EditorInterface::get_singleton()
                                     ->get_editor_settings()
                                     ->get_setting(name);
    return shortcut.is_valid() && shortcut->matches_event(event);
}

#define ED_IS_SHORTCUT(n, ev) editor_interface_shortcut_matches(n, ev)
