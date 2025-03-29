#include "int_node/int_node.h"
#include "core/cell_id.h"
#include "core/tile_orientation.h"
#include "godot_cpp/core/error_macros.hpp"
#include "godot_cpp/variant/array.hpp"
#include "godot_cpp/variant/dictionary.hpp"

unsigned HexMapIntNode::_add_cell_type(const String name, const Color color) {
    unsigned id = max_type++;
    cell_types.insert(id, CellType{ .name = name, .color = color });
    return id;
}

void HexMapIntNode::remove_cell_type(unsigned id) { cell_types.erase(id); }

bool HexMapIntNode::update_cell_type(unsigned id,
        const String name,
        const Color color) {
    struct CellType *entry = cell_types.getptr(id);
    ERR_FAIL_NULL_V_MSG(
            entry, false, "HexMapIntNode: unknown cell type " + itos(id));

    entry->name = name;
    entry->color = color;
    return true;
}

Array HexMapIntNode::_get_cell_types() const {
    Array out;
    for (const auto &it : cell_types) {
        Dictionary type;
        type["value"] = it.key;
        type["name"] = it.value.name;
        type["color"] = it.value.color;
        out.push_back(type);
    }
    return out;
}

void HexMapIntNode::_bind_methods() {
    ClassDB::bind_method(
            D_METHOD("get_cell_types"), &HexMapIntNode::_get_cell_types);
    ClassDB::bind_method(D_METHOD("add_cell_type", "name", "color"),
            &HexMapIntNode::_add_cell_type);
    ClassDB::bind_method(D_METHOD("remove_cell_type", "id"),
            &HexMapIntNode::remove_cell_type);
    ClassDB::bind_method(
            D_METHOD("update_cell_type", "value", "name", "color"),
            &HexMapIntNode::update_cell_type);
#ifdef TOOLS_ENABLED
    ADD_SIGNAL(MethodInfo("editor_plugin_cell_changed",
            PropertyInfo(Variant::VECTOR3I, "cell"),
            PropertyInfo(Variant::INT, "type")));
#endif // TOOLS_ENABLED
}

void HexMapIntNode::set_cell(const HexMapCellId &cell_id,
        int value,
        HexMapTileOrientation _) {
    cell_map.insert(cell_id, value);
#ifdef TOOLS_ENABLED
    emit_signal("editor_plugin_cell_changed", (Vector3i)cell_id, value);
#endif // TOOLS_ENABLED
}

HexMapNode::CellInfo HexMapIntNode::get_cell(
        const HexMapCellId &cell_id) const {
    const int *current_cell = cell_map.getptr(cell_id);
    if (current_cell == nullptr) {
        return CellInfo{ .value = INVALID_CELL_ITEM };
    }
    return CellInfo{ .value = *current_cell };
}

Array HexMapIntNode::get_cell_ids_v() const {
    Array out;
    for (const auto &iter : cell_map) {
        out.push_back((Vector3i)iter.key);
    }
    return out;
}
