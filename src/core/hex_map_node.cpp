#include "hex_map_node.h"
#include "cell_id.h"

void HexMapNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_cell_height", "height"),
            &HexMapNode::set_cell_height);
    ClassDB::bind_method(
            D_METHOD("get_cell_height"), &HexMapNode::get_cell_height);
    ClassDB::bind_method(D_METHOD("set_cell_radius", "radius"),
            &HexMapNode::set_cell_radius);
    ClassDB::bind_method(
            D_METHOD("get_cell_radius"), &HexMapNode::get_cell_radius);

    ClassDB::bind_method(
            D_METHOD("set_center_y", "enable"), &HexMapNode::set_center_y);
    ClassDB::bind_method(D_METHOD("get_center_y"), &HexMapNode::get_center_y);

    ClassDB::bind_method(D_METHOD("set_cell", "cell", "value", "orientation"),
            static_cast<void (HexMapNode::*)(
                    const Ref<hex_bind::HexMapCellId>, int, int)>(
                    &HexMapNode::set_cell));
    ClassDB::bind_method(D_METHOD("set_cells", "cells"),
            static_cast<void (HexMapNode::*)(const Array)>(
                    &HexMapNode::set_cells));
    ClassDB::bind_method(D_METHOD("get_cells", "cells"),
            static_cast<Array (HexMapNode::*)(const Array)>(
                    &HexMapNode::get_cells));

    ADD_GROUP("Cell", "cell_");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,
                         "cell_height",
                         PROPERTY_HINT_NONE,
                         "suffix:m"),
            "set_cell_height",
            "get_cell_height");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT,
                         "cell_radius",
                         PROPERTY_HINT_NONE,
                         "suffix:m"),
            "set_cell_radius",
            "get_cell_radius");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cell_center_y"),
            "set_center_y",
            "get_center_y");

    ADD_SIGNAL(MethodInfo("cell_scale_changed"));
    ADD_SIGNAL(MethodInfo("mesh_offset_changed"));
    ADD_SIGNAL(MethodInfo("mesh_library_changed"));
}

void HexMapNode::set_space(const HexMapSpace &space) {
    this->space = space;
    cell_scale_changed();
}

void HexMapNode::set_cell_height(real_t p_height) {
    space.set_cell_height(p_height);
    cell_scale_changed();
}

real_t HexMapNode::get_cell_height() const { return space.get_cell_height(); }

void HexMapNode::set_cell_radius(real_t p_radius) {
    space.set_cell_radius(p_radius);
    cell_scale_changed();
}

real_t HexMapNode::get_cell_radius() const { return space.get_cell_radius(); }

void HexMapNode::set_center_y(bool p_value) {
    if (p_value) {
        space.set_mesh_offset(Vector3(0, 0, 0));
    } else {
        space.set_mesh_offset(Vector3(0, -0.5, 0));
    }
    cell_scale_changed();
}

bool HexMapNode::get_center_y() const {
    return space.get_mesh_offset().y == 0;
}

bool HexMapNode::cell_scale_changed() {
    emit_signal("cell_scale_changed");
    return true;
}

Vector3 HexMapNode::get_cell_scale() const { return space.get_cell_scale(); }

Vector3 HexMapNode::get_cell_center(const HexMapCellId &cell_id) const {
    return space.get_cell_center(cell_id);
}

HexMapCellId HexMapNode::get_cell_id(Vector3 pos) const {
    return space.get_cell_id(pos);
}

void HexMapNode::set_cell(const Ref<hex_bind::HexMapCellId> cell_id,
        int p_item,
        int p_orientation) {
    set_cell(**cell_id, p_item, p_orientation);
}

void HexMapNode::set_cells(const Array cells) {
    int size = cells.size();
    for (int i = 0; i < size; i += 3) {
        HexMapCellId cell_id(cells[i]);
        int value = cells[i + 1];
        HexMapTileOrientation orientation = cells[i + 2];
        set_cell(cell_id, value, orientation);
    }
}

Array HexMapNode::get_cells(const Array cells) {
    int size = cells.size();
    Array out;
    out.resize(size * 3);
    for (int i = 0; i < size; i++) {
        HexMapCellId cell_id(cells[i]);
        auto [value, orientation] = get_cell(cell_id);
        int base = i * 3;
        out[base] = (Vector3i)cell_id;
        out[base + 1] = value;
        out[base + 2] = orientation;
    }
    return out;
}

void HexMapNode::set_cells_visibility(const Array cells) {
    int len = cells.size();
    for (int i = 0; i < len; i += 2) {
        set_cell_visibility((Vector3i)cells[i], cells[i + 1]);
    }
}
