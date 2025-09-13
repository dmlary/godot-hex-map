#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/core/object.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "cell_id.h"
#include "hex_map_node.h"

void HexMapNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_space"), &HexMapNode::_get_space);
    ClassDB::bind_method(D_METHOD("set_space", "space"),
            static_cast<void (HexMapNode::*)(
                    const Ref<hex_bind::HexMapSpace> &)>(
                    &HexMapNode::set_space));

    ClassDB::bind_method(D_METHOD("set_cell_height", "height"),
            &HexMapNode::set_cell_height);
    ClassDB::bind_method(
            D_METHOD("get_cell_height"), &HexMapNode::get_cell_height);
    ClassDB::bind_method(D_METHOD("set_cell_radius", "radius"),
            &HexMapNode::set_cell_radius);
    ClassDB::bind_method(
            D_METHOD("get_cell_radius"), &HexMapNode::get_cell_radius);

    ClassDB::bind_method(D_METHOD("set_cell", "cell", "value", "orientation"),
            static_cast<void (HexMapNode::*)(
                    const Ref<hex_bind::HexMapCellId>, int, int)>(
                    &HexMapNode::set_cell),
            DEFVAL(0));
    ClassDB::bind_method(D_METHOD("set_cells", "cells"),
            static_cast<void (HexMapNode::*)(const Array)>(
                    &HexMapNode::set_cells));

    ClassDB::bind_method(
            D_METHOD("get_cell", "cell_id"), &HexMapNode::_get_cell);
    ClassDB::bind_method(D_METHOD("get_cells", "cells"),
            static_cast<Array (HexMapNode::*)(const Array)>(
                    &HexMapNode::get_cells));

    ClassDB::bind_method(D_METHOD("has", "cell_id"),
            static_cast<bool (HexMapNode::*)(const Ref<hex_bind::HexMapCellId>)
                            const>(&HexMapNode::has));

    ClassDB::bind_method(D_METHOD("get_cell_vecs"),
            static_cast<Array (HexMapNode::*)() const>(
                    &HexMapNode::get_cell_vecs));
    ClassDB::bind_method(D_METHOD("find_cell_vecs_by_value", "value"),
            static_cast<Array (HexMapNode::*)(int) const>(
                    &HexMapNode::find_cell_vecs_by_value));

    ClassDB::bind_method(D_METHOD("get_cell_center", "cell_id"),
            static_cast<Vector3 (HexMapNode::*)(
                    const Ref<hex_bind::HexMapCellId>) const>(
                    &HexMapNode::get_cell_center));
    ClassDB::bind_method(
            D_METHOD("get_cell_id", "local_pos"), &HexMapNode::_get_cell_id);
    ClassDB::bind_method(D_METHOD("get_cell_ids_in_local_quad",
                                 "a",
                                 "b",
                                 "c",
                                 "d",
                                 "padding"),
            &HexMapNode::get_cell_ids_in_local_quad);

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

    ADD_SIGNAL(MethodInfo("hex_space_changed"));
    ADD_SIGNAL(MethodInfo("mesh_offset_changed"));
    ADD_SIGNAL(MethodInfo(
            "cells_changed", PropertyInfo(Variant::ARRAY, "cells")));

    BIND_CONSTANT(CELL_ARRAY_WIDTH);
    BIND_CONSTANT(CELL_ARRAY_INDEX_VEC);
    BIND_CONSTANT(CELL_ARRAY_INDEX_VALUE);
    BIND_CONSTANT(CELL_ARRAY_INDEX_ORIENTATION);
    BIND_CONSTANT(CELL_VALUE_NONE);
}

void HexMapNode::_notification(int p_what) {
    switch (p_what) {
    case NOTIFICATION_POSTINITIALIZE:
        set_notify_transform(true);
        break;
    case NOTIFICATION_TRANSFORM_CHANGED:
        space.set_transform(get_global_transform());
        on_hex_space_changed();
        break;
    }
}

void HexMapNode::set_space(const HexMapSpace &value) {
    space = value;
    on_hex_space_changed();
}

void HexMapNode::set_space(const Ref<hex_bind::HexMapSpace> &ref) {
    ERR_FAIL_COND_MSG(!ref.is_valid(), "argument was not HexMapSpace");
    space = ref->inner;
    on_hex_space_changed();
}

Ref<hex_bind::HexMapSpace> HexMapNode::_get_space() {
    Ref<hex_bind::HexMapSpace> ref(memnew(hex_bind::HexMapSpace(space)));
    return ref;
}

void HexMapNode::set_cell_height(real_t p_height) {
    space.set_cell_height(p_height);
    on_hex_space_changed();
}

real_t HexMapNode::get_cell_height() const { return space.get_cell_height(); }

void HexMapNode::set_cell_radius(real_t p_radius) {
    space.set_cell_radius(p_radius);
    on_hex_space_changed();
}

real_t HexMapNode::get_cell_radius() const { return space.get_cell_radius(); }

bool HexMapNode::on_hex_space_changed() {
    emit_signal("hex_space_changed");
    return true;
}

Vector3 HexMapNode::get_cell_center(const HexMapCellId &cell_id) const {
    return space.get_cell_center(cell_id);
}

Vector3 HexMapNode::get_cell_center(
        const Ref<hex_bind::HexMapCellId> ref) const {
    ERR_FAIL_COND_V_MSG(
            !ref.is_valid(), Vector3(), "argument was not HexMapSpace");
    return space.get_cell_center(ref->inner);
}

HexMapCellId HexMapNode::get_cell_id(Vector3 pos) const {
    return space.get_cell_id(pos);
}
Ref<hex_bind::HexMapCellId> HexMapNode::_get_cell_id(Vector3 pos) const {
    return space.get_cell_id(pos).to_ref();
}

void HexMapNode::set_cell(const Ref<hex_bind::HexMapCellId> ref,
        int p_item,
        int p_orientation) {
    ERR_FAIL_COND_MSG(!ref.is_valid(), "first argument was not HexMapCellId");
    set_cell(ref->inner, p_item, p_orientation);

    // Emit the signal noting that the cell has changed.
    static_assert(HexMapNode::CELL_ARRAY_INDEX_VEC == 0);
    static_assert(HexMapNode::CELL_ARRAY_INDEX_VALUE == 1);
    static_assert(HexMapNode::CELL_ARRAY_INDEX_ORIENTATION == 2);
    emit_signal("cells_changed",
            Array::make(ref->inner.to_vec(), p_item, p_orientation));
}

void HexMapNode::set_cells(const Array cells) {
    int size = cells.size();
    ERR_FAIL_COND_MSG(size % CELL_ARRAY_WIDTH != 0,
            "set_cells(): Array size must be a multiple of " +
                    itos(CELL_ARRAY_WIDTH));
    for (int i = 0; i < size; i += CELL_ARRAY_WIDTH) {
        HexMapCellId cell_id(cells[i + CELL_ARRAY_INDEX_VEC]);
        int value = cells[i + CELL_ARRAY_INDEX_VALUE];
        HexMapTileOrientation orientation =
                cells[i + CELL_ARRAY_INDEX_ORIENTATION];
        set_cell(cell_id, value, orientation);
    }
    emit_signal("cells_changed", cells);
}

Dictionary HexMapNode::_get_cell(
        const Ref<hex_bind::HexMapCellId> &ref) const {
    Dictionary out;
    ERR_FAIL_COND_V_MSG(!ref.is_valid(), out, "argument was not HexMapCellId");
    auto info = get_cell(ref->inner);
    out["value"] = info.value;
    out["orientation"] = info.orientation;
    return out;
}

bool HexMapNode::has(const Ref<hex_bind::HexMapCellId> ref) const {
    ERR_FAIL_COND_V_MSG(
            !ref.is_valid(), false, "argument was not HexMapCellId");
    return has(ref->inner);
}

Array HexMapNode::get_cells(const Array cells) {
    int size = cells.size();
    Array out;
    out.resize(size * CELL_ARRAY_WIDTH);
    for (int i = 0; i < size; i++) {
        HexMapCellId cell_id(cells[i]);
        auto [value, orientation] = get_cell(cell_id);
        int base = i * CELL_ARRAY_WIDTH;
        out[base + CELL_ARRAY_INDEX_VEC] = (Vector3i)cell_id;
        out[base + CELL_ARRAY_INDEX_VALUE] = value;
        out[base + CELL_ARRAY_INDEX_ORIENTATION] = orientation;
    }
    return out;
}

void HexMapNode::set_cells_visibility(const Array cells) {
    int len = cells.size();
    for (int i = 0; i < len; i += 2) {
        set_cell_visibility((Vector3i)cells[i], cells[i + 1]);
    }
}

Array HexMapNode::get_cell_ids_in_local_quad(Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 d,
        float padding) const {
    Vector<HexMapCellId> cell_ids =
            space.get_cell_ids_in_local_quad(a, b, c, d, padding);
    size_t count = cell_ids.size();
    Array out;
    out.resize(count);
    for (int i = 0; i < count; i++) {
        out[i] = cell_ids[i].to_ref();
    }
    return out;
}
