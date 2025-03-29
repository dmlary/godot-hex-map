#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/geometry2d.hpp>
#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/navigation_mesh.hpp>
#include <godot_cpp/classes/navigation_server3d.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/physics_material.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/aabb.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "core/cell_id.h"
#include "core/hex_map_node.h"
#include "core/iter_cube.h"
#include "core/math.h"
#include "core/tile_orientation.h"
#include "octant.h"
#include "profiling.h"
#include "tiled_node.h"

bool HexMapTiledNode::_set(const StringName &p_name, const Variant &p_value) {
    String name = p_name;

    if (name == "data") {
        Dictionary d = p_value;

        if (d.has("cells")) {
            const PackedByteArray cells = d["cells"];
            ERR_FAIL_COND_V(cells.size() % 10 != 0, false);

            size_t offset = 0;
            while (offset < cells.size()) {
                CellId::Key key;
                key.q = cells.decode_s16(offset);
                offset += 2;
                key.r = cells.decode_s16(offset);
                offset += 2;
                key.y = cells.decode_s16(offset);
                offset += 2;

                Cell cell;
                cell.cell = cells.decode_u32(offset);
                offset += 4;

                cell_map[key] = cell;
            }
        }

        _recreate_octant_data();
    } else if (name == "baked_meshes") {
        clear_baked_meshes();

        Array meshes = p_value;
        for (int i = 0; i < meshes.size(); i += 2) {
            OctantKey key(meshes[i]);
            ERR_CONTINUE_MSG(!octants.has(key),
                    "octant not found for baked mesh index");
            octants.get(key)->set_baked_mesh(meshes[i + 1]);
            baked_mesh_octants.push_back(key);
        }

        update_octant_meshes();

    } else {
        return false;
    }

    return true;
}

bool HexMapTiledNode::_get(const StringName &p_name, Variant &r_ret) const {
    String name = p_name;

    if (name == "data") {
        Dictionary d;

        PackedByteArray cells;
        cells.resize(cell_map.size() * 10);
        size_t offset = 0;
        for (const KeyValue<CellKey, Cell> &E : cell_map) {
            cells.encode_s16(offset, E.key.q);
            offset += 2;
            cells.encode_s16(offset, E.key.r);
            offset += 2;
            cells.encode_s16(offset, E.key.y);
            offset += 2;
            cells.encode_u32(offset, E.value.cell);
            offset += 4;
        }

        d["cells"] = cells;

        r_ret = d;
    } else if (name == "baked_meshes") {
        Array ret;
        ret.resize(baked_mesh_octants.size() * 2);
        for (int i = 0; i < baked_mesh_octants.size(); i++) {
            OctantKey key = baked_mesh_octants[i];
            ERR_CONTINUE_MSG(!octants.has(key),
                    "octant not found for baked mesh index");
            ret[2 * i] = key.key;
            ret[2 * i + 1] = octants[key]->get_baked_mesh();
        }
        r_ret = ret;

    } else {
        return false;
    }

    return true;
}

void HexMapTiledNode::_get_property_list(List<PropertyInfo> *p_list) const {
    p_list->push_back(PropertyInfo(Variant::DICTIONARY,
            "data",
            PROPERTY_HINT_NONE,
            "",
            PROPERTY_USAGE_STORAGE));
    p_list->push_back(PropertyInfo(Variant::ARRAY,
            "baked_meshes",
            PROPERTY_HINT_NONE,
            "",
            PROPERTY_USAGE_STORAGE));
}

void HexMapTiledNode::set_collision_debug(bool value) {
    if (collision_debug == value) {
        return;
    }
    collision_debug = value;

    for (const auto &it : octants) {
        it.value->exit_world();
        it.value->enter_world();
    }
}

bool HexMapTiledNode::get_collision_debug() const { return collision_debug; }

void HexMapTiledNode::set_collision_layer(uint32_t p_layer) {
    collision_layer = p_layer;
    for (const auto &it : octants) {
        it.value->update_collision_properties();
    }
}

uint32_t HexMapTiledNode::get_collision_layer() const {
    return collision_layer;
}

void HexMapTiledNode::set_collision_mask(uint32_t p_mask) {
    collision_mask = p_mask;
    for (const auto &it : octants) {
        it.value->update_collision_properties();
    }
}

uint32_t HexMapTiledNode::get_collision_mask() const { return collision_mask; }

void HexMapTiledNode::set_collision_layer_value(int p_layer_number,
        bool p_value) {
    ERR_FAIL_COND_MSG(p_layer_number < 1,
            "Collision layer number must be between 1 and 32 inclusive.");
    ERR_FAIL_COND_MSG(p_layer_number > 32,
            "Collision layer number must be between 1 and 32 inclusive.");
    uint32_t collision_layer_new = get_collision_layer();
    if (p_value) {
        collision_layer_new |= 1 << (p_layer_number - 1);
    } else {
        collision_layer_new &= ~(1 << (p_layer_number - 1));
    }
    set_collision_layer(collision_layer_new);
}

bool HexMapTiledNode::get_collision_layer_value(int p_layer_number) const {
    ERR_FAIL_COND_V_MSG(p_layer_number < 1,
            false,
            "Collision layer number must be between 1 and 32 inclusive.");
    ERR_FAIL_COND_V_MSG(p_layer_number > 32,
            false,
            "Collision layer number must be between 1 and 32 inclusive.");
    return get_collision_layer() & (1 << (p_layer_number - 1));
}

void HexMapTiledNode::set_collision_mask_value(int p_layer_number,
        bool p_value) {
    ERR_FAIL_COND_MSG(p_layer_number < 1,
            "Collision layer number must be between 1 and 32 inclusive.");
    ERR_FAIL_COND_MSG(p_layer_number > 32,
            "Collision layer number must be between 1 and 32 inclusive.");
    uint32_t mask = get_collision_mask();
    if (p_value) {
        mask |= 1 << (p_layer_number - 1);
    } else {
        mask &= ~(1 << (p_layer_number - 1));
    }
    set_collision_mask(mask);
}

void HexMapTiledNode::set_collision_priority(real_t p_priority) {
    collision_priority = p_priority;
    for (const auto &it : octants) {
        it.value->update_collision_properties();
    }
}

real_t HexMapTiledNode::get_collision_priority() const {
    return collision_priority;
}

// PhysicsMaterial.computed_friction() & computed_bounce() not exposed in
// godot-cpp
#define computed_friction(mat)                                                \
    ((mat)->is_rough() ? -mat->get_friction() : mat->get_friction())
#define computed_bounce(mat)                                                  \
    ((mat)->is_absorbent() ? -mat->get_bounce() : mat->get_bounce())
void HexMapTiledNode::set_physics_material(Ref<PhysicsMaterial> p_material) {
    physics_material = p_material;
    physics_body_friction = 1.0;
    physics_body_bounce = 0.0;

    if (physics_material.is_valid()) {
        physics_body_friction = computed_friction(physics_material);
        physics_body_bounce = computed_bounce(physics_material);
    }

    for (const auto &it : octants) {
        it.value->update_physics_params();
    }
}

Ref<PhysicsMaterial> HexMapTiledNode::get_physics_material() const {
    return physics_material;
}

bool HexMapTiledNode::get_collision_mask_value(int p_layer_number) const {
    ERR_FAIL_COND_V_MSG(p_layer_number < 1,
            false,
            "Collision layer number must be between 1 and 32 inclusive.");
    ERR_FAIL_COND_V_MSG(p_layer_number > 32,
            false,
            "Collision layer number must be between 1 and 32 inclusive.");
    return get_collision_mask() & (1 << (p_layer_number - 1));
}

void HexMapTiledNode::set_mesh_library(
        const Ref<MeshLibrary> &p_mesh_library) {
    if (!mesh_library.is_null()) {
        mesh_library->disconnect("changed",
                callable_mp(this, &HexMapTiledNode::mesh_library_changed));
    }
    mesh_library = p_mesh_library;
    if (!mesh_library.is_null()) {
        mesh_library->connect("changed",
                callable_mp(this, &HexMapTiledNode::mesh_library_changed));
    }
    mesh_library_changed();
}

Ref<MeshLibrary> HexMapTiledNode::get_mesh_library() const {
    return mesh_library;
}

bool HexMapTiledNode::mesh_library_changed() {
    emit_signal("mesh_library_changed");
    _recreate_octant_data();
    return true;
}

bool HexMapTiledNode::cell_scale_changed() {
    HexMapNode::cell_scale_changed();
    clear_baked_meshes();
    _recreate_octant_data();
    return true;
}

void HexMapTiledNode::set_octant_size(int p_size) {
    ERR_FAIL_COND(p_size == 0);
    octant_size = p_size;
    _recreate_octant_data();
}

int HexMapTiledNode::get_octant_size() const { return octant_size; }

void HexMapTiledNode::set_navigation_bake_only_navmesh_tiles(bool value) {
    navigation_bake_only_navmesh_tiles = value;
}

bool HexMapTiledNode::get_navigation_bake_only_navmesh_tiles() const {
    return navigation_bake_only_navmesh_tiles;
}

void HexMapTiledNode::set_cell(const HexMapCellId &cell_id,
        int tile,
        HexMapTileOrientation orientation) {
    set_cell_item(cell_id, tile, static_cast<int>(orientation));
}

void HexMapTiledNode::set_cell_item(const HexMapCellId &cell_id,
        int p_item,
        int p_rot) {
    auto prof = profiling_begin("set cell item");
    ERR_FAIL_COND_MSG(
            !cell_id.in_bounds(), "cell id is not in bounds: " + cell_id);

    // Convert the cell id into a key.  If the cell is already set to the
    // specified values, just return.
    CellKey cell_key(cell_id);
    Cell *current_cell = cell_map.getptr(cell_key);
    if (current_cell != nullptr && p_item == current_cell->item &&
            p_rot == current_cell->rot) {
        return;
    }

    // if the octant meshes have been baked, we need to clear them now.  Even
    // if we only clear the modified octant, the HexMap will look odd with the
    // lightmap only applied to one section.
    if (!baked_mesh_octants.is_empty()) {
        UtilityFunctions::print(
                "HexMap: map modified; clearing baked lighting meshes");
        clear_baked_meshes();
    }

    // look up the cell octant
    OctantKey octant_key(cell_id, octant_size);
    Octant **octant_ptr = octants.getptr(octant_key);
    Octant *octant = octant_ptr ? *octant_ptr : nullptr;

    if (p_item >= 0) {
        // set the cell
        Cell cell = {
            .item = static_cast<unsigned int>(p_item),
            .rot = static_cast<unsigned int>(p_rot),
            .visible = true,
        };
        cell_map.insert(cell_key, cell);
        updated_cells.insert(cell_key);

        // create a new octant if one doesn't already exist for this cell
        if (octant == nullptr) {
            octant = new Octant(*this);
            octants.insert(octant_key, octant);

            if (is_inside_tree()) {
                octant->enter_world();
            }
        }

        // add a cell to the octant, and schedule an update
        octant->set_cell(cell_key, p_item, p_rot);
        update_dirty_octants();

    } else if (current_cell != nullptr) {
        // clear the cell
        cell_map.erase(cell_key);
        updated_cells.insert(cell_key);

        ERR_FAIL_COND_MSG(octant == nullptr, "octant for cell does not exist");
        octant->clear_cell(cell_key);
        update_dirty_octants();
    }

    return;
}

void HexMapTiledNode::_set_cell_item(const Ref<hex_bind::HexMapCellId> cell_id,
        int p_item,
        int p_rot) {
    ERR_FAIL_COND_MSG(!cell_id.is_valid(), "null cell id");
    set_cell_item(**cell_id, p_item, p_rot);
}

void HexMapTiledNode::_set_cell_item_v(const Vector3i &cell_id,
        int p_item,
        int p_rot) {
    set_cell_item(cell_id, p_item, p_rot);
}

Array HexMapTiledNode::get_cell_ids_v() const {
    Array out;
    for (const auto &iter : cell_map) {
        out.push_back((Vector3i)iter.key);
    }
    return out;
}

HexMapNode::CellInfo HexMapTiledNode::get_cell(
        const HexMapCellId &cell_id) const {
    const Cell *current_cell = cell_map.getptr(cell_id);
    if (current_cell == nullptr) {
        return CellInfo{ .value = INVALID_CELL_ITEM };
    }
    return CellInfo{ .value = static_cast<int>(current_cell->item),
        .orientation = HexMapTileOrientation(current_cell->rot) };
}

int HexMapTiledNode::get_cell_item(const HexMapCellId &cell_id) const {
    ERR_FAIL_COND_V_MSG(!cell_id.in_bounds(),
            INVALID_CELL_ITEM,
            "cell id not in bounds: " + cell_id);

    CellKey key(cell_id);

    if (!cell_map.has(key)) {
        return INVALID_CELL_ITEM;
    }
    return cell_map[key].item;
}

int HexMapTiledNode::_get_cell_item(
        const Ref<hex_bind::HexMapCellId> p_cell_id) const {
    ERR_FAIL_COND_V_MSG(!p_cell_id.is_valid(), -1, "null cell id");
    return get_cell_item(**p_cell_id);
}
int HexMapTiledNode::_get_cell_item_v(const Vector3i &p_cell_vector) const {
    return get_cell_item(p_cell_vector);
}

int HexMapTiledNode::get_cell_item_orientation(
        const HexMapCellId &cell_id) const {
    ERR_FAIL_COND_V_MSG(!cell_id.in_bounds(), -1, "CellId out of bounds");

    CellKey key(cell_id);
    if (!cell_map.has(key)) {
        return -1;
    }
    return cell_map[key].rot;
}

int HexMapTiledNode::_get_cell_item_orientation(
        const Ref<hex_bind::HexMapCellId> p_cell_id) const {
    ERR_FAIL_COND_V_MSG(!p_cell_id.is_valid(), 0, "null cell id");
    return get_cell_item_orientation(**p_cell_id);
}

void HexMapTiledNode::set_cell_visibility(const HexMapCellId &cell_id,
        bool visibility) {
    ERR_FAIL_COND_MSG(
            !cell_id.in_bounds(), "cell id is not in bounds:" + cell_id);

    // Convert the cell id into a key.  If the cell is already set to the
    // specified values, just return.
    CellKey cell_key(cell_id);
    Cell *cell = cell_map.getptr(cell_key);
    if (cell == nullptr) {
        return;
    }
    cell->visible = visibility;

    OctantKey octant_key(cell_id, octant_size);
    Octant **octant = octants.getptr(octant_key);
    ERR_FAIL_COND_MSG(
            octant == nullptr, "no octant found for valid cell: " + cell_id);
    (**octant).set_dirty();
    update_dirty_octants();

    // Although we aren't truely modifying the map here, covering the edge case
    // of hide/unhide cells and restoring the baked mesh feels like a lot of
    // work.  For now we're just gonna clear the baked meshes when they start
    // hiding cells.
    if (!baked_mesh_octants.is_empty()) {
        UtilityFunctions::print(
                "HexMap: map modified; clearing baked lighting meshes");
        clear_baked_meshes();
    }

    return;
}

bool HexMapTiledNode::set_cells_visibility_callback(Array cells) {
    if (!baked_mesh_octants.is_empty()) {
        UtilityFunctions::print(
                "HexMap: map modified; clearing baked lighting meshes");
        clear_baked_meshes();
    }

    size_t size = cells.size();
    for (unsigned i = 0; i < size; i += 2) {
        CellId cell_id((Vector3i)cells[i]);
        ERR_CONTINUE_MSG(
                !cell_id.in_bounds(), "cell id is not in bounds: " + cell_id);
        OctantKey octant_key(cell_id, octant_size);
        Octant **octant = octants.getptr(octant_key);
        if (octant == nullptr) {
            continue;
        }
        (*octant)->set_cell_visibility(cell_id, cells[i + 1]);
    }

    update_dirty_octants();
    return true;
}

// based on blog post https://observablehq.com/@jrus/hexround
static inline Vector2i axial_round(real_t q_in, real_t r_in) {
    int q = round(q_in);
    int r = round(r_in);

    real_t q_rem = q_in - q;
    real_t r_rem = r_in - r;

    if (abs(q_rem) >= abs(r_rem)) {
        q += round(0.5 * r_rem + q_rem);
    } else {
        r += round(0.5 * q_rem + r_rem);
    }

    return Vector2i(q, r);
}

// convert axial hex coordinates to offset coordinates
// https://www.redblobgames.com/grids/hexagons/#conversions-offset
static inline Vector3i axial_to_oddr(Vector3i axial) {
    int x = axial.x + (axial.z - (axial.z & 1)) / 2;
    return Vector3i(x, axial.y, axial.z);
}

static inline Vector3i oddr_to_axial(Vector3i oddr) {
    int q = oddr.x - (oddr.z - (oddr.z & 1)) / 2;
    return Vector3i(q, oddr.y, oddr.z);
}

HexMapCellId HexMapTiledNode::local_to_cell_id(
        const Vector3 &local_position) const {
    return space.get_cell_id(local_position);
}

Ref<hex_bind::HexMapCellId> HexMapTiledNode::_local_to_cell_id(
        const Vector3 &p_local_position) const {
    return local_to_cell_id(p_local_position);
}

Vector3 HexMapTiledNode::cell_id_to_local(const HexMapCellId &cell_id) const {
    return space.get_cell_center(cell_id);
}

Vector3 HexMapTiledNode::_cell_id_to_local(
        const Ref<hex_bind::HexMapCellId> p_cell_id) const {
    ERR_FAIL_COND_V_MSG(!p_cell_id.is_valid(), Vector3(), "null cell id");
    return cell_id_to_local(**p_cell_id);
}

Vector<HexMapCellId> HexMapTiledNode::local_quad_to_cell_ids(Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 d) const {
    // UtilityFunctions::print("a ", a, ", b ", b, ", c ", ", d ", d);
    // immediatly switch into unit scale
    a /= get_cell_scale();
    b /= get_cell_scale();
    c /= get_cell_scale();
    d /= get_cell_scale();
    //UtilityFunctions::print("a ", a, ", b ", b, ", c ", ", d ", d);

    Plane plane(a, b, c);
    ERR_FAIL_COND_V_MSG(!plane.has_point(d, 0.0001f),
            Vector<HexMapCellId>(),
            "quad points must all be on the same plane");
    //
    // Vector3 center = (a + b + c + d) / 4;
    // Vector3 cell_center =
    // HexMapCellId::from_unit_point(a).unit_center(); Vector3 direction =
    // a - center; a = cell_center + direction * 0.01;
    //
    // cell_center = HexMapCellId::from_unit_point(c).unit_center();
    // direction = c - center;
    // c = cell_center + direction * 0.01;

    Vector3 top_right = a.max(b).max(c).max(d);
    Vector3 bottom_left = a.min(b).min(c).min(d);
    //UtilityFunctions::print(
    //        "top-right ", top_right, ", bottom-left ", bottom_left);

    HexMapIterCube iter(top_right, bottom_left);

    // we're going to reduce the 3d problem to 2d by using the planes that
    // are most perpendicular to the plane normal.
    int axis[2];
    switch (plane.normal.abs().max_axis_index()) {
        case godot::Vector3::AXIS_X:
            axis[0] = Vector3::AXIS_Y;
            axis[1] = Vector3::AXIS_Z;
            break;
        case godot::Vector3::AXIS_Y:
            axis[0] = Vector3::AXIS_X;
            axis[1] = Vector3::AXIS_Z;
            break;
        case godot::Vector3::AXIS_Z:
            axis[0] = Vector3::AXIS_X;
            axis[1] = Vector3::AXIS_Y;
            break;
    }

    // break the 3D quad down into two 2D triangles
    Vector2 aa(a[axis[0]], a[axis[1]]);
    Vector2 ab(b[axis[0]], b[axis[1]]);
    Vector2 ac(c[axis[0]], c[axis[1]]);

    Vector2 ba(a[axis[0]], a[axis[1]]);
    Vector2 bb(d[axis[0]], d[axis[1]]);
    Vector2 bc(c[axis[0]], c[axis[1]]);

    // pull these points in to make it easier to select SW/SE line
    const PackedVector3Array vertices = PackedVector3Array({
            Vector3(0.0, 0.5, -1.0) * 0.5, // 0
            Vector3(-Math_SQRT3_2, 0.5, -0.5) * 0.5, // 1
            Vector3(-Math_SQRT3_2, 0.5, 0.5) * 0.5, // 2
            Vector3(0.0, 0.5, 1.0) * 0.5, // 3
            Vector3(Math_SQRT3_2, 0.5, 0.5) * 0.5, // 4
            Vector3(Math_SQRT3_2, 0.5, -0.5) * 0.5, // 5
            Vector3(0.0, -0.5, -1.0) * 0.5, // 6
            Vector3(-Math_SQRT3_2, -0.5, -0.5) * 0.5, // 7
            Vector3(-Math_SQRT3_2, -0.5, 0.5) * 0.5, // 8
            Vector3(0.0, -0.5, 1.0) * 0.5, // 9
            Vector3(Math_SQRT3_2, -0.5, 0.5) * 0.5, // 10 (0xa)
            Vector3(Math_SQRT3_2, -0.5, -0.5) * 0.5, // 11 (0xb)
    });

    Geometry2D *geo = Geometry2D::get_singleton();
    Vector<HexMapCellId> out;
    auto prof = profiling_begin("selecting cells");
    for (const CellId &cell_id : iter) {
        Vector3 center = cell_id.unit_center();
        Vector2 point(center[axis[0]], center[axis[1]]);
        bool intercept_quad =
                geo->point_is_inside_triangle(point, aa, ab, ac) ||
                geo->point_is_inside_triangle(point, ba, bb, bc);
        bool intercept_plane = false;
        bool above = plane.is_point_over(center);

        for (Vector3 vert : vertices) {
            vert += center;

            if (!intercept_plane && plane.is_point_over(vert) != above) {
                intercept_plane = true;
            }

            Vector2 point(vert[axis[0]], vert[axis[1]]);
            if (!intercept_quad &&
                    (geo->point_is_inside_triangle(point, aa, ab, ac) ||
                            geo->point_is_inside_triangle(
                                    point, ba, bb, bc))) {
                intercept_quad = true;
            }

            if (intercept_plane && intercept_quad) {
                out.push_back(cell_id);
                break;
            }
        }
    }

    return out;
}

HexMapIterCube HexMapTiledNode::local_region_to_cell_ids(Vector3 p_a,
        Vector3 p_b,
        Planes planes) const {
    return HexMapIterCube(p_a, p_b);
}

Ref<hex_bind::HexMapIter>
HexMapTiledNode::_local_region_to_cell_ids(Vector3 p_a, Vector3 p_b) const {
    return local_region_to_cell_ids(p_a, p_b);
}

void HexMapTiledNode::_notification(int p_what) {
    static Transform3D last_transform;

    switch (p_what) {
        case NOTIFICATION_POSTINITIALIZE:
            // XXX workaround for connect during initialize
            break;

        case NOTIFICATION_ENTER_WORLD:
            for (auto &pair : octants) {
                pair.value->enter_world();
            }
            last_transform = get_global_transform();
            break;

        case NOTIFICATION_ENTER_TREE:
            _update_visibility();
            break;

        case NOTIFICATION_TRANSFORM_CHANGED: {
            Transform3D transform = get_global_transform();
            if (transform != last_transform) {
                space.set_transform(transform);
                update_octant_meshes();
                last_transform = transform;
            }
            break;
        }
        case NOTIFICATION_EXIT_WORLD:
            for (auto &pair : octants) {
                pair.value->exit_world();
            }
            break;

        case NOTIFICATION_VISIBILITY_CHANGED:
            _update_visibility();
            break;
    }
}

void HexMapTiledNode::_update_visibility() {
    if (!is_inside_tree()) {
        return;
    }

    for (auto &pair : octants) {
        pair.value->update_visibility();
    }
}

void HexMapTiledNode::update_dirty_octants_callback() {
    ERR_FAIL_COND_MSG(!awaiting_update,
            "update_dirty_octants_callback() called unexpectedly");
    Vector<OctantKey> empty_octants;
    for (const auto pair : octants) {
        Octant *octant = pair.value;
        if (!octant->is_dirty()) {
            continue;
        }

        octant->apply_changes();
        if (octant->is_empty()) {
            empty_octants.push_back(pair.key);
        }
    }

    for (const auto key : empty_octants) {
        delete octants[key];
        octants.erase(key);
    }

    // XXX why is this needed here?
    _update_visibility();
    awaiting_update = false;

    if (!updated_cells.is_empty()) {
        Array cells;
        for (const CellKey &key : updated_cells) {
            cells.push_back((Vector3i)key);
        }
        updated_cells.clear();
        emit_signal("cells_changed", cells);
    }
}

void HexMapTiledNode::update_dirty_octants() {
    if (awaiting_update) {
        return;
    }
    awaiting_update = true;
    callable_mp(this, &HexMapTiledNode::update_dirty_octants_callback)
            .call_deferred();
}

void HexMapTiledNode::update_octant_meshes() {
    UtilityFunctions::print("calling apply_changes() on all meshes");
    for (auto &it : octants) {
        it.value->apply_changes();
    }
}

void HexMapTiledNode::_recreate_octant_data() {
    HashMap<CellKey, Cell> cell_copy = cell_map;
    _clear_internal();
    for (const KeyValue<CellKey, Cell> &E : cell_copy) {
        set_cell_item(CellId(E.key), E.value.item, E.value.rot);
    }
}

void HexMapTiledNode::_clear_internal() {
    for (auto &octant_pair : octants) {
        Octant *octant = octant_pair.value;
        if (is_inside_tree()) {
            octant->exit_world();
        }
        delete octant;
    }
    octants.clear();
    cell_map.clear();
}

void HexMapTiledNode::clear() {
    _clear_internal();
    clear_baked_meshes();
}

void HexMapTiledNode::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_mesh_library", "mesh_library"),
            &HexMapTiledNode::set_mesh_library);
    ClassDB::bind_method(
            D_METHOD("get_mesh_library"), &HexMapTiledNode::get_mesh_library);

    ClassDB::bind_method(D_METHOD("set_collision_debug", "debug"),
            &HexMapTiledNode::set_collision_debug);
    ClassDB::bind_method(D_METHOD("get_collision_debug"),
            &HexMapTiledNode::get_collision_debug);

    ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"),
            &HexMapTiledNode::set_collision_layer);
    ClassDB::bind_method(D_METHOD("get_collision_layer"),
            &HexMapTiledNode::get_collision_layer);

    ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"),
            &HexMapTiledNode::set_collision_mask);
    ClassDB::bind_method(D_METHOD("get_collision_mask"),
            &HexMapTiledNode::get_collision_mask);

    ClassDB::bind_method(
            D_METHOD("set_collision_mask_value", "layer_number", "value"),
            &HexMapTiledNode::set_collision_mask_value);
    ClassDB::bind_method(D_METHOD("get_collision_mask_value", "layer_number"),
            &HexMapTiledNode::get_collision_mask_value);

    ClassDB::bind_method(
            D_METHOD("set_collision_layer_value", "layer_number", "value"),
            &HexMapTiledNode::set_collision_layer_value);
    ClassDB::bind_method(D_METHOD("get_collision_layer_value", "layer_number"),
            &HexMapTiledNode::get_collision_layer_value);

    ClassDB::bind_method(D_METHOD("set_collision_priority", "priority"),
            &HexMapTiledNode::set_collision_priority);
    ClassDB::bind_method(D_METHOD("get_collision_priority"),
            &HexMapTiledNode::get_collision_priority);

    ClassDB::bind_method(D_METHOD("set_physics_material", "material"),
            &HexMapTiledNode::set_physics_material);
    ClassDB::bind_method(D_METHOD("get_physics_material"),
            &HexMapTiledNode::get_physics_material);

    ClassDB::bind_method(
            D_METHOD("set_navigation_bake_only_navmesh_tiles", "enable"),
            &HexMapTiledNode::set_navigation_bake_only_navmesh_tiles);
    ClassDB::bind_method(D_METHOD("get_navigation_bake_only_navmesh_tiles"),
            &HexMapTiledNode::get_navigation_bake_only_navmesh_tiles);

    ClassDB::bind_method(D_METHOD("set_octant_size", "size"),
            &HexMapTiledNode::set_octant_size);
    ClassDB::bind_method(
            D_METHOD("get_octant_size"), &HexMapTiledNode::get_octant_size);

    ClassDB::bind_method(
            D_METHOD("set_cell_item", "position", "item", "orientation"),
            &HexMapTiledNode::_set_cell_item,
            DEFVAL(0));
    ClassDB::bind_method(
            D_METHOD("set_cell_item_v", "position", "item", "orientation"),
            &HexMapTiledNode::_set_cell_item_v,
            DEFVAL(0));

    ClassDB::bind_method(D_METHOD("get_cell_item", "cell_id"),
            &HexMapTiledNode::_get_cell_item);
    ClassDB::bind_method(D_METHOD("get_cell_item_v", "cell_vector"),
            &HexMapTiledNode::_get_cell_item_v);

    ClassDB::bind_method(D_METHOD("get_cell_item_orientation", "position"),
            &HexMapTiledNode::_get_cell_item_orientation);

    ClassDB::bind_method(D_METHOD("local_region_to_cell_ids",
                                 "local_point_a",
                                 "local_point_b"),
            &HexMapTiledNode::_local_region_to_cell_ids);

    ClassDB::bind_method(D_METHOD("local_to_cell_id", "local_position"),
            &HexMapTiledNode::_local_to_cell_id);
    ClassDB::bind_method(D_METHOD("cell_id_to_local", "cell_id"),
            &HexMapTiledNode::_cell_id_to_local);

    ClassDB::bind_method(D_METHOD("clear"), &HexMapTiledNode::clear);

    ClassDB::bind_method(
            D_METHOD("get_used_cells"), &HexMapTiledNode::get_used_cells);
    ClassDB::bind_method(D_METHOD("get_used_cells_by_item", "item"),
            &HexMapTiledNode::get_used_cells_by_item);

    ClassDB::bind_method(
            D_METHOD("get_bake_meshes"), &HexMapTiledNode::get_bake_meshes);
    ClassDB::bind_method(D_METHOD("get_bake_mesh_instance", "idx"),
            &HexMapTiledNode::get_bake_mesh_instance);
    ClassDB::bind_method(D_METHOD("clear_baked_meshes"),
            &HexMapTiledNode::clear_baked_meshes);

    ClassDB::bind_method(D_METHOD("make_baked_meshes",
                                 "gen_lightmap_uv",
                                 "lightmap_uv_texel_size"),
            &HexMapTiledNode::make_baked_meshes,
            DEFVAL(false),
            DEFVAL(0.1));

    ADD_PROPERTY(PropertyInfo(Variant::OBJECT,
                         "mesh_library",
                         PROPERTY_HINT_RESOURCE_TYPE,
                         "MeshLibrary"),
            "set_mesh_library",
            "get_mesh_library");
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT,
                         "physics_material",
                         PROPERTY_HINT_RESOURCE_TYPE,
                         "PhysicsMaterial"),
            "set_physics_material",
            "get_physics_material");
    ADD_GROUP("Cell", "cell_");
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "cell_octant_size",
                         PROPERTY_HINT_RANGE,
                         "1,1024,1"),
            "set_octant_size",
            "get_octant_size");
    ADD_GROUP("Collision", "collision_");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL,
                         "collision_debug",
                         PROPERTY_HINT_LAYERS_3D_PHYSICS),
            "set_collision_debug",
            "get_collision_debug");
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "collision_layer",
                         PROPERTY_HINT_LAYERS_3D_PHYSICS),
            "set_collision_layer",
            "get_collision_layer");
    ADD_PROPERTY(PropertyInfo(Variant::INT,
                         "collision_mask",
                         PROPERTY_HINT_LAYERS_3D_PHYSICS),
            "set_collision_mask",
            "get_collision_mask");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_priority"),
            "set_collision_priority",
            "get_collision_priority");
    ADD_GROUP("Navigation", "navigation_");

    ADD_PROPERTY(
            PropertyInfo(Variant::BOOL, "navigation_bake_only_navmesh_tiles"),
            "set_navigation_bake_only_navmesh_tiles",
            "get_navigation_bake_only_navmesh_tiles");

    BIND_CONSTANT(INVALID_CELL_ITEM);

    ADD_SIGNAL(MethodInfo(
            "cell_changed", PropertyInfo(Variant::VECTOR3I, "cell")));
    ADD_SIGNAL(MethodInfo(
            "cells_changed", PropertyInfo(Variant::ARRAY, "cell_id_vectors")));
}

Array HexMapTiledNode::get_used_cells() const {
    Array a;
    a.resize(cell_map.size());
    int i = 0;
    for (const KeyValue<CellKey, Cell> &E : cell_map) {
        HexMapCellId cell_id(E.key);
        a[i++] = static_cast<Ref<hex_bind::HexMapCellId>>(cell_id);
    }

    return a;
}

TypedArray<Vector3i> HexMapTiledNode::get_used_cells_by_item(
        int p_item) const {
    Array a;
    for (const KeyValue<CellKey, Cell> &E : cell_map) {
        if ((int)E.value.item == p_item) {
            HexMapCellId cell_id(E.key);
            a.push_back(static_cast<Ref<hex_bind::HexMapCellId>>(cell_id));
        }
    }

    return a;
}

void HexMapTiledNode::clear_baked_meshes() {
    for (auto &it : octants) {
        it.value->clear_baked_mesh();
    }
    baked_mesh_octants.clear();
}

void HexMapTiledNode::make_baked_meshes(bool p_gen_lightmap_uv,
        float p_lightmap_uv_texel_size) {}

Array HexMapTiledNode::get_bake_meshes() {
    baked_mesh_octants.clear();
    Array arr;
    for (auto &it : octants) {
        baked_mesh_octants.push_back(it.key);
        arr.push_back(it.value->get_baked_mesh());
        arr.push_back(Transform3D());
    }
    return arr;
}

RID HexMapTiledNode::get_bake_mesh_instance(int p_idx) {
    ERR_FAIL_INDEX_V(p_idx, baked_mesh_octants.size(), RID());
    OctantKey key = baked_mesh_octants[p_idx];
    ERR_FAIL_COND_V_MSG(
            !octants.has(key), RID(), "octant not found for baked mesh index");
    return octants.get(key)->get_baked_mesh_instance();
}

bool HexMapTiledNode::generate_navigation_source_geometry(Ref<NavigationMesh>,
        Ref<NavigationMeshSourceGeometryData3D> source_geometry_data,
        Node *) const {
    UtilityFunctions::print("navigation source generator");

    for (const auto &it : cell_map) {
        const Cell &cell = it.value;

        if (navigation_bake_only_navmesh_tiles) {
            // If there's a tile in the cell above this one, do not include
            // this tile, otherwise when a navigable mesh has a
            // non-navigable on top, the nav mesh incorrectly cuts through
            // that upper tile.
            if (it.key.y < SHRT_MAX && cell_map.has(it.key.get_cell_above())) {
                continue;
            }

            // if the cell doesn't have a navmesh, skip it
            if (!mesh_library->get_item_navigation_mesh(cell.item)
                            .is_valid()) {
                continue;
            }
        }

        Ref<Mesh> mesh = mesh_library->get_item_mesh(cell.item);
        if (!mesh.is_valid()) {
            continue;
        }

        Transform3D cell_transform(
                cell.get_basis(), space.get_mesh_origin(it.key));
        source_geometry_data->add_mesh(mesh,
                cell_transform *
                        mesh_library->get_item_mesh_transform(cell.item));
    }

    // Unused return value.  To turn this function into a Callable, the
    // return type needs to be able to be converted into a Variant.  `void`
    // is not a supported option.
    return true;
}

HexMapTiledNode::HexMapTiledNode() {
    set_notify_transform(true);

    // copied from SceneTree::get_debug_collision_material()
    collision_debug_mat.instantiate();
    collision_debug_mat->set_shading_mode(
            StandardMaterial3D::SHADING_MODE_UNSHADED);
    collision_debug_mat->set_transparency(
            StandardMaterial3D::TRANSPARENCY_ALPHA);
    collision_debug_mat->set_flag(
            StandardMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);
    collision_debug_mat->set_flag(
            StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    collision_debug_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
    collision_debug_mat->set_albedo(
            ProjectSettings::get_singleton()->get_setting_with_override(
                    "debug/shapes/collision/shape_color"));

    // set up the source geometry parser for navmesh generation
    NavigationServer3D *ns = NavigationServer3D::get_singleton();
    navigation_source_geometry_parser = ns->source_geometry_parser_create();
    ns->source_geometry_parser_set_callback(navigation_source_geometry_parser,
            callable_mp(this,
                    &HexMapTiledNode::generate_navigation_source_geometry));
}

HexMapTiledNode::~HexMapTiledNode() {
    clear();
    NavigationServer3D::get_singleton()->free_rid(
            navigation_source_geometry_parser);
}
