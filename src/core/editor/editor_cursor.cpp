#ifdef TOOLS_ENABLED

#include <cassert>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "core/cell_id.h"
#include "core/hex_map_node.h"
#include "core/iter_radial.h"
#include "core/math.h"
#include "editor_cursor.h"
#include "profiling.h"

using RS = RenderingServer;

/// radius in cells of the edit grid
#define GRID_RADIUS 40u

void EditorCursor::set_space(HexMapSpace space) {
    bool scale_changed =
            space.get_cell_scale() != parent_space.get_cell_scale();
    parent_space = space;

    // rebuild the grid mesh if the cell scale changed
    if (scale_changed) {
        set_axis(edit_axis);
    }
    transform_meshes();
}

void EditorCursor::set_mesh_library(Ref<MeshLibrary> value) {
    mesh_tool.set_mesh_library(value);
}

void EditorCursor::set_mesh_origin(Vector3 value) {
    mesh_tool.set_mesh_origin(value);
}

void EditorCursor::set_cells_visibility_callback(Callable value) {
    set_cells_visibility = value;
}

void EditorCursor::set_orientation(HexMapTileOrientation orientation) {
    this->orientation = orientation;
    transform_meshes();
}

HexMapTileOrientation EditorCursor::get_orientation() const {
    return orientation;
}

void EditorCursor::transform_meshes() {
    // update the grid transform
    grid_mesh_transform.set_origin(
            parent_space.get_cell_center_global(pointer_cell) -
            Vector3(0, parent_space.get_cell_height() / 2, 0));
    RenderingServer::get_singleton()->instance_set_transform(
            grid_mesh_instance, grid_mesh_transform);

    // update the mesh_tool space to include the new rotation
    Transform3D cursor_transform(
            orientation, parent_space.get_cell_center(pointer_cell));
    HexMapSpace space = parent_space;
    space.set_transform(space.get_transform() * cursor_transform);
    mesh_tool.set_space(space);
    // XXX we're destroying multimesh and rebuilding when we call this; we
    // should make MeshTool smarter about what work it needs to do.
    mesh_tool.refresh();

    if (!set_cells_visibility.is_valid()) {
        return;
    }

    // To prevent the HexMapNode cells from hiding the cursor cells (by having
    // larger meshes, etc), we need to get a the list of cells the
    // cursor now overlaps, and hide those.  Additionally we need to show any
    // cells we no longer overlap.
    //
    // In this loop we'll construct an array of cell id followed by visibility
    // state (true = visible, false = hidden), for only those cells that need
    // to change.
    //
    auto profile = profiling_begin("EditorCursor: updating hidden cell list");
    Array changes; // changes to call set_cells_visibility()

    // make a copy of the set of hidden cells; we'll use this to detect which
    // cells visibiliby should be changed.
    HashSet<HexMapCellId::Key> current_cells = hidden_cells;

    // iterate through all the cells in the editor cursor
    for (const auto &iter : mesh_tool.get_cells()) {
        // the MeshTool may be rotated, so we need to map its cell id to the
        // parent space's cell id.  We do this by grabbing the global
        // coordinates for the center of the cell in the mesh_tool, then
        // getting the cell id for that point in our parent space.
        Vector3 center = space.get_cell_center_global(iter.key);
        HexMapCellId cell_id = parent_space.get_cell_id_global(center);

        // if the cell id is currently hidden, remove it
        if (!current_cells.erase(cell_id)) {
            // cell id is not already hidden; add it to our hidden list and add
            // the arguments to hide it to our changes Array.
            hidden_cells.insert(cell_id);
            changes.push_back((Vector3i)cell_id);
            changes.push_back(false);
        }
    }

    // we removed all the cursor cell ids from the current cells list, so
    // anything left is no longer covered by the editor cursor and should be
    // unhidden.
    for (const auto key : current_cells) {
        hidden_cells.erase(key);
        changes.push_back((Vector3i)key);
        changes.push_back(true);
    }

    // call the callback if we have any changes
    if (!changes.is_empty()) {
        set_cells_visibility.call(changes);
    }
}

void EditorCursor::clear_tiles() {
    mesh_tool.clear();
    set_orientation(HexMapTileOrientation::Upright0);
}

void EditorCursor::set_tile(HexMapCellId cell,
        int tile,
        HexMapTileOrientation orientation) {
    mesh_tool.set_cell(cell, tile, orientation);
    // explicitly not calling transform_meshes() here because multiple tiles
    // may be set in a single frame.
}

bool EditorCursor::is_empty() const {
    return mesh_tool.get_cells().is_empty();
};

size_t EditorCursor::size() const { return mesh_tool.get_cells().size(); };

Array EditorCursor::get_cells_v() const {
    const HexMapSpace &space = mesh_tool.get_space();
    const auto cells = mesh_tool.get_cells();
    auto cell_iter = cells.begin();

    unsigned len = cells.size() * HexMapNode::CELL_ARRAY_WIDTH;
    Array out;
    out.resize(len);

    for (int i = 0; i < len; i += HexMapNode::CELL_ARRAY_WIDTH) {
        Vector3 center = space.get_cell_center_global(cell_iter->key);
        Vector3i cell_id = parent_space.get_cell_id_global(center);
        out[i] = (Vector3i)cell_id;
        out[i + 1] = cell_iter->value.index;
        out[i + 2] = cell_iter->value.orientation + orientation;
        ++cell_iter;
    }

    return out;
}

Array EditorCursor::get_cell_ids_v() const {
    const HexMapSpace &space = mesh_tool.get_space();
    const auto cells = mesh_tool.get_cells();
    auto cell_iter = cells.begin();

    unsigned len = cells.size();
    Array out;
    out.resize(len);

    for (int i = 0; i < len; ++i, ++cell_iter) {
        Vector3 center = space.get_cell_center_global(cell_iter->key);
        Vector3i cell_id = parent_space.get_cell_id_global(center);
        out[i] = cell_id;
    }

    return out;
}

HexMapNode::CellInfo EditorCursor::get_origin_cell_info() const {
    const HexMapLibraryMeshTool::CellState *cell =
            mesh_tool.get_cells().getptr(HexMapCellId());
    if (cell == nullptr) {
        return HexMapNode::CellInfo{};
    }
    return HexMapNode::CellInfo{
        .value = cell->index,
        .orientation = cell->orientation + orientation,
    };
}

bool EditorCursor::update(const Camera3D *camera,
        const Point2 &pointer,
        const HexMapNode *hex_map,
        Vector3 *point) {
    ERR_FAIL_COND_V_MSG(
            camera == nullptr, false, "null camera in EditorCursor.update()");
    ERR_FAIL_COND_V_MSG(hex_map == nullptr,
            false,
            "null hex_map in EditorCursor.update()");

    Transform3D local_transform =
            parent_space.get_transform().affine_inverse();
    Vector3 origin = camera->project_ray_origin(pointer);
    Vector3 normal = camera->project_ray_normal(pointer);
    origin = local_transform.xform(origin);
    normal = local_transform.basis.xform(normal).normalized();

    Vector3 pos;
    if (!edit_plane.intersects_ray(origin, normal, &pos)) {
        return false;
    }
    last_update_origin = origin;
    last_update_normal = normal;

    auto space = hex_map->get_world_3d()->get_direct_space_state();
    auto raycast = space->intersect_ray(PhysicsRayQueryParameters3D::create(
            camera->project_ray_origin(pointer), pos, 1, {}));
    if (!raycast.is_empty()) {
        auto position = raycast.get("position", 0);
        if (position.get_type() == Variant::Type::VECTOR3) {
            pos = position;
        }
    }
    if (point != nullptr) {
        *point = pos;
    }
    pointer_pos = pos;

    HexMapCellId cell = parent_space.get_cell_id(pos);
    if (hex_map->get_cell(cell).value != HexMapNode::CELL_VALUE_NONE) {
        // currently if cell is occupied we will just query along the
        // work axis until its unoccupied, but another valid choice is to
        // step towards nearest cell in direction of camera

        // naively step "upwards" along work axis, does not account for if
        // camera is below work plane
        cell = cell + HexMapCellId(edit_plane.normal);
    }
    if (hex_map->get_cell(cell).value != HexMapNode::CELL_VALUE_NONE) {
        return false;
    }

    if (cell == pointer_cell) {
        return false;
    }
    pointer_cell = cell;

    transform_meshes();

    return true;
}

bool EditorCursor::update(const Camera3D *camera,
        const Point2 &pointer,
        Vector3 *point) {
    ERR_FAIL_COND_V_MSG(
            camera == nullptr, false, "null camera in EditorCursor.update()");

    Transform3D local_transform =
            parent_space.get_transform().affine_inverse();
    Vector3 origin = camera->project_ray_origin(pointer);
    Vector3 normal = camera->project_ray_normal(pointer);
    origin = local_transform.xform(origin);
    normal = local_transform.basis.xform(normal).normalized();

    Vector3 pos;
    if (!edit_plane.intersects_ray(origin, normal, &pos)) {
        return false;
    }
    last_update_origin = origin;
    last_update_normal = normal;

    if (point != nullptr) {
        *point = pos;
    }
    pointer_pos = pos;

    HexMapCellId cell = parent_space.get_cell_id(pos);
    if (cell == pointer_cell) {
        return false;
    }
    pointer_cell = cell;

    transform_meshes();

    return true;
}

void EditorCursor::update(bool force) {
    Vector3 pos;
    if (!edit_plane.intersects_ray(
                last_update_origin, last_update_normal, &pos)) {
        return;
    }
    pointer_pos = pos;

    HexMapCellId cell = parent_space.get_cell_id(pos);
    if (!force && cell == pointer_cell) {
        return;
    }
    pointer_cell = cell;

    transform_meshes();
}

HexMapCellId EditorCursor::get_cell() const { return pointer_cell; };

Vector3 EditorCursor::get_pos() const { return pointer_pos; };

void EditorCursor::set_visible(bool visible) {
    mesh_tool.set_visible(visible);
}

bool EditorCursor::get_visible() const { return mesh_tool.get_visible(); }

void EditorCursor::set_axis(EditAxis axis) {
    // some axis values may have rotated the grid mesh transform.  We want to
    // reset that rotation while preserving the origin.
    grid_mesh_transform = Transform3D();

    // clear the grid mesh
    RenderingServer *rs = RenderingServer::get_singleton();
    rs->mesh_clear(grid_mesh);

    switch (axis) {
    case EditAxis::AXIS_Y:
        build_y_grid();
        edit_plane.normal = Vector3(0, 1, 0);
        break;
    case EditAxis::AXIS_Q:
        build_r_grid();
        grid_mesh_transform.rotate(Vector3(0, 1, 0), -Math_PI / 3.0);
        edit_plane.normal = Vector3(Math_SQRT3_2, 0, -0.5).normalized();
        break;
    case EditAxis::AXIS_R:
        build_r_grid();
        edit_plane.normal = Vector3(0, 0, 1);
        break;
    case EditAxis::AXIS_S:
        build_r_grid();
        grid_mesh_transform.rotate(Vector3(0, 1, 0), Math_PI / 3.0);
        edit_plane.normal = Vector3(Math_SQRT3_2, 0, 0.5).normalized();
        break;
    }

    grid_mesh_transform.scale(parent_space.get_cell_scale());
    RenderingServer::get_singleton()->instance_set_transform(
            grid_mesh_instance, grid_mesh_transform);

    edit_axis = axis;
    update(true);
}

EditorCursor::EditAxis EditorCursor::get_axis() const { return edit_axis; };

void EditorCursor::set_depth(int depth) {
    Vector3 cell_scale = parent_space.get_cell_scale();
    real_t cell_depth;

    switch (edit_axis) {
    case EditAxis::AXIS_Y:
        // the y plane is at the bottom of the cell, but to avoid floating
        // point errors during raycast, we pull it slightly higher into the
        // cell.
        edit_plane.d = depth * cell_scale.y + (cell_scale.y * 0.1);
        break;
    case EditAxis::AXIS_Q:
    case EditAxis::AXIS_R:
    case EditAxis::AXIS_S:
        // Q/R/S plane goes through the center of the cell, so no floating
        // point concerns here.
        edit_plane.d = depth * cell_scale.x * 1.5;
        break;
    }
    update(true);
}

// create a mesh and draw a grid of hexagonal cells on it
void EditorCursor::build_y_grid() {
    // create the points that make up the top of a hex cell
    Vector<Vector3> shape_points;
    shape_points.append(Vector3(0.0, 0, -1.0));
    shape_points.append(Vector3(-Math_SQRT3_2, 0, -0.5));
    shape_points.append(Vector3(-Math_SQRT3_2, 0, 0.5));
    shape_points.append(Vector3(0.0, 0, 1.0));
    shape_points.append(Vector3(Math_SQRT3_2, 0, 0.5));
    shape_points.append(Vector3(Math_SQRT3_2, 0, -0.5));
    shape_points.append(Vector3(0.0, 0, -1.0));

    // pick a point on the middle of an edge to find the closest edge of the
    // cells
    float max = HexMapCellId(GRID_RADIUS, -(int)GRID_RADIUS / 2, 0)
                        .unit_center()
                        .length_squared();
    PackedVector3Array grid_points;
    PackedColorArray grid_colors;
    for (const auto cell :
            HexMapCellId::ZERO.get_neighbors(GRID_RADIUS, HexMapPlanes::QRS)) {
        Vector3 center = cell.unit_center();
        float transparency =
                Math::pow(MAX(0, (max - center.length_squared()) / max), 2);

        for (int j = 1; j < shape_points.size(); j++) {
            grid_points.append(center + shape_points[j - 1]);
            grid_points.append(center + shape_points[j]);
            grid_colors.push_back(Color(1, 1, 1, transparency));
            grid_colors.push_back(Color(1, 1, 1, transparency));
        }
    }

    Array d;
    d.resize(RS::ARRAY_MAX);
    d[RS::ARRAY_VERTEX] = grid_points;
    d[RS::ARRAY_COLOR] = grid_colors;

    RenderingServer *rs = RenderingServer::get_singleton();
    rs->mesh_add_surface_from_arrays(
            grid_mesh, RenderingServer::PRIMITIVE_LINES, d);
    rs->mesh_surface_set_material(grid_mesh, 0, grid_mat->get_rid());
}

void EditorCursor::build_r_grid() {
    // create the points that traverse from the center of one edge to the
    // opposite edge
    Array shape_points;
    shape_points.append(Vector3(-Math_SQRT3_2, 0.5, 0));
    shape_points.append(Vector3(-Math_SQRT3_2, -0.5, 0));
    shape_points.append(Vector3(Math_SQRT3_2, -0.5, 0));
    shape_points.append(Vector3(Math_SQRT3_2, 0.5, 0));
    shape_points.append(Vector3(-Math_SQRT3_2, 0.5, 0));

    float max = HexMapCellId(0, 0, GRID_RADIUS).unit_center().length_squared();
    PackedVector3Array grid_points;
    PackedColorArray grid_colors;
    for (const auto cell :
            HexMapCellId::ZERO.get_neighbors(GRID_RADIUS, HexMapPlanes::YQS)) {
        Vector3 center = cell.unit_center();
        float transparency =
                Math::pow(MAX(0, (max - center.length_squared()) / max), 2);

        for (int j = 1; j < shape_points.size(); j++) {
            grid_points.append(center + shape_points[j - 1]);
            grid_points.append(center + shape_points[j]);
            grid_colors.push_back(Color(1, 1, 1, transparency));
            grid_colors.push_back(Color(1, 1, 1, transparency));
        }
    }

    Array d;
    d.resize(RS::ARRAY_MAX);
    d[RS::ARRAY_VERTEX] = grid_points;
    d[RS::ARRAY_COLOR] = grid_colors;

    RenderingServer *rs = RenderingServer::get_singleton();
    rs->mesh_add_surface_from_arrays(
            grid_mesh, RenderingServer::PRIMITIVE_LINES, d);
    rs->mesh_surface_set_material(grid_mesh, 0, grid_mat->get_rid());
}

EditorCursor::EditorCursor(RID scenario) {
    // create the grid material
    grid_mat.instantiate();
    grid_mat->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
    grid_mat->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    grid_mat->set_flag(StandardMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);
    grid_mat->set_flag(
            StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
    grid_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
    grid_mat->set_albedo(Color(0.8, 0.5, 0.1));

    // create the grid mesh
    RenderingServer *rs = RenderingServer::get_singleton();
    grid_mesh = rs->mesh_create();
    grid_mesh_instance = rs->instance_create2(grid_mesh, scenario);
    // 24 = Node3DEditorViewport::MISC_TOOL_LAYER
    rs->instance_set_layer_mask(grid_mesh_instance, 1 << 24);

    mesh_tool.set_scenario(scenario);
    mesh_tool.set_visible(true);

    set_axis(edit_axis);
    set_depth(0);
    transform_meshes(); // to account for HexMap global transform
}

EditorCursor::~EditorCursor() {
    // free the grid meshes
    RenderingServer *rs = RenderingServer::get_singleton();
    rs->free_rid(grid_mesh);
    rs->free_rid(grid_mesh_instance);
}

#endif
