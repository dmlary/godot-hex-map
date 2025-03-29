#include <cassert>
#ifdef TOOLS_ENABLED

#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/classes/world3d.hpp>

#include "core/math.h"
#include "selection_manager.h"

using CellId = HexMapCellId;
using RS = RenderingServer;

void SelectionManager::build_cell_mesh() {
    mesh_mat.instantiate();
    mesh_mat->set_albedo(Color(0.7, 0.7, 1.0, 0.2));
    mesh_mat->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
    mesh_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
    mesh_mat->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);

    line_mat.instantiate();
    line_mat->set_albedo(Color(0.7, 0.7, 1.0, 0.8));
    // outer_mat->set_on_top_of_alpha(); ->
    line_mat->set_transparency(godot::BaseMaterial3D::TRANSPARENCY_DISABLED);
    line_mat->set_render_priority(
            RenderingServer::MATERIAL_RENDER_PRIORITY_MAX);
    line_mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);

    line_mat->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
    line_mat->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
    line_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);

    Array mesh_array, lines_array;
    mesh_array.resize(RS::ARRAY_MAX);
    lines_array.resize(RS::ARRAY_MAX);

    /*
     *               (0)             Y
     *              /   \            |
     *           (1)     (5)         o---X
     *            |       |           \
     *           (2)     (4)           Z
     *            | \   / |
     *            |  (3)  |
     *            |   |   |
     *            |  (6)  |
     *            | / | \ |
     *           (7)  |  (b)
     *            |   |   |
     *           (8)  |  (a)
     *              \ | /
     *               (9)
     */
    mesh_array[RS::ARRAY_VERTEX] = lines_array[RS::ARRAY_VERTEX] =
            PackedVector3Array({
                    Vector3(0.0, 0.5, -1.0), // 0
                    Vector3(-Math_SQRT3_2, 0.5, -0.5), // 1
                    Vector3(-Math_SQRT3_2, 0.5, 0.5), // 2
                    Vector3(0.0, 0.5, 1.0), // 3
                    Vector3(Math_SQRT3_2, 0.5, 0.5), // 4
                    Vector3(Math_SQRT3_2, 0.5, -0.5), // 5
                    Vector3(0.0, -0.5, -1.0), // 6
                    Vector3(-Math_SQRT3_2, -0.5, -0.5), // 7
                    Vector3(-Math_SQRT3_2, -0.5, 0.5), // 8
                    Vector3(0.0, -0.5, 1.0), // 9
                    Vector3(Math_SQRT3_2, -0.5, 0.5), // 10 (0xa)
                    Vector3(Math_SQRT3_2, -0.5, -0.5), // 11 (0xb)
            });

    // clang-format off
	mesh_array[RS::ARRAY_INDEX] = PackedInt32Array({
		// top
		0, 5, 1,
		1, 5, 2,
		2, 5, 4,
		2, 4, 3,
		// bottom
		6, 7, 11,
		11, 7, 8,
		8, 10, 11,
		10, 8, 9,
		// east
		4,  5, 11,
		11, 10, 4,
		// northeast
		5, 0,  6,
		6, 11, 5,
		// northwest
		0, 1, 7,
		7, 6, 0,
		// west
		1, 2, 8,
		8, 7, 1,
		// southwest
		2, 3, 9,
		9, 8, 2,
		// southeast
		3, 4, 10,
		10, 9, 3,
	});
    // clang-format on

    lines_array[RS::ARRAY_INDEX] = PackedInt32Array({
            0,
            1,
            2,
            3,
            4,
            5, // top
            11,
            6,
            7,
            8,
            9,
            10, // bottom
            11,
            5,
            0,
            6, // northeast face
            7,
            1,
            2,
            8, // west face
            9,
            3,
            4,
            10, // southeast face
    });

    RenderingServer *rs = RS::get_singleton();
    cell_mesh = rs->mesh_create();
    rs->mesh_add_surface_from_arrays(
            cell_mesh, RS::PRIMITIVE_TRIANGLES, mesh_array);
    rs->mesh_surface_set_material(cell_mesh, 0, mesh_mat->get_rid());

    // add lines around the cell
    rs->mesh_add_surface_from_arrays(
            cell_mesh, RS::PRIMITIVE_LINE_STRIP, lines_array);
    rs->mesh_surface_set_material(cell_mesh, 1, line_mat->get_rid());
}

void SelectionManager::redraw_selection() {
    RenderingServer *rs = RS::get_singleton();

    // transform the mesh manager and redraw it
    Vector<CellId> cells = get_cells();
    mesh_manager.clear();
    set_cells(cells);

    // make sure it is visible
    mesh_manager.set_visibility(true);
}

void SelectionManager::clear() { mesh_manager.clear(); }

void SelectionManager::add_cell(CellId cell) {
    assert(cell_mesh.is_valid() && "mesh should be valid");
    mesh_manager.set_cell(cell,
            cell_mesh,
            Basis::from_scale(mesh_manager.get_space().get_cell_scale()));
    mesh_manager.refresh();
}

void SelectionManager::set_cells(Vector<CellId> other) {
    mesh_manager.clear();

    Transform3D mesh_transform(
            Basis::from_scale(mesh_manager.get_space().get_cell_scale()));
    for (const CellId &cell : other) {
        mesh_manager.set_cell(cell, cell_mesh, mesh_transform);
    }
    mesh_manager.refresh();
}

void SelectionManager::set_cells(Array other) {
    mesh_manager.clear();

    Transform3D mesh_transform(
            Basis::from_scale(mesh_manager.get_space().get_cell_scale()));
    auto size = other.size();
    for (int i = 0; i < size; i++) {
        Vector3i cell = other[i];
        mesh_manager.set_cell(cell, cell_mesh, mesh_transform);
    }
    mesh_manager.refresh();
}

CellId SelectionManager::get_center() {
    auto cells = mesh_manager.get_cells();
    if (cells.is_empty()) {
        return CellId();
    }

    CellId first = cells.begin()->key;
    Vector3 center = first.unit_center();
    Vector3 min = center, max = center;
    for (const auto &it : cells) {
        CellId cell = it.key;
        center = cell.unit_center();
        if (center.x < min.x) {
            min.x = center.x;
        } else if (center.x > max.x) {
            max.x = center.x;
        }
        if (center.y < min.y) {
            min.y = center.y;
        } else if (center.y > max.y) {
            max.y = center.y;
        }
        if (center.z < min.z) {
            min.z = center.z;
        } else if (center.z > max.z) {
            max.z = center.z;
        }
    }

    return CellId::from_unit_point((max + min) / 2);
}

Vector<HexMapCellId> SelectionManager::get_cells() const {
    Vector<HexMapCellId> out;
    for (const auto &cell : mesh_manager.get_cells()) {
        out.push_back((CellId)cell.key);
    }
    return out;
}

Array SelectionManager::get_cells_v() const {
    Array out;
    for (const auto &cell : mesh_manager.get_cells()) {
        out.push_back((CellId)cell.key);
    }
    return out;
}

void SelectionManager::set_space(HexMapSpace space) {
    space.set_mesh_offset(Vector3());
    bool redraw = space.get_cell_scale() !=
            mesh_manager.get_space().get_cell_scale();
    mesh_manager.set_space(space);
    if (redraw) {
        redraw_selection();
    } else {
        mesh_manager.refresh();
    }
}

SelectionManager::SelectionManager(RID scenario) {
    build_cell_mesh();
    assert(cell_mesh.is_valid() && "mesh should be valid");

    mesh_manager.set_scenario(scenario);
}

SelectionManager::~SelectionManager() {
    RenderingServer *rs = RS::get_singleton();
    mesh_manager.clear();
    rs->free_rid(cell_mesh);
}

#endif
