#ifdef TOOLS_ENABLED

#include <cassert>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/window.hpp>
#include <godot_cpp/classes/world3d.hpp>

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

    HexMapSpace space;
    cell_mesh = space.build_cell_mesh();
    cell_mesh->surface_set_material(0, mesh_mat);
    cell_mesh->surface_set_material(1, line_mat);
}

void SelectionManager::redraw_selection() {
    RenderingServer *rs = RS::get_singleton();

    // transform the mesh manager and redraw it
    Vector<CellId> cells = get_cell_ids();
    mesh_manager.clear();
    set_cells(cells);

    // make sure it is visible
    mesh_manager.set_visible(true);
}

void SelectionManager::clear() { mesh_manager.clear(); }

size_t SelectionManager::size() const {
    return mesh_manager.get_cells().size();
}

bool SelectionManager::is_empty() const {
    return mesh_manager.get_cells().is_empty();
};

void SelectionManager::add_cell(CellId cell) {
    assert(cell_mesh.is_valid() && "mesh should be valid");
    mesh_manager.set_cell(cell,
            cell_mesh->get_rid(),
            Basis::from_scale(mesh_manager.get_space().get_cell_scale()));
    mesh_manager.refresh();
}

void SelectionManager::set_cells(Vector<CellId> other) {
    mesh_manager.clear();

    Transform3D mesh_transform(
            Basis::from_scale(mesh_manager.get_space().get_cell_scale()));
    for (const CellId &cell : other) {
        mesh_manager.set_cell(cell, cell_mesh->get_rid(), mesh_transform);
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
        mesh_manager.set_cell(cell, cell_mesh->get_rid(), mesh_transform);
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

Vector<HexMapCellId> SelectionManager::get_cell_ids() const {
    Vector<HexMapCellId> out;
    for (const auto &cell : mesh_manager.get_cells()) {
        out.push_back((CellId)cell.key);
    }
    return out;
}

Array SelectionManager::get_cell_vecs() const {
    Array out;
    for (const auto &cell : mesh_manager.get_cells()) {
        out.push_back(((CellId)cell.key).to_vec());
    }
    return out;
}

void SelectionManager::set_space(HexMapSpace space) {
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
}

#endif
