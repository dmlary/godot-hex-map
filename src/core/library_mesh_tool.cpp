#include <godot_cpp/classes/rendering_server.hpp>

#include "../profiling.h"
#include "cell_id.h"
#include "library_mesh_tool.h"
#include "math.h"
#include "space.h"

void HexMapLibraryMeshTool::set_space(const HexMapSpace &value) {
    // only need to rebuild if the cell scale or mesh offset changes.
    if (value.get_cell_scale() != space.get_cell_scale()) {
        rebuild = true;
    }
    HexMapMeshTool::set_space(value);
}

void HexMapLibraryMeshTool::set_mesh_library(Ref<MeshLibrary> &value) {
    if (value != mesh_library) {
        mesh_library = value;
        rebuild = true;
    }
}

void HexMapLibraryMeshTool::set_cell(HexMapCellId cell_id,
        int index,
        HexMapTileOrientation orientation) {
    Transform3D mesh_transform;
    Ref<Mesh> mesh;

    // try to get the mesh from the MeshLibrary
    if (mesh_library.is_valid()) {
        mesh = mesh_library->get_item_mesh(index);
        mesh_transform = mesh_library->get_item_mesh_transform(index);
    }

    // mesh not found in MeshLibrary; use placeholder
    if (!mesh.is_valid()) {
        mesh = get_placeholder_mesh();
        mesh_transform = Transform3D(Basis::from_scale(space.get_cell_scale()),
                -get_mesh_origin() * space.get_cell_scale());
    }

    Transform3D cell_transform(orientation);
    HexMapMeshTool::set_cell(
            cell_id, mesh->get_rid(), cell_transform * mesh_transform);

    cell_map.insert(cell_id,
            CellState{
                    .index = index,
                    .orientation = orientation,
            });
}

void HexMapLibraryMeshTool::clear_cell(const HexMapCellId &cell_id) {
    HexMapMeshTool::clear_cell(cell_id);
    cell_map.erase(cell_id);
}

Ref<ArrayMesh> HexMapLibraryMeshTool::get_placeholder_mesh() {
    if (placeholder_mesh.is_valid()) {
        return placeholder_mesh;
    }

    Ref<StandardMaterial3D> surface_mat;
    surface_mat.instantiate();
    surface_mat->set_albedo(Color(1.7, 0.1, 0.1, 0.7));
    surface_mat->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
    surface_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
    surface_mat->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);

    placeholder_mesh = space.build_cell_mesh();
    placeholder_mesh->surface_set_material(0, surface_mat);
    return placeholder_mesh;
}

void HexMapLibraryMeshTool::refresh() {
    // when do we need to rebuild the underlying HexMapMeshTool
    // - mesh_library changed; all new mesh ids
    // - space changed; all new transform
    // - cell_set; maybe not, maybe we handle it directly in set_cell?
    // - cell cleared: no, pass it through in clear_cell()
    // - cell visibility changed: no
    // purpose: need to preserve MeshTool state for visibility to work

    if (rebuild) {
        auto prof = profiling_begin("HexMapLibraryMeshTool: rebuilding inner");
        rebuild = false;
        HexMapMeshTool::clear();

        if (!mesh_library.is_valid()) {
            // if the MeshLibrary isn't set, use all placeholder meshes

            RID mesh = get_placeholder_mesh()->get_rid();
            Transform3D transform =
                    Transform3D(Basis::from_scale(space.get_cell_scale()),
                            -get_mesh_origin());

            for (const auto &iter : cell_map) {
                HexMapMeshTool::set_cell(iter.key, mesh, transform);
            }
        } else {
            // mesh_library is valid, use it to look up meshes

            for (const auto &iter : cell_map) {
                // get the mesh and any mesh transform
                Transform3D mesh_transform;
                Ref<Mesh> mesh = mesh_library->get_item_mesh(iter.value.index);
                if (mesh.is_valid()) {
                    mesh_transform = mesh_library->get_item_mesh_transform(
                            iter.value.index);
                } else {
                    // mesh not found in MeshLibrary; use placeholder
                    mesh = get_placeholder_mesh();
                    mesh_transform = Transform3D(
                            Basis::from_scale(space.get_cell_scale()),
                            -get_mesh_origin());
                }

                // get the cell transform based on cell orientation
                Transform3D cell_transform(iter.value.orientation);

                HexMapMeshTool::set_cell(iter.key,
                        mesh->get_rid(),
                        cell_transform * mesh_transform);
            }
        }
    }

    HexMapMeshTool::refresh();
}

void HexMapLibraryMeshTool::clear() {
    cell_map.clear();
    HexMapMeshTool::clear();
}
