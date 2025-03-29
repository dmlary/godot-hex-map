#pragma once
#include <godot_cpp/classes/mesh_library.hpp>

#include "cell_id.h"
#include "mesh_tool.h"
#include "tile_orientation.h"

using namespace godot;

/// Wrapper around HexMapMeshTool with added support for looking up the mesh
/// from a MeshLibrary.
class HexMapLibraryMeshTool : public HexMapMeshTool {
public:
    /// Mesh configuration for a cell
    struct CellState {
        /// `MeshLibrary` item index
        int index = -1;

        /// Mesh orientation
        HexMapTileOrientation orientation;
    };

    using CellMap = HashMap<HexMapCellId::Key, CellState>;

    /// Set the hex space parameters
    void set_space(const HexMapSpace &);

    /// Set the `MeshLibrary`; will take effect on next `refresh()` call
    void set_mesh_library(Ref<MeshLibrary> &);
    inline Ref<MeshLibrary> get_mesh_library() const { return mesh_library; }

    // delete parent implementation
    void set_cell(CellId, RID, Transform3D) = delete;

    /// Set the mesh & rotation for a given cell from a MeshLibrary.
    ///
    /// This helper will get the mesh and mesh transform from the mesh library
    /// by the provided `index`.  If the `mesh_library` is null, or the `index`
    /// does not exist in the `mesh_library`, a placeholder mesh will be drawn
    /// in the cell.
    void
    set_cell(HexMapCellId cell, int index, HexMapTileOrientation orientation);

    /// clear the mesh for `cell`
    void clear_cell(const HexMapCellId &cell_id);

    /// Get the list of cells & mesh details
    inline const CellMap &get_cells() const { return cell_map; };

    /// Rebuild the meshes
    void refresh();

    /// Clear the meshes
    void clear();

private:
    /// get the cached `Ref<Mesh>` we use when a mesh is not found in the
    /// `mesh_library`
    Ref<ArrayMesh> get_placeholder_mesh();

    Ref<MeshLibrary> mesh_library;
    CellMap cell_map;
    Ref<ArrayMesh> placeholder_mesh;

    /// flag to denote that the `HexMapMeshTool` must be rebuilt from the
    /// `cell_map`.
    bool rebuild = false;
};
