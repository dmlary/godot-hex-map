#pragma once

#include <cstdint>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include "cell_id.h"
#include "space.h"

using namespace godot;

class HexMapLibraryMeshTool;

class HexMapMeshTool {
public:
    /// Type used as a key when looking up cells
    using CellId = HexMapCellId;
    using CellKey = CellId::Key;

    /// cell state
    struct Cell {
        /// mesh to show in the cell
        RID mesh;
        /// mesh transform including rotation
        Transform3D transform;
        /// flag for mesh visibility; set to false to exclude from multimesh
        bool visible = true;
    };

    HexMapMeshTool(RID scenario = RID(), uint64_t object_id = 0) :
            scenario(scenario), object_id(object_id) {};
    ~HexMapMeshTool();

    /// Set the scenario to add meshes to
    inline void set_scenario(RID value) { scenario = value; };

    /// Set the object id for all mesh instances; applies on next refresh()
    inline void set_object_id(ObjectID value) { object_id = value; };
    inline void set_object_id(uint64_t value) { object_id = (ObjectID)value; };

    /// Set the hex space parameters
    void set_space(const HexMapSpace &);

    /// Get the `HexSpace` for this MeshManager
    inline const HexMapSpace &get_space() const { return space; }

    /// Set the mesh and mesh instance transform for a given cell.  The
    /// transform is relative to the center of the cell.
    void set_cell(CellId cell_id,
            RID mesh,
            Transform3D mesh_transform = Transform3D());

    /// clear the specified cell
    void clear_cell(CellId);

    /// set per-cell visibility
    ///
    /// If the cell has not been added to the `HexMapMeshTool`, this will have
    /// no affect.  If the cell is set after visibility has been changed, the
    /// cell will become visible.
    void set_cell_visibility(CellId, bool visible);

    /// make all cells in the mesh visible
    ///
    /// This function is provided to easily restore cell visibility without the
    /// caller having to maintain a list of every hidden cell.
    void set_all_cells_visible();

    /// get an iterator to go over the cell ids in the mesh
    const HashMap<CellKey, Cell> &get_cells() const { return cell_map; };

    /// update the meshes that are displayed
    void refresh();

    /// clear all cells in the mesh
    void clear();

    /// set the visibility of any meshes present
    void set_visibility(bool visible);

    /// helper function to simplify needed calls when hexmap enters world
    void enter_world(RID scenario);

    /// helper function to simplify needed calls when hexmap exits world
    void exit_world();

protected:
    /// hexagonal space to use for converting cell ids to points
    HexMapSpace space;

private:
    /// multimesh & multimesh instance pair
    struct MultiMesh {
        RID multimesh;
        RID instance;
    };

    /// map of cell keys to the visual for each cell
    HashMap<CellKey, Cell> cell_map;

    /// list of multimeshes
    Vector<MultiMesh> multimeshes;

    /// scenario for mesh instances
    RID scenario;

    /// godot object id all mesh instances belong to
    ObjectID object_id;

    void free_multimeshes();
    void build_multimeshes();
};
