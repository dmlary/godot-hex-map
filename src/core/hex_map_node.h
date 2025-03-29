#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/navigation_mesh.hpp>
#include <godot_cpp/classes/navigation_mesh_source_geometry_data3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_material.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3i.hpp>

#include "cell_id.h"
#include "core/tile_orientation.h"
#include "space.h"

using namespace godot;

class HexMapOctant;

class HexMapNode : public Node3D {
    GDCLASS(HexMapNode, Node3D);

protected:
    HexMapSpace space;

    static void _bind_methods();

public:
    enum { INVALID_CELL_ITEM = -1 };

    /// common contents for any cell in a hex node
    struct CellInfo {
        int value = INVALID_CELL_ITEM;
        HexMapTileOrientation orientation;
    };

    // various cell size getters/setters
    void set_cell_height(real_t p_height);
    real_t get_cell_height() const;
    void set_cell_radius(real_t p_radius);
    real_t get_cell_radius() const;
    void set_center_y(bool value);
    bool get_center_y() const;

    /// set the HexMapSpace
    void set_space(const HexMapSpace &space);

    /// get the HexMapSpace
    inline const HexMapSpace &get_space() { return space; }

    /// Return the cell scale factor for cells in this hexmap.
    ///
    /// A scale factor of `Vector3(1,1,1)` is returned for `cell_radius = 1.0`,
    /// `cell_height = 1.0`.
    Vector3 get_cell_scale() const;

    /// called when the cell scale changes
    virtual bool cell_scale_changed();

    /// given a cell id, return the local position of the cell
    Vector3 get_cell_center(const HexMapCellId &) const;

    /// given a local position, return the cell id that contains that point
    HexMapCellId get_cell_id(Vector3) const;

    /// set a single cell
    virtual void set_cell(const HexMapCellId &,
            int tile,
            HexMapTileOrientation orientation = 0) = 0;

    /// set a single cell
    void set_cell(const Ref<hex_bind::HexMapCellId> cell_id,
            int p_item,
            int p_orientation = 0);

    /// get the info for a single cell
    virtual CellInfo get_cell(const HexMapCellId &) const = 0;

    /// set multiple cells
    ///
    /// `cells` is a flat Array containing three members for each cell:
    /// - `Vector3i` representing the cell id
    /// - int `value`
    /// - int `orientation`
    void set_cells(const Array p_cells);

    /// get tile & orientation for a subset of cells
    ///
    /// @param p_cells Array of Vector3i cell ids to look up
    /// @returns flat Array of [Vector3i, int, int] for each cell id requested.
    ///          The results will be in the same order as requested.
    Array get_cells(const Array p_cells);

    /// get the list of CellIds as Vector3i occupied in this node
    virtual Array get_cell_ids_v() const = 0;

    /// number of elements that are used to represent a cell in the Array
    /// returned by `_get_cells()`
    static const int CellArrayWidth = 3;

    /// set the visibility of a cell
    ///
    /// This is used by HexMapEditorPlugin to show/hide cells that overlap the
    /// editor cursor.  This information should not be saved to the node.
    virtual void set_cell_visibility(const HexMapCellId &,
            bool visibility) = 0;

    /// set the visibility of a number of cells
    ///
    /// @param cells flattened Array of [Vector3, bool] where bool is the
    /// visibility state of the cell with id Vector3.
    void set_cells_visibility(const Array cells);
};
