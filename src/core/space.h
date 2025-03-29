#pragma once

#include "cell_id.h"
#include "godot_cpp/classes/array_mesh.hpp"
#include "godot_cpp/variant/packed_int32_array.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

/// Helper class for mapping `HexMapCellId` to `Vector3` coordinates
class HexMapSpace {
    /// cell scale; scale from radius 1, height 1 cell size
    Vector3 cell_scale = Vector3(1, 1, 1);

    /// cell mesh origin offset
    Vector3 mesh_offset;

public:
    // PERMANENT NOTE TO SELF:
    // Do not try to move the hex cell vertices into a static const in this
    // class.  `Vector3` cannot be in a global variable because they are
    // initialized during dll load, but the Vector3 initialization depends on
    // the godot engine for initialization.  The engine pointer isn't set until
    // after dll load has completed.
    //
    // DO NOT DELETE THIS NOTE.  I've tried doing this three times already and
    // keep forgetting why.

    /// global transform of the meshes
    Transform3D transform;

    /// Set the cell scaling for the hex grid.  Scale modifies the unit hex
    /// cell with `height = 1`, `radius = 1`.  This will update any
    /// `mesh_offset` that was previously set.
    void set_cell_scale(Vector3);

    /// Get the cell scale
    inline Vector3 get_cell_scale() const { return cell_scale; }

    void set_cell_height(real_t);
    inline real_t get_cell_height() const { return cell_scale.y; }

    void set_cell_radius(real_t);
    inline real_t get_cell_radius() const { return cell_scale.x; }

    /// Set the cell mesh origin offset
    ///
    /// This value is in `cell_scale` units.  To move the mesh origin to the
    /// bottom of the cell, regardless of cell height, you would use the value
    /// `Vector3(0, -0.5, 0)`
    void set_mesh_offset(Vector3);

    /// Get the cell mesh origin offset
    inline Vector3 get_mesh_offset() const { return mesh_offset; }

    /// Set the global transform of the meshes
    void set_transform(Transform3D);

    /// Get the global transform of the `HexSpace`
    inline Transform3D get_transform() const { return transform; }

    /// Get the mesh origin point for the given `cell` in global space
    inline Vector3 get_mesh_origin(const HexMapCellId &cell) const {
        return transform.xform(
                cell.unit_center() * cell_scale + mesh_offset * cell_scale);
    }

    /// Get the global transform for mesh origin point for the given `cell`
    inline Transform3D get_mesh_transform(const HexMapCellId &cell) const {
        Transform3D mesh(Basis(),
                cell.unit_center() * cell_scale + mesh_offset * cell_scale);
        return transform * mesh;
    }

    /// Get the center point for the given `cell` in local space
    inline Vector3 get_cell_center(const HexMapCellId &cell) const {
        return cell.unit_center() * cell_scale;
    }

    /// Get the center point for the given `cell` in local space
    inline Vector3 get_cell_center_global(const HexMapCellId &cell) const {
        return transform.xform(cell.unit_center() * cell_scale);
    }

    /// Get the `HexMapCellId` for a point in local space
    inline HexMapCellId get_cell_id(const Vector3 &local_pos) const {
        return HexMapCellId::from_unit_point(local_pos / cell_scale);
    }

    /// Get the `HexMapCellId` for a point in global space
    inline HexMapCellId get_cell_id_global(const Vector3 &global_pos) const {
        Vector3 local = transform.inverse().xform(global_pos);
        return get_cell_id(local);
    }

    inline bool operator!=(const HexMapSpace &other) const {
        return cell_scale != other.cell_scale ||
                mesh_offset != other.mesh_offset ||
                transform != other.transform;
    }

    /// generate a PackedVector3Array for the vertices of a hex-shaped cell
    PackedVector3Array get_cell_vertices(
            Vector3 scale = Vector3(1, 1, 1)) const;

    /// generate an ArrayMesh that encompases the entire cell
    ///
    /// surface 0 contains the triangles, surface 1 contains the lines
    Ref<ArrayMesh> build_cell_mesh() const;

    /// return the `HexMapCellId` of every cell within a quad in local space
    ///
    /// @param a quad vertex
    /// @param b quad vertex
    /// @param c quad vertex
    /// @param d quad vertex
    /// @param padding percent to reduce the vertices of each cell to make it
    /// easier to select a cell withoug selecting the neighboring cell
    Vector<HexMapCellId> get_cell_ids_in_local_quad(Vector3 a,
            Vector3 b,
            Vector3 c,
            Vector3 d,
            float padding = 0.5) const;
};
