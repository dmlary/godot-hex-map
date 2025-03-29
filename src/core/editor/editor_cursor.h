#ifdef TOOLS_ENABLED
#pragma once

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include "core/cell_id.h"
#include "core/library_mesh_tool.h"
#include "core/tile_orientation.h"

using namespace godot;

class EditorCursor {
public:
    using CellId = HexMapCellId;
    using CellKey = HexMapCellId::Key;
    using TileOrientation = HexMapTileOrientation;

    enum EditAxis {
        AXIS_Y,
        AXIS_Q, // northwest/southeast
        AXIS_R, // east/west
        AXIS_S, // northeast/southeast
    };

    //
    struct CellState {
        CellId cell_id;
        int index = -1;
        TileOrientation orientation;
    };

    EditorCursor(RID scenario);
    ~EditorCursor();

    void set_space(HexMapSpace);
    void set_mesh_library(Ref<MeshLibrary> &);

    /// Set the callback function for `EditorCursor` to call when it needs to
    /// show/hide cells in the parent HexMap.  The arguments to this function
    /// will be an `Array` of `Vector3i` followed by `bool`.  The `Vector3i`
    /// can be converted to a `HexMapCellId`, while the `bool` is whether the
    /// cell should be visible.
    void set_cells_visibility_callback(Callable);

    void set_orientation(HexMapTileOrientation orientation);
    inline HexMapTileOrientation get_orientation() { return orientation; }

    void set_axis(EditAxis axis);
    inline EditAxis get_axis() { return edit_axis; };

    /// set the depth of the active edit plane
    void set_depth(int);

    /// update the cursor based on the camera and pointer positition
    ///
    /// Function will return `true` when the cursor moves to a different cell
    bool update(const Camera3D *camera, const Point2 &pointer, Vector3 *point);
    void update(bool force = false);

    /// set the visibility of the tiles
    ///
    /// this does not apply to the visibility of the hex grid
    void set_visibility(bool visible);

    /// set a cell; takes effect on next `update()` call
    void set_tile(CellId cell,
            int tile,
            TileOrientation orientation = TileOrientation::Upright0);

    /// clear all cells; takes effect on next `update()` call
    void clear_tiles();

    inline bool is_empty() { return mesh_tool.get_cells().is_empty(); };

    /// returns the number of cells the cursor occupies
    inline size_t size() { return mesh_tool.get_cells().size(); };

    /// get the cell map for the cursor, adjusted for cursor transforms
    Vector<CellState> get_cells() const;

    /// get the cell map for the cursor, in Array form described by
    /// `HexMapNode._set_cells()`
    Array get_cells_v() const;

    /// get the list of cell ids occupied by the cursor
    Array get_cell_ids_v() const;

    /// Get the state of the only cell in the cursor.
    ///
    /// This function will assert if the number of cells set in the cursor does
    /// not equal 1.
    CellState get_only_cell_state() const;

    /// get the cell the cursor is occupying
    inline HexMapCellId get_cell() { return pointer_cell; };

    /// get the coordinates of the cursor on the edit plane
    inline Vector3 get_pos() { return pointer_pos; };

private:
    /// Mesh tool for drawing meshes
    HexMapLibraryMeshTool mesh_tool;

    /// axis the cursor is editing
    EditAxis edit_axis = EditAxis::AXIS_Y;

    /// orientation of the cursor
    TileOrientation orientation;

    /// callback used for toggling cell visibility in the parent class to
    /// prevent existing cells from hiding cursor cells
    Callable set_cells_visibility;

    /// Copy of parent HexMap's HexSpace
    ///
    /// We keep this independent of the space in `mesh_tool` because that space
    /// includes our cursor rotation & translation.
    HexMapSpace parent_space;

    HexMapMeshTool mesh_manager;
    Vector3 pointer_pos;
    CellId pointer_cell;

    Vector3 last_update_origin;
    Vector3 last_update_normal;

    Plane edit_plane;

    Ref<StandardMaterial3D> grid_mat;
    RID grid_mesh;
    RID grid_mesh_instance;
    Transform3D grid_mesh_transform;

    /// HashSet of the cells we're currently hiding; used to limit the
    /// callbacks we make
    HashSet<CellKey> hidden_cells;

    /// Update the grid & tile meshes, along with `cursor_transform()`
    void transform_meshes();

    void build_y_grid();
    void build_r_grid();
};

VARIANT_ENUM_CAST(EditorCursor::EditAxis);

#endif
