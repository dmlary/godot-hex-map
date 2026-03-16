#ifdef TOOLS_ENABLED
#pragma once

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/templates/list.hpp>
#include <godot_cpp/variant/plane.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector2.hpp>

#include "core/cell_id.h"
#include "core/hex_map_node.h"
#include "core/library_mesh_tool.h"
#include "core/tile_orientation.h"

using namespace godot;

class EditorCursor {
public:
    enum EditAxis {
        AXIS_Y,
        AXIS_Q, // northwest/southeast
        AXIS_R, // east/west
        AXIS_S, // northeast/southeast
    };

    EditorCursor(RID scenario);
    ~EditorCursor();

    void set_space(HexMapSpace);
    void set_mesh_library(Ref<MeshLibrary>);
    void set_mesh_origin(Vector3);

    /// Set the callback function for `EditorCursor` to call when it needs to
    /// show/hide cells in the parent HexMap.  The arguments to this function
    /// will be an `Array` of `Vector3i` followed by `bool`.  The `Vector3i`
    /// can be converted to a `HexMapCellId`, while the `bool` is whether the
    /// cell should be visible.
    void set_cells_visibility_callback(Callable);

    void set_orientation(HexMapTileOrientation orientation);
    HexMapTileOrientation get_orientation() const;

    void set_axis(EditAxis axis);
    EditAxis get_axis() const;

    /// set the depth of the active edit plane
    void set_depth(int);

    /// update the cursor based on camera and pointer position
    /// querying the hex_map
    ///
    /// Function will return `true` when the cursor moves to a different cell
    bool update(const Camera3D *camera,
            const Point2 &pointer,
            const HexMapNode *hex_map,
            Vector3 *point);

    /// update the cursor based on the camera and pointer positition
    ///
    /// Function will return `true` when the cursor moves to a different cell
    bool update(const Camera3D *camera, const Point2 &pointer, Vector3 *point);

    /// update the cursor using the previous camera & pointer info
    ///
    /// This called when the edit plane or cursor is changed without camera or
    /// pointer info.  It updates the display appropriately for the new state
    /// as if the mouse cursor did not move.
    void update(bool force = false);

    /// set the visibility of the tiles
    ///
    /// this does not apply to the visibility of the hex grid
    void set_visible(bool visible);

    /// check if the cursor is visible
    bool get_visible() const;

    /// set a cell; takes effect on next `update()` call
    void set_tile(HexMapCellId cell,
            int tile,
            HexMapTileOrientation orientation =
                    HexMapTileOrientation::Upright0);

    /// clear all cells; takes effect on next `update()` call
    void clear_tiles();

    /// check if the cursor has no cells set
    bool is_empty() const;

    /// returns the number of cells the cursor occupies
    size_t size() const;

    /// get the cell map for the cursor, in Array form described by
    /// `HexMapNode._set_cells()`
    Array get_cells_v() const;

    /// get the list of cell ids occupied by the cursor
    Array get_cell_ids_v() const;

    /// Get the state of the only cell in the cursor.
    ///
    /// This function will assert if the number of cells set in the cursor does
    /// not equal 1.
    HexMapNode::CellInfo get_origin_cell_info() const;

    /// get the cell the cursor is occupying
    HexMapCellId get_cell() const;

    /// get the coordinates of the cursor on the edit plane
    Vector3 get_pos() const;

private:
    /// Mesh tool for drawing meshes
    HexMapLibraryMeshTool mesh_tool;

    /// axis the cursor is editing
    EditAxis edit_axis = EditAxis::AXIS_Y;

    /// orientation of the cursor
    HexMapTileOrientation orientation;

    /// callback used for toggling cell visibility in the parent class to
    /// prevent existing cells from hiding cursor cells
    Callable set_cells_visibility;

    /// Copy of parent HexMap's HexSpace
    ///
    /// We keep this independent of the space in `mesh_tool` because that space
    /// includes our cursor rotation & translation.
    HexMapSpace parent_space;

    /// the 3D position of the pointer on the edit plane
    Vector3 pointer_pos;

    /// the cell id that the pointer is hovering over
    HexMapCellId pointer_cell;

    /// camera raycast origin from the last call to update with a camera
    /// instance
    Vector3 last_update_origin;
    /// camera raycast normal from the last call to update with a camera
    /// instance
    Vector3 last_update_normal;

    /// description of the edit plane
    Plane edit_plane;

    /// material used in the edit grid
    Ref<StandardMaterial3D> grid_mat;

    /// mesh for the edit grid
    RID grid_mesh;

    /// mesh instance of the edit grid
    RID grid_mesh_instance;

    /// transform for the edit grid
    Transform3D grid_mesh_transform;

    /// HashSet of the cells we're currently hiding; used to limit the
    /// callbacks we make
    HashSet<HexMapCellId::Key> hidden_cells;

    /// Update the grid & tile meshes, along with `cursor_transform()`
    void transform_meshes();

    /// build a hexagonal edit grid along x/z plane
    void build_y_grid();

    /// build a rectangular edit grid along the y axis
    void build_r_grid();
};

VARIANT_ENUM_CAST(EditorCursor::EditAxis);

#endif
