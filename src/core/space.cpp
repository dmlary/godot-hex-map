
#include "space.h"
#include "cell_id.h"
#include "iter_cube.h"
#include "math.h"
#include "profiling.h"

#include <godot_cpp/classes/geometry2d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

void HexMapSpace::set_transform(Transform3D value) { transform = value; }

void HexMapSpace::set_cell_scale(Vector3 value) { cell_scale = value; }
void HexMapSpace::set_cell_height(real_t value) { cell_scale.y = value; }
void HexMapSpace::set_cell_radius(real_t value) {
    cell_scale.x = cell_scale.z = value;
}

void HexMapSpace::set_mesh_offset(Vector3 value) { mesh_offset = value; }

PackedVector3Array HexMapSpace::get_cell_vertices(Vector3 scale) const {
    // clang-format off
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
    return PackedVector3Array({
        // top face
        Vector3(0.0, 0.5, -1.0) * scale,            // 0
        Vector3(-Math_SQRT3_2, 0.5, -0.5) * scale,  // 1
        Vector3(-Math_SQRT3_2, 0.5, 0.5) * scale,   // 2
        Vector3(0.0, 0.5, 1.0) * scale,             // 3
        Vector3(Math_SQRT3_2, 0.5, 0.5) * scale,    // 4
        Vector3(Math_SQRT3_2, 0.5, -0.5) * scale,   // 5
        // bottom face
        Vector3(0.0, -0.5, -1.0) * scale,           // 6
        Vector3(-Math_SQRT3_2, -0.5, -0.5) * scale, // 7
        Vector3(-Math_SQRT3_2, -0.5, 0.5) * scale,  // 8
        Vector3(0.0, -0.5, 1.0) * scale,            // 9
        Vector3(Math_SQRT3_2, -0.5, 0.5) * scale,   // 10 (0xa)
        Vector3(Math_SQRT3_2, -0.5, -0.5) * scale,  // 11 (0xb)
    });
    // clang-format on
}

Ref<ArrayMesh> HexMapSpace::build_cell_mesh() const {
    Array triangles, lines;
    triangles.resize(RenderingServer::ARRAY_MAX);
    lines.resize(RenderingServer::ARRAY_MAX);

    PackedVector3Array v = get_cell_vertices(cell_scale);
    // Use the points to construct each individual cell face.  This requires
    // us to duplicate vertices so corners can have independent UVs to
    // do flat-shading.
    // clang-format off
    triangles[RenderingServer::ARRAY_VERTEX] = PackedVector3Array({
        v[0], v[1], v[2], v[3], v[4], v[5],     // top
        v[6], v[7], v[8], v[9], v[10], v[11],   // bottom
        v[4], v[5], v[11], v[10],               // east         (12..15)
        v[5], v[0], v[6], v[11],                // northeast    (16..19)
        v[0], v[1], v[7], v[6],                 // northwest    (20..23)
        v[1], v[2], v[8], v[7],                 // west         (24..27)
        v[2], v[3], v[9], v[8],                 // southwest    (28..31)
        v[3], v[4], v[10], v[9],                // southwest    (32..35)
    });
    triangles[RenderingServer::ARRAY_NORMAL] = PackedVector3Array({
        // top
        Vector3(0, 1, 0),
        Vector3(0, 1, 0),
        Vector3(0, 1, 0),
        Vector3(0, 1, 0),
        Vector3(0, 1, 0),
        Vector3(0, 1, 0),
        // bottom
        Vector3(0, -1, 0),
        Vector3(0, -1, 0),
        Vector3(0, -1, 0),
        Vector3(0, -1, 0),
        Vector3(0, -1, 0),
        Vector3(0, -1, 0),
        // east
        Vector3(1, 0, 0),
        Vector3(1, 0, 0),
        Vector3(1, 0, 0),
        Vector3(1, 0, 0),
        // northeast
        Vector3(0.5, 0, -Math_SQRT3_2),
        Vector3(0.5, 0, -Math_SQRT3_2),
        Vector3(0.5, 0, -Math_SQRT3_2),
        Vector3(0.5, 0, -Math_SQRT3_2),
        // northwest
        Vector3(-0.5, 0, -Math_SQRT3_2),
        Vector3(-0.5, 0, -Math_SQRT3_2),
        Vector3(-0.5, 0, -Math_SQRT3_2),
        Vector3(-0.5, 0, -Math_SQRT3_2),
        // west
        Vector3(-1, 0, 0),
        Vector3(-1, 0, 0),
        Vector3(-1, 0, 0),
        Vector3(-1, 0, 0),
        // southwest
        Vector3(-0.5, 0, Math_SQRT3_2),
        Vector3(-0.5, 0, Math_SQRT3_2),
        Vector3(-0.5, 0, Math_SQRT3_2),
        Vector3(-0.5, 0, Math_SQRT3_2),
        // southeast
        Vector3(0.5, 0, Math_SQRT3_2),
        Vector3(0.5, 0, Math_SQRT3_2),
        Vector3(0.5, 0, Math_SQRT3_2),
        Vector3(0.5, 0, Math_SQRT3_2),
    });
    triangles[RenderingServer::ARRAY_INDEX] =  PackedInt32Array({
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
        12 + 0, 12 + 1, 12 + 2,
        12 + 2, 12 + 3, 12 + 0,
	// northeast
        16 + 0, 16 + 1, 16 + 2,
        16 + 2, 16 + 3, 16 + 0,
	// northwest
        20 + 0, 20 + 1, 20 + 2,
        20 + 2, 20 + 3, 20 + 0,
	// west
        24 + 0, 24 + 1, 24 + 2,
        24 + 2, 24 + 3, 24 + 0,
	// southwest
        28 + 0, 28 + 1, 28 + 2,
        28 + 2, 28 + 3, 28 + 0,
	// southeast
        32 + 0, 32 + 1, 32 + 2,
        32 + 2, 32 + 3, 32 + 0,
    });

    // Also the lines surface
    lines[RenderingServer::ARRAY_VERTEX] = v;
    lines[RenderingServer::ARRAY_INDEX] = PackedInt32Array({
        5, 4, 3, 2, 1, 0, 5,    // top
        11, 10, 9, 8, 7, 6, 11, // bottom
        11, 5, 4, 10,           // east
        9, 3, 2, 8,             // southwest
        7, 1, 0, 6,             // northwest
    });
    // clang-format on

    Ref<ArrayMesh> mesh;
    mesh.instantiate();
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, triangles);
    mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINE_STRIP, lines);

    return mesh;
}

Vector<HexMapCellId> HexMapSpace::get_cell_ids_in_local_quad(Vector3 a,
        Vector3 b,
        Vector3 c,
        Vector3 d,
        float padding) const {
    ERR_FAIL_COND_V_MSG(padding <= 0 && padding > 1.0,
            Vector<HexMapCellId>(),
            "HexMapSpace::get_cell_ids_in_quad(): invalid padding value;"
            "must be `0 < padding <= 1`");

    // XXX build_cell_vertex_array() applies cell_scale; try this without
    // converting to unit space
    //
    // convert local space to unit space
    a /= cell_scale;
    b /= cell_scale;
    c /= cell_scale;
    d /= cell_scale;

    // Ensure all four points fall on a common plane.  The approach we use
    // below will not work unless all four points are on the same plane.
    Plane plane(a, b, c);
    ERR_FAIL_COND_V_MSG(!plane.has_point(d, 0.0001f),
            Vector<HexMapCellId>(),
            "HexMapSpace::get_cell_ids_in_quad(): quad points must all be on "
            "the same plane");

    // create a hex space cube iterator to enumerate every hex cell id within
    // the cube defined by the top-right, bottom-left points
    Vector3 top_right = a.max(b).max(c).max(d);
    Vector3 bottom_left = a.min(b).min(c).min(d);
    HexMapIterCube iter(top_right, bottom_left);

    // we're going to reduce the 3d problem to 2d by using the planes that
    // are most perpendicular to the plane normal.
    int axis[2];
    switch (plane.normal.abs().max_axis_index()) {
        case godot::Vector3::AXIS_X:
            axis[0] = Vector3::AXIS_Y;
            axis[1] = Vector3::AXIS_Z;
            break;
        case godot::Vector3::AXIS_Y:
            axis[0] = Vector3::AXIS_X;
            axis[1] = Vector3::AXIS_Z;
            break;
        case godot::Vector3::AXIS_Z:
            axis[0] = Vector3::AXIS_X;
            axis[1] = Vector3::AXIS_Y;
            break;
    }

    // break the 3D quad down into two 2D triangles
    Vector2 aa(a[axis[0]], a[axis[1]]);
    Vector2 ab(b[axis[0]], b[axis[1]]);
    Vector2 ac(c[axis[0]], c[axis[1]]);

    Vector2 ba(a[axis[0]], a[axis[1]]);
    Vector2 bb(d[axis[0]], d[axis[1]]);
    Vector2 bc(c[axis[0]], c[axis[1]]);

    // XXX pull these points in to make it easier to select SW/SE line
    float scale = 1.0 - padding;
    PackedVector3Array vertices =
            get_cell_vertices(Vector3(scale, scale, scale));

    Geometry2D *geo = Geometry2D::get_singleton();
    Vector<HexMapCellId> out;
    auto prof = profiling_begin("selecting cells");
    for (const HexMapCellId &cell_id : iter) {
        Vector3 center = cell_id.unit_center();
        Vector2 point(center[axis[0]], center[axis[1]]);
        bool intercept_quad =
                geo->point_is_inside_triangle(point, aa, ab, ac) ||
                geo->point_is_inside_triangle(point, ba, bb, bc);
        bool intercept_plane = false;
        bool above = plane.is_point_over(center);

        for (Vector3 vert : vertices) {
            vert += center;

            if (!intercept_plane && plane.is_point_over(vert) != above) {
                intercept_plane = true;
            }

            Vector2 point(vert[axis[0]], vert[axis[1]]);
            if (!intercept_quad &&
                    (geo->point_is_inside_triangle(point, aa, ab, ac) ||
                            geo->point_is_inside_triangle(
                                    point, ba, bb, bc))) {
                intercept_quad = true;
            }

            if (intercept_plane && intercept_quad) {
                out.push_back(cell_id);
                break;
            }
        }
    }

    return out;
}
