#include <cassert>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/mesh_data_tool.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include "octant.h"
#include "profiling.h"
#include "tiled_node.h"

void HexMapOctant::free_baked_mesh() {
    baked_mesh = Ref<Mesh>();
    if (baked_mesh_instance.is_valid()) {
        RenderingServer::get_singleton()->free_rid(baked_mesh_instance);
        baked_mesh_instance = RID();
    }
}

void HexMapOctant::build_physics_body() {
    auto profiler = profiling_begin("Octant::build_physics_body()");

    assert(hex_map.is_inside_tree() &&
            "should be only be called when HexMap is in SceneTree");
    RenderingServer *rs = RenderingServer::get_singleton();
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    Ref<MeshLibrary> &mesh_library = hex_map.mesh_library;

    ps->body_clear_shapes(physics_body);

    if (collision_debug_mesh.is_valid()) {
        rs->mesh_clear(collision_debug_mesh);
    }

    // can't do anything without the MeshLibrary
    if (!mesh_library.is_valid()) {
        return;
    }

    // to update the collision debugging mesh, we need the vertices for all of
    // the collision shapes
    PackedVector3Array debug_mesh_vertices;

    // iterate through the cells, save off RID & Transform pairs for multimesh
    // creation later, update the physics body, and if collision debugging is
    // enabled, update that mesh.
    for (const CellKey &cell_key : cells) {
        const HexMapTiledNode::Cell *cell = hex_map.cell_map.getptr(cell_key);
        ERR_CONTINUE_MSG(cell == nullptr, "nonexistent HexMap cell in Octant");

        Ref<Mesh> mesh = mesh_library->get_item_mesh(cell->item);
        if (!mesh.is_valid()) {
            continue;
        }
        RID mesh_rid = mesh->get_rid();

        Transform3D cell_transform(
                cell->get_basis(), hex_map.get_cell_center(cell_key));

        // Update the static body for the octant if the mesh library has any
        // collision shapes for this cell type.  Note that the collision shape
        // has its own transform independent of the mesh transform.  Also
        // get_item_shapes() returns an array of Shape3D followed by
        // Transform3D for each shape.
        const Array shapes = mesh_library->get_item_shapes(cell->item);
        for (int i = 0; i < shapes.size(); i += 2) {
            Shape3D *shape = Object::cast_to<Shape3D>(shapes[i]);
            Transform3D shape_transform =
                    cell_transform * (Transform3D)shapes[i + 1];

            // add the shape to the physics body
            ps->body_add_shape(
                    physics_body, shape->get_rid(), shape_transform);

            // if we have a collision debugging mesh, add the shape vertices to
            // the vertex array.
            if (collision_debug_mesh.is_valid()) {
                const Ref<ArrayMesh> &debug_mesh = shape->get_debug_mesh();
                const Array arrays = debug_mesh->surface_get_arrays(0);
                assert(arrays.size() > Mesh::ARRAY_VERTEX &&
                        "arrays should include vertex arrays");
                const PackedVector3Array vertices = arrays[Mesh::ARRAY_VERTEX];
                for (const Vector3 &v : vertices) {
                    debug_mesh_vertices.append(shape_transform.xform(v));
                }
            }
        }
    }

    Transform3D global_transform = hex_map.get_global_transform();
    ps->body_set_state(physics_body,
            PhysicsServer3D::BODY_STATE_TRANSFORM,
            global_transform);

    // update the collision debugging mesh if one exists
    if (collision_debug_mesh.is_valid() && !debug_mesh_vertices.is_empty()) {
        Array surface_arrays;
        surface_arrays.resize(RS::ARRAY_MAX);
        surface_arrays[RS::ARRAY_VERTEX] = debug_mesh_vertices;
        rs->mesh_add_surface_from_arrays(
                collision_debug_mesh, RS::PRIMITIVE_LINES, surface_arrays);
        rs->mesh_surface_set_material(collision_debug_mesh,
                0,
                hex_map.collision_debug_mat->get_rid());
        rs->instance_set_transform(
                collision_debug_mesh_instance, global_transform);
    }
}

void HexMapOctant::bake_mesh() {
    auto profiler = profiling_begin("Octant::bake_mesh()");

    Ref<MeshLibrary> &mesh_library = hex_map.mesh_library;
    HashMap<Ref<Material>, Ref<SurfaceTool>> mat_map;

    // hide the mesh_manager; we want to preserve its state
    mesh_tool.set_visibility(false);

    // iterate through the cells, add cell mesh surfaces to the surface tool
    // for each material.
    for (const CellKey &cell_key : cells) {
        const HexMapTiledNode::Cell *cell = hex_map.cell_map.getptr(cell_key);
        ERR_CONTINUE_MSG(cell == nullptr, "nonexistent HexMap cell in Octant");

        Ref<ArrayMesh> mesh = mesh_library->get_item_mesh(cell->item);
        if (!mesh.is_valid()) {
            continue;
        }
        RID mesh_rid = mesh->get_rid();

        Transform3D transform;
        transform.basis = cell->get_basis();
        transform.set_origin(hex_map.get_cell_center(cell_key));
        transform *= mesh_library->get_item_mesh_transform(cell->item);

        for (int i = 0; i < mesh->get_surface_count(); i++) {
            if (mesh->surface_get_primitive_type(i) !=
                    Mesh::PRIMITIVE_TRIANGLES) {
                continue;
            }
            Ref<Material> material = mesh->surface_get_material(i);
            Ref<SurfaceTool> *surface_tool = mat_map.getptr(material);
            if (surface_tool == nullptr) {
                auto iter = mat_map.insert(material, Ref<SurfaceTool>());
                surface_tool = &iter->value;
                surface_tool->instantiate();
                surface_tool->ptr()->begin(Mesh::PRIMITIVE_TRIANGLES);
                surface_tool->ptr()->set_material(material);
            }
            surface_tool->ptr()->append_from(mesh, i, transform);
        }
    }

    baked_mesh.instantiate();
    for (const auto &it : mat_map) {
        it.value->commit(baked_mesh);
    }

    RenderingServer *rs = RenderingServer::get_singleton();
    RID scenario = hex_map.get_world_3d()->get_scenario();
    baked_mesh_instance =
            rs->instance_create2(baked_mesh->get_rid(), scenario);
    rs->instance_attach_object_instance_id(
            baked_mesh_instance, hex_map.get_instance_id());
    rs->instance_set_transform(
            baked_mesh_instance, hex_map.get_global_transform());

    // XXX texel size not easily modified for gridmap; do we need to expose
    // this value or fetch it from someplace?  Also not sure about global
    // transform here, but it matches GridMap

    baked_mesh->lightmap_unwrap(hex_map.get_global_transform(), 0.1);
}

void HexMapOctant::update_collision_properties() {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    ps->body_set_collision_layer(physics_body, hex_map.collision_layer);
    ps->body_set_collision_mask(physics_body, hex_map.collision_mask);
    ps->body_set_collision_layer(physics_body, hex_map.collision_layer);
}

void HexMapOctant::update_physics_params() {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    ps->body_set_param(physics_body,
            PhysicsServer3D::BODY_PARAM_FRICTION,
            hex_map.physics_body_friction);
    ps->body_set_param(physics_body,
            PhysicsServer3D::BODY_PARAM_BOUNCE,
            hex_map.physics_body_bounce);
}

void HexMapOctant::update_visibility() {
    ERR_FAIL_COND(!hex_map.is_inside_tree());
    RenderingServer *rs = RenderingServer::get_singleton();

    // XXX should hidden also disable the static shape?

    bool visible = hex_map.is_visible_in_tree();
    mesh_tool.set_visibility(visible);
    if (collision_debug_mesh_instance.is_valid()) {
        rs->instance_set_visible(collision_debug_mesh_instance, visible);
    }
    if (baked_mesh_instance.is_valid()) {
        rs->instance_set_visible(baked_mesh_instance, visible);
    }
}

void HexMapOctant::enter_world() {
    ERR_FAIL_COND(!hex_map.is_inside_tree());

    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    RenderingServer *rs = RenderingServer::get_singleton();
    SceneTree *st = hex_map.get_tree();

    mesh_tool.set_scenario(hex_map.get_world_3d()->get_scenario());

    ps->body_set_space(physics_body, hex_map.get_world_3d()->get_space());

    // if debugging collisions, create collision debug mesh and show it
    if (hex_map.collision_debug || st->is_debugging_collisions_hint()) {
        assert(!collision_debug_mesh.is_valid() && "mesh should not exist");
        assert(!collision_debug_mesh_instance.is_valid() &&
                "mesh instance should not exist");

        collision_debug_mesh = rs->mesh_create();
        collision_debug_mesh_instance = rs->instance_create2(
                collision_debug_mesh, hex_map.get_world_3d()->get_scenario());
    }

    apply_changes();
}

void HexMapOctant::exit_world() {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    RenderingServer *rs = RenderingServer::get_singleton();

    ps->body_set_space(physics_body, RID());

    mesh_tool.exit_world();

    if (baked_mesh_instance.is_valid()) {
        rs->free_rid(baked_mesh_instance);
        baked_mesh_instance = RID();
    }

    if (collision_debug_mesh_instance.is_valid()) {
        rs->free_rid(collision_debug_mesh_instance);
        collision_debug_mesh_instance = RID();
    }

    if (collision_debug_mesh.is_valid()) {
        rs->free_rid(collision_debug_mesh);
        collision_debug_mesh = RID();
    }
}

void HexMapOctant::apply_changes() {
    if (!hex_map.is_inside_tree()) {
        return;
    }

    build_physics_body();

    if (baked_mesh_instance.is_valid()) {
        RenderingServer *rs = RenderingServer::get_singleton();
        rs->instance_set_transform(
                baked_mesh_instance, hex_map.get_global_transform());
        return;
    }
    if (baked_mesh.is_valid()) {
        RenderingServer *rs = RenderingServer::get_singleton();
        baked_mesh_instance = rs->instance_create2(
                baked_mesh->get_rid(), hex_map.get_world_3d()->get_scenario());
        rs->instance_attach_object_instance_id(
                baked_mesh_instance, hex_map.get_instance_id());
        rs->instance_set_transform(
                baked_mesh_instance, hex_map.get_global_transform());
        return;
    }
    dirty = false;

    mesh_tool.set_space(hex_map.get_space());
    Ref<MeshLibrary> mesh_library = hex_map.get_mesh_library();
    mesh_tool.set_mesh_library(mesh_library);
    mesh_tool.refresh();
}

void HexMapOctant::set_cell(const CellKey cell_key,
        int index,
        HexMapTileOrientation orientation) {
    free_baked_mesh();
    cells.insert(cell_key);
    mesh_tool.set_cell(cell_key, index, orientation);
    dirty = true;
}

void HexMapOctant::clear_cell(const CellKey cell_key) {
    free_baked_mesh();
    cells.erase(cell_key);
    mesh_tool.clear_cell(cell_key);
    dirty = true;
}

void HexMapOctant::set_cell_visibility(HexMapCellId cell_id, bool visible) {
    free_baked_mesh();
    mesh_tool.set_cell_visibility(cell_id, visible);
    dirty = true;
}

void HexMapOctant::set_all_cells_visible() {
    free_baked_mesh();
    mesh_tool.set_all_cells_visible();
    dirty = true;
}

void HexMapOctant::set_baked_mesh(Ref<Mesh> mesh) { baked_mesh = mesh; }

Ref<Mesh> HexMapOctant::get_baked_mesh() {
    if (!baked_mesh.is_valid()) {
        bake_mesh();
    }
    return baked_mesh;
}

RID HexMapOctant::get_baked_mesh_instance() const {
    ERR_FAIL_COND_V_MSG(!baked_mesh_instance.is_valid(),
            RID(),
            "HexMapOctant: no baked mesh instance");
    return baked_mesh_instance;
}

void HexMapOctant::clear_baked_mesh() {
    if (!baked_mesh.is_valid()) {
        return;
    }
    baked_mesh = Ref<Mesh>();
    RenderingServer::get_singleton()->free_rid(baked_mesh_instance);
    baked_mesh_instance = RID();

    if (hex_map.is_inside_tree()) {
        build_physics_body();
    }
}

HexMapOctant::HexMapOctant(HexMapTiledNode &hex_map) : hex_map(hex_map) {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

    mesh_tool.set_object_id(hex_map.get_instance_id());

    physics_body = ps->body_create();
    ps->body_set_mode(physics_body, PhysicsServer3D::BODY_MODE_STATIC);
    ps->body_attach_object_instance_id(
            physics_body, hex_map.get_instance_id());
    update_collision_properties();
    update_physics_params();
}

HexMapOctant::~HexMapOctant() {
    PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
    ps->free_rid(physics_body);
}
