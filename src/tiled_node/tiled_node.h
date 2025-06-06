#pragma once

#include <cstdint>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/navigation_mesh.hpp>
#include <godot_cpp/classes/navigation_mesh_source_geometry_data3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/physics_material.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3i.hpp>

#include "core/cell_id.h"
#include "core/hex_map_node.h"
#include "core/planes.h"
#include "core/tile_orientation.h"
#include "octant.h"

using namespace godot;

class HexMapTiledNode : public HexMapNode {
    using HexMapTiled = HexMapTiledNode;
    GDCLASS(HexMapTiled, HexMapNode);

    using CellKey = HexMapCellId::Key;
    using Octant = HexMapOctant;
    friend HexMapOctant;
    using OctantKey = Octant::Key;

public:
    using CellId = HexMapCellId;
    using Planes = HexMapPlanes;
    using TileOrientation = HexMapTileOrientation;

    // enum for setting mesh origin within the cell
    enum MeshOrigin {
        MESH_ORIGIN_CENTER,
        MESH_ORIGIN_TOP,
        MESH_ORIGIN_BOTTOM,
    };

    /// source geometry parser for all TiledNode instances; this is the
    /// id within the RID, or 0 when not set.
    // We cannot declare this as `static RID` because the godot dll provides
    // the implementation for RID, and it's not available when statics are
    // initialized.  I don't want to make these per-instance, because that's
    // just a waste.  I also cannot set this up during godot type registration
    // because the NavigationServer3D is not initialized at that time.
    //
    // So here we just pull the RID apart, and save the id; 0 means not set.
    //
    // This is released in uninitialize_hexmap_module()
    static uint64_t navigation_source_geometry_parser;

private:
    /**
     * @brief A Cell is a single cell in the cube map space; it is defined by
     * its coordinates and the populating Item, identified by int id.
     */
    union Cell {
        struct {
            unsigned int value : 16;
            unsigned int rot : 4;
            unsigned int visible : 1;
        };
        uint32_t cell = 0;

        Basis get_basis() const { return TileOrientation(rot); }
    };

    struct BakedMesh {
        Ref<Mesh> mesh;
        RID instance;
    };

    Ref<MeshLibrary> mesh_library;

    // map properties
    real_t cell_radius = 1.0;
    real_t cell_height = 1.0;
    MeshOrigin mesh_origin = MeshOrigin::MESH_ORIGIN_CENTER;

    // rendering properties
    int octant_size = 8;

    // physics body properties
    Ref<StandardMaterial3D> collision_debug_mat;
    uint32_t collision_layer = 1;
    uint32_t collision_mask = 1;
    real_t collision_priority = 1.0;
    bool collision_debug = false;
    Ref<PhysicsMaterial> physics_material;
    real_t physics_body_friction = 1.0;
    real_t physics_body_bounce = 0.0;

    bool navigation_bake_only_navmesh_tiles = false;

    HashMap<CellKey, Cell> cell_map;
    HashMap<OctantKey, Octant *> octants;

    // The LightmapGI node assumes we're tracking the lightmap meshes by index.
    // We use this Vector to map from the index they have to an OctantKey for
    // lookup in get_bake_mesh_instance().
    Vector<OctantKey> baked_mesh_octants;

    void recreate_octant_data();
    void update_octant_meshes();

    void update_physics_bodies_collision_properties();
    void update_physics_bodies_characteristics();

    bool awaiting_update = false;
    void update_dirty_octants();
    void update_dirty_octants_callback();

    void clear_internal();

protected:
    bool _set(const StringName &p_name, const Variant &p_value);
    bool _get(const StringName &p_name, Variant &r_ret) const;
    void _get_property_list(List<PropertyInfo> *p_list) const;

    void _notification(int p_what);
    void _update_visibility();
    static void _bind_methods();

public:
    void set_mesh_library(const Ref<MeshLibrary> &);
    Ref<MeshLibrary> get_mesh_library() const;
    bool on_mesh_library_changed();

    void set_mesh_origin(MeshOrigin);
    MeshOrigin get_mesh_origin() const;
    Vector3 get_mesh_origin_vec() const;

    bool on_hex_space_changed() override;

    void set_collision_debug(bool value);
    bool get_collision_debug() const;

    void set_collision_layer(uint32_t p_layer);
    uint32_t get_collision_layer() const;

    void set_collision_mask(uint32_t p_mask);
    uint32_t get_collision_mask() const;

    void set_collision_layer_value(int p_layer_number, bool p_value);
    bool get_collision_layer_value(int p_layer_number) const;

    void set_collision_mask_value(int p_layer_number, bool p_value);
    bool get_collision_mask_value(int p_layer_number) const;

    void set_collision_priority(real_t p_priority);
    real_t get_collision_priority() const;

    void set_physics_material(Ref<PhysicsMaterial> p_material);
    Ref<PhysicsMaterial> get_physics_material() const;

    void set_navigation_bake_only_navmesh_tiles(bool);
    bool get_navigation_bake_only_navmesh_tiles() const;

    void set_octant_size(int p_size);
    int get_octant_size() const;

    Transform3D get_cell_transform(const HexMapCellId &) const;

    void set_cell(const HexMapCellId &,
            int tile,
            HexMapTileOrientation orientation = 0) override;
    CellInfo get_cell(const HexMapCellId &) const override;
    bool has(HexMapCellId) const override;
    Array get_cell_vecs() const override;
    Array find_cell_vecs_by_value(int value) const override;

    // used by the editor to conceal cells for the editor cursor
    // value is not saved
    void set_cell_visibility(const HexMapCellId &cell_id,
            bool visibility) override;

    void clear_baked_meshes();
    void make_baked_meshes(bool p_gen_lightmap_uv = false,
            float p_lightmap_uv_texel_size = 0.1);
    Array get_bake_meshes();
    RID get_bake_mesh_instance(int p_idx);

    static bool generate_navigation_source_geometry(Ref<NavigationMesh>,
            Ref<NavigationMeshSourceGeometryData3D>,
            Node *);

    /// get the mesh origin in local coordinates for a given cell
    Vector3 get_cell_origin(const Ref<hex_bind::HexMapCellId>) const;

    void clear();

    HexMapTiledNode();
    ~HexMapTiledNode();
};

VARIANT_ENUM_CAST(HexMapTiledNode::MeshOrigin);
