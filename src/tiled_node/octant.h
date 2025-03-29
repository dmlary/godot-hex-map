#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/rid.hpp>

#include "core/cell_id.h"
#include "core/library_mesh_tool.h"
#include "core/tile_orientation.h"

using namespace godot;

class HexMapTiledNode;

class HexMapOctant {
private:
    using CellKey = HexMapCellId::Key;

    // one per tile type used in octant
    struct MultiMesh {
        RID multimesh; // need MeshLibrary
        RID multimesh_instance; // need scene scenario
    };

    HexMapTiledNode &hex_map;
    HashSet<CellKey> cells;

    HexMapLibraryMeshTool mesh_tool;

    // The baked mesh and the multimeshes are mutually exclusive
    Ref<ArrayMesh> baked_mesh;
    RID baked_mesh_instance;

    RID physics_body;
    RID collision_debug_mesh;
    RID collision_debug_mesh_instance;

    bool dirty = false;

    // clear and rebuild the multimeshes
    void build_physics_body();
    void build_baked_mesh();

    void free_baked_mesh();

    // bake the mesh
    void bake_mesh();

public:
    /// Key type to use in HashMaps when referencing Octants
    union Key {
        struct {
            int16_t x, y, z;
        };
        uint64_t key = 0;

        Key() {};

        _FORCE_INLINE_ Key(const HexMapCellId &cell_id, int octant_size) {
            Vector3i oddr = cell_id.to_oddr() / octant_size;
            x = oddr.x;
            y = oddr.y;
            z = oddr.z;
        }
        _FORCE_INLINE_ Key(uint64_t key) : key(key) {};

        _FORCE_INLINE_ bool operator<(const Key &other) const {
            return key < other.key;
        }
        _FORCE_INLINE_ bool operator==(const Key &other) const {
            return key == other.key;
        }
        _FORCE_INLINE_ operator uint64_t() const { return key; }
    };

    HexMapOctant(HexMapTiledNode &hex_map);
    ~HexMapOctant();

    void enter_world();
    void exit_world();

    void update_collision_properties();
    void update_physics_params();
    void update_visibility();

    void apply_changes();

    void set_cell(CellKey, int, HexMapTileOrientation);
    void clear_cell(CellKey);
    void set_cell_visibility(HexMapCellId, bool visible);
    void set_all_cells_visible();

    inline bool is_empty() const { return cells.is_empty(); };
    inline bool is_dirty() const { return dirty; };
    inline void set_dirty() { dirty = true; };

    // bake if needed and return the baked mesh
    void set_baked_mesh(Ref<Mesh> mesh);
    Ref<Mesh> get_baked_mesh();
    void clear_baked_mesh();
    RID get_baked_mesh_instance() const;
};
