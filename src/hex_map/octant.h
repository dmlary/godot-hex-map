#pragma once

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/rid.hpp>

#include "cell_id.h"

using namespace godot;

class HexMap;

class HexMapOctant {
private:
    using CellKey = HexMapCellId::Key;

    // one per tile type used in octant
    struct MultiMesh {
        RID multimesh; // need MeshLibrary
        RID multimesh_instance; // need scene scenario
    };

    HexMap &hex_map;
    HashSet<CellKey> cells;

    // The baked mesh and the multimeshes are mutually exclusive
    Ref<ArrayMesh> baked_mesh;
    RID baked_mesh_instance;
    Vector<struct MultiMesh> multimeshes;

    RID physics_body;
    RID collision_debug_mesh;
    RID collision_debug_mesh_instance;

    bool dirty = false;

    // clear and rebuild the multimeshes
    void build_physics_body();
    void build_multimesh();
    void build_baked_mesh();

    // apply the HexMap global transform
    void apply_global_transform();

    void free_multimeshes();
    void free_baked_mesh();

    // bake the mesh
    void bake_mesh();

public:
    // octant key used in HexMap
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

    HexMapOctant(HexMap &hex_map);
    ~HexMapOctant();

    void enter_world();
    void exit_world();

    void update_collision_properties();
    void update_physics_params();
    void update_transform();
    void update_visibility();

    void apply_changes();

    void add_cell(const CellKey);
    void remove_cell(const CellKey);

    inline bool is_empty() const { return cells.is_empty(); };
    inline bool is_dirty() const { return dirty; };
    inline void set_dirty() { dirty = true; };

    // bake if needed and return the baked mesh
    void set_baked_mesh(Ref<Mesh> mesh);
    Ref<Mesh> get_baked_mesh();
    void clear_baked_mesh();
    RID get_baked_mesh_instance() const;
};
