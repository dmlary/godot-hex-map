#pragma once

#include "godot_cpp/classes/mesh.hpp"
#include "godot_cpp/templates/hash_set.hpp"
#include "godot_cpp/templates/vector.hpp"
#include "godot_cpp/variant/rid.hpp"
#include "hex_map_cell_id.h"

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

	// XXX due to the amount of data needed from the HexMap instance, it looks
	// like we'll need to just keep a pointer to it here, and probably should
	// mark this class as friend.
	HexMap &hex_map;
	HashSet<CellKey> cells;
	Vector<struct MultiMesh> multimeshes;

	RID physics_body; // need instance_id, space
	RID collision_debug_mesh; // need to know if debug is enabled
	RID collision_debug_mesh_instance; // need scene scenario

	Ref<Mesh> baked_mesh;
	RID baked_mesh_instance; // need scene scenario

	bool dirty = false;

	// clear and rebuild the collision debug mesh
	void build_collision_debug_mesh();

	// clear and rebuild the multimeshes
	void build_meshes();

public:
	// octant key used in HexMap
	union Key {
		struct {
			int16_t x, y, z;
		};
		uint64_t key = 0;

		Key(){};

		_FORCE_INLINE_ Key(const HexMapCellId &cell_id, int octant_size) {
			Vector3i oddr = cell_id.to_oddr() / octant_size;
			x = oddr.x;
			y = oddr.y;
			z = oddr.z;
		}

		_FORCE_INLINE_ bool operator<(const Key &other) const {
			return key < other.key;
		}
		_FORCE_INLINE_ bool operator==(const Key &other) const {
			return key == other.key;
		}
		_FORCE_INLINE_ operator uint64_t() const { return key; }
	};

	//
	// create the static body
	// [punted] set collision properties & physics material
	//
	// need hexmap instance id for static body
	HexMapOctant(HexMap &hex_map);
	~HexMapOctant();

	// add meshes to scene, add static body to space
	// can only check if debugging collisions here
	//
	// need hexmap world3d for scene scenario & space
	void enter_world();
	// clear scenario for meshes, switch body to invalid space
	void exit_world();

	void update_transforms();
	void update_collision_properties();
	void update_physics_params();

	// clear baked
	// rebuild static body
	// rebuild meshes; multimesh will free when we call multimesh_allocate_data
	// re-enter world? if we don't exit the world, we won't need HexMap to
	// enter
	// mark clean
	//
	// need MeshLibrary to rebuild multimesh, and multimesh is per mesh
	void update();

	// add cell to cells
	// mark dirty
	void add_cell(const CellKey);

	// remove cell, mark dirty, return bool if empty
	void remove_cell(const CellKey);

	inline bool is_empty() const { return cells.is_empty(); };
	inline bool is_dirty() const { return dirty; };

	// bake the mesh
	void bake_mesh();
	// don't allow if dirty
	Ref<Mesh> get_baked_mesh() const;
	void set_baked_mesh(Ref<Mesh> mesh);
};
