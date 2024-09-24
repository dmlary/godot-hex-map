/**************************************************************************/
/*  grid_map.h                                                            */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "godot_cpp/classes/physics_material.hpp"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/core/defs.hpp"
#include "godot_cpp/templates/hash_map.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include "hex_map/planes.h"
#include "hex_map/tile_orientation.h"
#include "hex_map_cell_id.h"
#include "octant.h"
#include <cstdint>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/templates/hash_set.hpp>
#include <godot_cpp/templates/vector.hpp>

// SQRT(3)/2; used both in the editor and the GridMap.  Due to the division, it
// didn't fit the pattern of other Math_SQRTN defines, so I'm putting it here.
#define SQRT3_2 0.8660254037844386
#define Math_SQRT3 1.7320508075688772935274463415059

// Rewrite notes
//
// IN PROGRESS:
// - IndexKey needs to be swapped to q/r/y; bad transpose elsewhere
// - switching HexMap GDScript functions to return HexMapCellId
// - figuring out how octant_update() works
// - collision shape debugging not working, but collision shapes work
//
// Types
// - Use HexMapCellId to replace IndexKey; need 32-bit hash, lt, eq operators
// - Cell union doesn't use layer bits
// - Octant
//		- leave NavigationCell stuff; don't know what it is for at the moment
//		- MultimeshInstance
//			- drop Vector<Item> items; not used
//		- Is
// - OctantKey could be uint32_t because the fields are never accessed after
//   creation
//
// * Octants look like individual octants can be marked dirty
//
// * octant creation
//		- memnew
//		- mark dirty
//		- create a static body for collision detection
//			- set static body friction from physics material
//			- set static body bounce from physics material
//		- set up collision debugging (if enabled)
//			- create mesh
//			- create mesh instance
//		- if HexMap is in the tree
//			- _octant_enter_world()
//			- _octant_transform()
//
// * _octant_enter_world()
//		- called from NOTIFICATION_ENTER_WORLD & set_cell_item()
//		- update collision body
//			- set transform to HexMap global transform
//			- set space to get_world_3d()->get_space()
//		- update collision debug mesh instance
//			- set scenario
//			- set transform to HexMap global transform
//		- update multimesh instances
//			- set scenario & global transform
//		- if bake_navigation is set && mesh_library is valid
//			- go through navigation_cell_ids; these were set up in
//			  octant_update
//				- do nothing unless there's a navmesh for the cell in meshlib
//				- create a new region
//				- set the region owner
//				- set region navigation layers
//				- set region navigation mesh
//				- set region transform
//				- set region map (possibly override with map_overrive)
//			- if bake_navigation (redundant)
//				- create navigation debug mesh & mesh instance if not present
//				- _update_octant_navigation_debug_edge_connections_mesh()
//
// * _octant_update(OctantKey)
//		- NOTE: even though we bake the mesh, we don't bake the collision
//		  data; so that must be recreated
//		- NOTE: even when octants are baked, collision shapes aren't.
//		- Ths function is only called by _update_octants_callback() in deferred
//		  call (debounced)
//		- return unless the key exists in the octant map
//		- return unless the octant is marked dirty <<---- IMPORTANT
//		- clear PhysicsServer3D shapes for the octant's static body
//		- free navigation regions & free debugging mesh instance
//		- free the multimesh & multimesh instances
//		- if no cells left in octant, _octant_clean_up() and return
//			- _octant_clean_up() has all the steps above
//			- return true to get caller to delete this octant from hash
//		- for each cell in the octant
//			- create a hash of each tile to transform & cell id
//				- cell id is only copied into unused
//				  Octant::MultimeshInstance::Item; can be dropped
//				- only do this if the mesh isn't baked
//				- QUESTION: octant ever dirty while baked?  No, never get into
//				  this code while baked; cleared before now
//			- add collision shapes for cell to octant's static body
//			- update collision shape debug vector array
//			- do cell navmesh stuff; not doing
//		- if not using baked meshes (which we'd never get here if we were)
//			- create multimesh & multimesh instances
//		- if there are collision debug vector array
//			- draw it
//		- clear dirty flag
//		- return false to say don't delete me
//
//	* _update_octant_navigation_debug_edge_connections_mesh(OctantKey)
//		- if debug not enabled, hide the mesh instance and return
//		- if not in tree, return
//		- if !bake_navigation, hide debug mesh instance, and return
//		- allocate mesh & mesh instance if needed
//		- clear mesh surfaces
//		- construct vertex array from region connections
//		- NOTE: gonna drop this functionality; let navmesh do all this for us.
//
//	* _recreate_octant_data()
//		- set flag denoting octants are being recreated
//		- copy cell_map
//		- clear internal state
//		- use set_cell_item() to rebuild octants
//
//	* _octant_clean_up()
//		- free collision debug mesh & mesh instance
//		- free collision static body
//		- free navigation regions & free debugging mesh instances
//		- if bake_navigation; free debug edge connection mesh & mesh instance
//		- erase multimeshes
//
// NavigationServer:
// * PROPOSAL: Drop meshlib navmesh support; only allow baking into another
//   navmesh
//
// * NavigationRegion3d parent to HexMap and BakeNavigationMesh
// * bake navigation mesh for each tile, export to tilemap, enable baked
//   navigation mesh
//
// * API for supporting bake mesh
//   https://github.com/godotengine/godot/pull/90876
//
// * NavMesh baked issues:
//   https://chat.godotengine.org/channel/devel?msg=khkdDMnDxeiteSmBh
//		* using baked navmeshes from tiles don't take into account current
//		  baking bound and border size
//
//
// Bake Mesh
// - make_baked_meshes()
//		- break cells into octants
//		- for each octant create a surface tool for each material used in the
//		  octant
//		- for each mesh surface in the cell mesh, append it to the surface tool
//		  for the specific material
//			- only for triangles primitive types
//
//		- for each octant hash key/value
//			- create a new mesh
//			- apply all SurfaceTools in octant to mesh
//			- create a mesh instance for the mesh
//			- add the mesh and mesh_instance to an array
//		- recreate_octant_data()
//
//	PROPOSAL:
//	* drop cell scaling
//		- meshes can be scaled before exporting to MeshLibrary, or cell radius
//		  can be changed

// heh heh, godotsphir!! this shares no code and the design is completely
// different with previous projects i've done.. should scale better with
// hardware that supports instancing

using namespace godot;

#define RS RenderingServer

class HexMap : public Node3D {
	GDCLASS(HexMap, Node3D);

	using CellKey = HexMapCellId::Key;
	using Octant = HexMapOctant;
	friend Octant;
	using OctantKey = Octant::Key;

public:
	using CellId = HexMapCellId;
	using Planes = HexMapPlanes;

private:
	enum {
		MAP_DIRTY_TRANSFORMS = 1,
		MAP_DIRTY_INSTANCES = 2,
	};

	// union IndexKey {
	// 	struct {
	// 		int16_t x;
	// 		int16_t y;
	// 		int16_t z;
	// 	};
	// 	uint64_t key = 0;
	//
	// 	static uint32_t hash(const IndexKey &p_key) {
	// 		return hash_one_uint64(p_key.key);
	// 	}
	// 	_FORCE_INLINE_ bool operator<(const IndexKey &p_key) const {
	// 		return key < p_key.key;
	// 	}
	// 	_FORCE_INLINE_ bool operator==(const IndexKey &p_key) const {
	// 		return key == p_key.key;
	// 	}
	//
	// 	_FORCE_INLINE_ operator Vector3i() const { return Vector3i(x, y, z); }
	//
	// 	IndexKey(Vector3i p_vector) {
	// 		x = (int16_t)p_vector.x;
	// 		y = (int16_t)p_vector.y;
	// 		z = (int16_t)p_vector.z;
	// 	}
	// 	IndexKey() {}
	// };

	/**
	 * @brief A Cell is a single cell in the cube map space; it is defined by
	 * its coordinates and the populating Item, identified by int id.
	 */
	union Cell {
		struct {
			unsigned int item : 16;
			unsigned int rot : 5;
			unsigned int layer : 8;
		};
		uint32_t cell = 0;

		Basis get_basis() { return TileOrientation(rot); }
	};

	/**
	 * @brief An Octant is a prism containing Cells;
	 */
	struct OctantOld {
		struct NavigationCell {
			RID region;
			Transform3D xform;
			RID navigation_mesh_debug_instance;
			uint32_t navigation_layers = 1;
		};

		struct MultimeshInstance {
			RID instance;
			RID multimesh;
			struct Item {
				int index = 0;
				Transform3D transform;
				CellKey key;
			};

			Vector<Item> items; // tools only, for changing visibility
		};

		Vector<MultimeshInstance> multimesh_instances;
		HashSet<CellKey> cells;
		RID collision_debug;
		RID collision_debug_instance;
#ifdef DEBUG_ENABLED
		RID navigation_debug_edge_connections_instance;
		Ref<ArrayMesh> navigation_debug_edge_connections_mesh;
#endif // DEBUG_ENABLED

		bool dirty = false;
		RID static_body;
		HashMap<CellKey, NavigationCell> navigation_cell_ids;
	};

	union OctantKeyOld {
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t empty;
		};

		uint64_t key = 0;

		static uint32_t hash(const OctantKeyOld &p_key) {
			return hash_one_uint64(p_key.key);
		}
		_FORCE_INLINE_ bool operator==(const OctantKeyOld &p_key) const {
			return key == p_key.key;
		}
		_FORCE_INLINE_ operator uint64_t() const { return key; }

		// OctantKey(const IndexKey& p_k, int p_item) { indexkey=p_k.key;
		// item=p_item; }
		OctantKeyOld() {}
	};

	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;
	real_t collision_priority = 1.0;
	Ref<PhysicsMaterial> physics_material;
	bool bake_navigation = false;
	RID map_override;

	Transform3D last_transform;

	bool _in_tree = false;
	TypedArray<Basis> cell_orientations;
	Vector3 cell_size = Vector3(2, 2, 2);
	int octant_size = 8;
	bool center_x = true;
	bool center_y = true;
	bool center_z = true;
	float cell_scale = 1.0;

	bool recreating_octants = false;

	Ref<MeshLibrary> mesh_library;

	HashMap<OctantKeyOld, OctantOld *> octant_map;
	HashMap<HexMapCellId::Key, Cell> cell_map;

	// XXX NEW STUFF
	real_t physics_body_friction = 1.0, physics_body_bounce = 0.0;
	HashMap<OctantKey, Octant *> octants;
	Ref<StandardMaterial3D> collision_debug_mat;

	void _recreate_octant_data();

	_FORCE_INLINE_ Vector3 _octant_get_offset(
			const OctantKeyOld &p_key) const {
		return Vector3(p_key.x, p_key.y, p_key.z) * cell_size * octant_size;
	}

	void _update_physics_bodies_collision_properties();
	void _update_physics_bodies_characteristics();
	void _octant_enter_world(const OctantKeyOld &p_key);
	void _octant_exit_world(const OctantKeyOld &p_key);
	bool _octant_update(const OctantKeyOld &p_key);
	void _octant_clean_up(const OctantKeyOld &p_key);
	void _octant_transform(const OctantKeyOld &p_key);
#ifdef DEBUG_ENABLED
	void _update_octant_navigation_debug_edge_connections_mesh(
			const OctantKeyOld &p_key);
	void _navigation_map_changed(RID p_map);
	void _update_navigation_debug_edge_connections();
#endif // DEBUG_ENABLED
	bool awaiting_update = false;

	void _queue_octants_dirty();
	void _update_octants_callback();

	void _clear_internal();

	Vector3 _get_offset() const;

	struct BakedMesh {
		Ref<Mesh> mesh;
		RID instance;
	};

	Vector<BakedMesh> baked_meshes;

protected:
	bool _set(const StringName &p_name, const Variant &p_value);
	bool _get(const StringName &p_name, Variant &r_ret) const;
	void _get_property_list(List<PropertyInfo> *p_list) const;

	void _notification(int p_what);
	void _update_visibility();
	static void _bind_methods();

public:
	enum { INVALID_CELL_ITEM = -1 };

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

	Array get_collision_shapes() const;

	void set_bake_navigation(bool p_bake_navigation);
	bool is_baking_navigation();

	void set_navigation_map(RID p_navigation_map);
	RID get_navigation_map() const;

	void set_mesh_library(const Ref<MeshLibrary> &p_mesh_library);
	Ref<MeshLibrary> get_mesh_library() const;

	void set_cell_size(const Vector3 &p_size);
	Vector3 get_cell_size() const;

	void set_octant_size(int p_size);
	int get_octant_size() const;

	void set_center_x(bool p_enable);
	bool get_center_x() const;
	void set_center_y(bool p_enable);
	bool get_center_y() const;
	void set_center_z(bool p_enable);
	bool get_center_z() const;

	void
	set_cell_item_evil(const HexMapCellId &cell_id, int p_item, int p_rot = 0);
	void set_cell_item(const HexMapCellId &cell_id, int p_item, int p_rot = 0);
	void do_nothing(const HexMapCellId &cell_id, int p_item, int p_rot = 0);
	void _set_cell_item(const Ref<HexMapCellIdRef> cell_id,
			int p_item,
			int p_rot = 0);
	void _set_cell_item_v(const Vector3i &cell_id, int p_item, int p_rot = 0);
	void _fill_cells(const Array &p_cells, int p_item, int p_rot = 0);
	int get_cell_item(const HexMapCellId &cell_id) const;
	int _get_cell_item(const Ref<HexMapCellIdRef> p_cell_id) const;
	int get_cell_item_orientation(const HexMapCellId &cell_id) const;
	int _get_cell_item_orientation(const Ref<HexMapCellIdRef> cell_id) const;

	HexMapCellId local_to_cell_id(const Vector3 &local_position) const;
	Ref<HexMapCellIdRef> _local_to_cell_id(
			const Vector3 &p_local_position) const;
	Vector3 cell_id_to_local(const HexMapCellId &cell_id) const;
	Vector3 _cell_id_to_local(
			const Ref<HexMapCellIdRef> p_local_position) const;

	Vector<HexMapCellId>
			local_region_to_map(Vector3, Vector3, Planes = Planes::All) const;
	TypedArray<Vector3i> _local_region_to_map(Vector3 p_local_point_a,
			Vector3 p_local_point_b) const;

	void set_cell_scale(float p_scale);
	float get_cell_scale() const;

	Array get_used_cells() const;
	TypedArray<Vector3i> get_used_cells_by_item(int p_item) const;

	Array get_meshes() const;

	void clear_baked_meshes();
	void make_baked_meshes(bool p_gen_lightmap_uv = false,
			float p_lightmap_uv_texel_size = 0.1);

	void clear();

	Array get_bake_meshes();
	RID get_bake_mesh_instance(int p_idx);

	HexMap();
	~HexMap();
};

#endif // GRID_MAP_H
