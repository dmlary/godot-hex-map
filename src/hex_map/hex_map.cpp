/**************************************************************************/
/*  grid_map.cpp                                                          */
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

#include "hex_map.h"
#include "godot_cpp/classes/engine.hpp"
#include "godot_cpp/classes/project_settings.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "hex_map/octant.h"
#include "hex_map_cell_id.h"
#include "profiling.h"
#include "tile_orientation.h"

#include "godot_cpp/variant/basis.hpp"
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/main_loop.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/navigation_mesh.hpp>
#include <godot_cpp/classes/navigation_server3d.hpp>
#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/physics_material.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/templates/pair.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

bool HexMap::_set(const StringName &p_name, const Variant &p_value) {
	String name = p_name;

	if (name == "data") {
		Dictionary d = p_value;

		if (d.has("cells")) {
			const PackedByteArray cells = d["cells"];
			ERR_FAIL_COND_V(cells.size() % 10 != 0, false);

			size_t offset = 0;
			while (offset < cells.size()) {
				CellId::Key key;
				key.q = cells.decode_s16(offset);
				offset += 2;
				key.r = cells.decode_s16(offset);
				offset += 2;
				key.y = cells.decode_s16(offset);
				offset += 2;

				Cell cell;
				cell.cell = cells.decode_u32(offset);
				offset += 4;

				cell_map[key] = cell;
			}
		}

		_recreate_octant_data();

	} else if (name == "baked_meshes") {
		clear_baked_meshes();

		Array meshes = p_value;

		// this will change to array[3], Ref<Mesh>
		for (int i = 0; i < meshes.size(); i++) {
			BakedMesh bm;
			bm.mesh = meshes[i];
			ERR_CONTINUE(!bm.mesh.is_valid());
			bm.instance = RS::get_singleton()->instance_create();
			RS::get_singleton()->instance_set_base(
					bm.instance, bm.mesh->get_rid());
			RS::get_singleton()->instance_attach_object_instance_id(
					bm.instance, get_instance_id());
			if (is_inside_tree()) {
				RS::get_singleton()->instance_set_scenario(
						bm.instance, get_world_3d()->get_scenario());
				RS::get_singleton()->instance_set_transform(
						bm.instance, get_global_transform());
			}
			baked_meshes.push_back(bm);
		}

		_recreate_octant_data();

	} else {
		return false;
	}

	return true;
}

bool HexMap::_get(const StringName &p_name, Variant &r_ret) const {
	String name = p_name;

	if (name == "data") {
		Dictionary d;

		PackedByteArray cells;
		cells.resize(cell_map.size() * 10);
		size_t offset = 0;
		for (const KeyValue<CellKey, Cell> &E : cell_map) {
			cells.encode_s16(offset, E.key.q);
			offset += 2;
			cells.encode_s16(offset, E.key.r);
			offset += 2;
			cells.encode_s16(offset, E.key.y);
			offset += 2;
			cells.encode_u32(offset, E.value.cell);
			offset += 4;
		}

		d["cells"] = cells;

		r_ret = d;
	} else if (name == "baked_meshes") {
		Array ret;
		ret.resize(baked_meshes.size());
		for (int i = 0; i < baked_meshes.size(); i++) {
			ret[i] = baked_meshes[i].mesh;
		}
		r_ret = ret;

	} else {
		return false;
	}

	return true;
}

void HexMap::_get_property_list(List<PropertyInfo> *p_list) const {
	if (baked_meshes.size()) {
		p_list->push_back(PropertyInfo(Variant::ARRAY,
				"baked_meshes",
				PROPERTY_HINT_NONE,
				"",
				PROPERTY_USAGE_STORAGE));
	}

	p_list->push_back(PropertyInfo(Variant::DICTIONARY,
			"data",
			PROPERTY_HINT_NONE,
			"",
			PROPERTY_USAGE_STORAGE));
}

void HexMap::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	_update_physics_bodies_collision_properties();
}

uint32_t HexMap::get_collision_layer() const { return collision_layer; }

void HexMap::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	_update_physics_bodies_collision_properties();
}

uint32_t HexMap::get_collision_mask() const { return collision_mask; }

void HexMap::set_collision_layer_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1,
			"Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32,
			"Collision layer number must be between 1 and 32 inclusive.");
	uint32_t collision_layer_new = get_collision_layer();
	if (p_value) {
		collision_layer_new |= 1 << (p_layer_number - 1);
	} else {
		collision_layer_new &= ~(1 << (p_layer_number - 1));
	}
	set_collision_layer(collision_layer_new);
}

bool HexMap::get_collision_layer_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1,
			false,
			"Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32,
			false,
			"Collision layer number must be between 1 and 32 inclusive.");
	return get_collision_layer() & (1 << (p_layer_number - 1));
}

void HexMap::set_collision_mask_value(int p_layer_number, bool p_value) {
	ERR_FAIL_COND_MSG(p_layer_number < 1,
			"Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_MSG(p_layer_number > 32,
			"Collision layer number must be between 1 and 32 inclusive.");
	uint32_t mask = get_collision_mask();
	if (p_value) {
		mask |= 1 << (p_layer_number - 1);
	} else {
		mask &= ~(1 << (p_layer_number - 1));
	}
	set_collision_mask(mask);
}

void HexMap::set_collision_priority(real_t p_priority) {
	collision_priority = p_priority;
	_update_physics_bodies_collision_properties();
}

real_t HexMap::get_collision_priority() const { return collision_priority; }

void HexMap::set_physics_material(Ref<PhysicsMaterial> p_material) {
	physics_material = p_material;
	_update_physics_bodies_characteristics();
}

Ref<PhysicsMaterial> HexMap::get_physics_material() const {
	return physics_material;
}

bool HexMap::get_collision_mask_value(int p_layer_number) const {
	ERR_FAIL_COND_V_MSG(p_layer_number < 1,
			false,
			"Collision layer number must be between 1 and 32 inclusive.");
	ERR_FAIL_COND_V_MSG(p_layer_number > 32,
			false,
			"Collision layer number must be between 1 and 32 inclusive.");
	return get_collision_mask() & (1 << (p_layer_number - 1));
}

Array HexMap::get_collision_shapes() const {
	Array shapes;
	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		OctantOld *g = E.value;
		RID body = g->static_body;
		Transform3D body_xform =
				PhysicsServer3D::get_singleton()->body_get_state(
						body, PhysicsServer3D::BODY_STATE_TRANSFORM);
		int nshapes =
				PhysicsServer3D::get_singleton()->body_get_shape_count(body);
		for (int i = 0; i < nshapes; i++) {
			RID shape =
					PhysicsServer3D::get_singleton()->body_get_shape(body, i);
			Transform3D xform =
					PhysicsServer3D::get_singleton()->body_get_shape_transform(
							body, i);
			shapes.push_back(body_xform * xform);
			shapes.push_back(shape);
		}
	}

	return shapes;
}

void HexMap::set_bake_navigation(bool p_bake_navigation) {
	bake_navigation = p_bake_navigation;
	_recreate_octant_data();
}

bool HexMap::is_baking_navigation() { return bake_navigation; }

void HexMap::set_navigation_map(RID p_navigation_map) {
	map_override = p_navigation_map;
	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		OctantOld &g = *octant_map[E.key];
		for (KeyValue<CellKey, OctantOld::NavigationCell> &F :
				g.navigation_cell_ids) {
			if (F.value.region.is_valid()) {
				NavigationServer3D::get_singleton()->region_set_map(
						F.value.region, map_override);
			}
		}
	}
}

RID HexMap::get_navigation_map() const {
	if (map_override.is_valid()) {
		return map_override;
	} else if (is_inside_tree()) {
		return get_world_3d()->get_navigation_map();
	}
	return RID();
}

void HexMap::set_mesh_library(const Ref<MeshLibrary> &p_mesh_library) {
	if (!mesh_library.is_null()) {
		mesh_library->disconnect(
				"changed", callable_mp(this, &HexMap::_recreate_octant_data));
	}
	mesh_library = p_mesh_library;
	if (!mesh_library.is_null()) {
		mesh_library->connect(
				"changed", callable_mp(this, &HexMap::_recreate_octant_data));
	}

	_recreate_octant_data();
	emit_signal("mesh_library_changed");
}

Ref<MeshLibrary> HexMap::get_mesh_library() const { return mesh_library; }

void HexMap::set_cell_size(const Vector3 &p_size) {
	ERR_FAIL_COND(p_size.x < 0.001 || p_size.y < 0.001 || p_size.z < 0.001);
	cell_size = p_size;
	// hex cells have a radius stored in x, and height stored in y.  To make
	// it clear that irregular hexagons are not supported, the z value of hex
	// cells will always be updated to be the same as x.
	cell_size.z = cell_size.x;
	_recreate_octant_data();
	emit_signal("cell_size_changed", cell_size);
}

Vector3 HexMap::get_cell_size() const { return cell_size; }

void HexMap::set_octant_size(int p_size) {
	ERR_FAIL_COND(p_size == 0);
	octant_size = p_size;
	_recreate_octant_data();
}

int HexMap::get_octant_size() const { return octant_size; }

void HexMap::set_center_x(bool p_enable) {
	center_x = p_enable;
	_recreate_octant_data();
}

bool HexMap::get_center_x() const { return center_x; }

void HexMap::set_center_y(bool p_enable) {
	center_y = p_enable;
	_recreate_octant_data();
}

bool HexMap::get_center_y() const { return center_y; }

void HexMap::set_center_z(bool p_enable) {
	center_z = p_enable;
	_recreate_octant_data();
}

bool HexMap::get_center_z() const { return center_z; }

void HexMap::set_cell_item(const HexMapCellId &cell_id,
		int p_item,
		int p_rot) {
	ERR_FAIL_COND_MSG(!cell_id.in_bounds(), "cell id is not in bounds");

	// if (baked_meshes.size() && !recreating_octants) {
	// 	// if you set a cell item, baked meshes go good bye
	// 	clear_baked_meshes();
	// 	_recreate_octant_data();
	// }

	// auto profile = profiling_begin(
	// 		"set_cell_item(%hd, %hd, %hd)", cell_id.q, cell_id.r, cell_id.y);
	CellKey cell_key(cell_id);
	OctantKey octant_key(cell_id, octant_size);
	Octant **octant_ptr = octants.getptr(octant_key);
	Octant *octant = octant_ptr ? *octant_ptr : nullptr;

	if (p_item >= 0) {
		Cell cell = {
			.item = static_cast<unsigned int>(p_item),
			.rot = static_cast<unsigned int>(p_rot),
		};

		cell_map.insert(cell_key, cell);

		// possibly create a new octant
		if (octant == nullptr) {
			profiling_emit("allocate new octant",
					"(%hd, %hd, %hd)",
					octant_key.x,
					octant_key.y,
					octant_key.z);
			octant = new Octant(*this);
			octants.insert(octant_key, octant);

			if (is_inside_tree()) {
				// auto ew = profiling_begin("octant enter world");
				octant->enter_world();
			}
		}

		// add a cell to the octant, and schedule an update
		octant->add_cell(cell_key);
		_queue_octants_dirty();
	} else if (cell_map.has(cell_key)) {
		cell_map.erase(cell_key);
		// emit_signal("cell_changed", cell_id);

		ERR_FAIL_COND_MSG(octant == nullptr, "octant for cell does not exist");
		octant->remove_cell(cell_key);
		_queue_octants_dirty();
	}

	// OctantKey octantkey = octant_key;
	//
	// if (!octant_map.has(octantkey)) {
	// 	// create octant because it does not exist
	// 	OctantOld *g = memnew(OctantOld);
	// 	g->dirty = true;
	// 	g->static_body = PhysicsServer3D::get_singleton()->body_create();
	// 	PhysicsServer3D::get_singleton()->body_set_mode(
	// 			g->static_body, PhysicsServer3D::BODY_MODE_STATIC);
	// 	PhysicsServer3D::get_singleton()->body_attach_object_instance_id(
	// 			g->static_body, get_instance_id());
	// 	PhysicsServer3D::get_singleton()->body_set_collision_layer(
	// 			g->static_body, collision_layer);
	// 	PhysicsServer3D::get_singleton()->body_set_collision_mask(
	// 			g->static_body, collision_mask);
	// 	PhysicsServer3D::get_singleton()->body_set_collision_priority(
	// 			g->static_body, collision_priority);
	// 	if (physics_material.is_valid()) {
	// 		PhysicsServer3D::get_singleton()->body_set_param(g->static_body,
	// 				PhysicsServer3D::BODY_PARAM_FRICTION,
	// 				// expanded PhysicsMaterial.computed_friction()
	// 				physics_material->is_rough()
	// 						? -physics_material->get_friction()
	// 						: physics_material->get_friction());
	// 		PhysicsServer3D::get_singleton()->body_set_param(g->static_body,
	// 				PhysicsServer3D::BODY_PARAM_BOUNCE,
	// 				// expanded PhysicsMaterial.computed_bounce()
	// 				physics_material->is_absorbent()
	// 						? -physics_material->get_bounce()
	// 						: physics_material->get_bounce());
	// 	}
	// 	SceneTree *st = get_tree();
	//
	// 	// if (st && st->is_debugging_collisions_hint()) {
	// 	g->collision_debug = RenderingServer::get_singleton()->mesh_create();
	// 	g->collision_debug_instance =
	// 			RenderingServer::get_singleton()->instance_create();
	// 	RenderingServer::get_singleton()->instance_set_base(
	// 			g->collision_debug_instance, g->collision_debug);
	// 	// }
	//
	// 	octant_map[octantkey] = g;
	//
	// 	if (is_inside_tree()) {
	// 		_octant_enter_world(octant_key);
	// 		_octant_transform(octant_key);
	// 	}
	// }
	//
	// OctantOld &g = *octant_map[octantkey];
	// g.cells.insert(cell_key);
	// g.dirty = true;
	// _queue_octants_dirty();

	// Cell c;
	// c.item = p_item;
	// c.rot = p_rot;
	//
	// cell_map[cell_key] = c;
	// emit_signal("cell_changed", cell_id);
}
void HexMap::_set_cell_item(const Ref<HexMapCellIdRef> cell_id,
		int p_item,
		int p_rot) {
	ERR_FAIL_COND_MSG(!cell_id.is_valid(), "null cell id");
	set_cell_item(**cell_id, p_item, p_rot);
}

int HexMap::get_cell_item(const HexMapCellId &cell_id) const {
	ERR_FAIL_COND_V_MSG(
			!cell_id.in_bounds(), INVALID_CELL_ITEM, "cell id not in bounds");

	CellKey key(cell_id);

	if (!cell_map.has(key)) {
		return INVALID_CELL_ITEM;
	}
	return cell_map[key].item;
}

int HexMap::_get_cell_item(const Ref<HexMapCellIdRef> p_cell_id) const {
	return get_cell_item(**p_cell_id);
}

int HexMap::get_cell_item_orientation(const HexMapCellId &cell_id) const {
	ERR_FAIL_COND_V_MSG(!cell_id.in_bounds(), -1, "CellId out of bounds");

	CellKey key(cell_id);
	if (!cell_map.has(key)) {
		return -1;
	}
	return cell_map[key].rot;
}

int HexMap::_get_cell_item_orientation(
		const Ref<HexMapCellIdRef> p_cell_id) const {
	return get_cell_item_orientation(**p_cell_id);
}

// based on blog post https://observablehq.com/@jrus/hexround
static inline Vector2i axial_round(real_t q_in, real_t r_in) {
	int q = round(q_in);
	int r = round(r_in);

	real_t q_rem = q_in - q;
	real_t r_rem = r_in - r;

	if (abs(q_rem) >= abs(r_rem)) {
		q += round(0.5 * r_rem + q_rem);
	} else {
		r += round(0.5 * q_rem + r_rem);
	}

	return Vector2i(q, r);
}

// convert axial hex coordinates to offset coordinates
// https://www.redblobgames.com/grids/hexagons/#conversions-offset
static inline Vector3i axial_to_oddr(Vector3i axial) {
	int x = axial.x + (axial.z - (axial.z & 1)) / 2;
	return Vector3i(x, axial.y, axial.z);
}

static inline Vector3i oddr_to_axial(Vector3i oddr) {
	int q = oddr.x - (oddr.z - (oddr.z & 1)) / 2;
	return Vector3i(q, oddr.y, oddr.z);
}

HexMapCellId HexMap::local_to_cell_id(const Vector3 &local_position) const {
	Vector3 unit_pos = local_position / cell_size;
	return HexMapCellId::from_unit_point(unit_pos);
}

Ref<HexMapCellIdRef> HexMap::_local_to_cell_id(
		const Vector3 &p_local_position) const {
	return local_to_cell_id(p_local_position);
}

Vector3 HexMap::cell_id_to_local(const HexMapCellId &cell_id) const {
	return cell_id.unit_center() * cell_size;
}

Vector3 HexMap::_cell_id_to_local(const Ref<HexMapCellIdRef> cell_id) const {
	return cell_id_to_local(**cell_id);
}

// Vector3 HexMap::map_to_local(const HexMapCellId &p_cell_id) const {
// 	Vector3 offset = _get_offset();
//
// 	// convert axial hex coordinates to a point
// 	// https://www.redblobgames.com/grids/hexagons/#hex-to-pixel
// 	Vector3 local;
// 	local.x = cell_size.x * (Math_SQRT3 * p_cell_id.q + SQRT3_2 *
// p_cell_id.r); 	local.y = p_cell_id.y * cell_size.y + offset.y; 	local.z
// = cell_size.x * (3.0 / 2 * p_cell_id.r); 	return local;
// }
Vector<HexMapCellId>
HexMap::local_region_to_map(Vector3 p_a, Vector3 p_b, Planes planes) const {
	Vector<HexMapCellId> cells;

	// XXX OddR Iterator, and Planes support

	// shuffle the fields of a & b around so that a is bottom-left, b is
	// top-right
	if (p_a.x > p_b.x) {
		SWAP(p_a.x, p_b.x);
	}
	if (p_a.y > p_b.y) {
		SWAP(p_a.y, p_b.y);
	}
	if (p_a.z > p_b.z) {
		SWAP(p_a.z, p_b.z);
	}
	Vector3i bottom_left = local_to_cell_id(p_a);
	Vector3i top_right = local_to_cell_id(p_b);

	// we need the x coordinate of the center of the corner cells
	// later. grab them now before we switch coordinate systems.
	real_t left_x_center = cell_id_to_local(bottom_left).x;
	real_t right_x_center = cell_id_to_local(top_right).x;

	// we're going to use a different coordinate system for this
	// operation.  It's much easier to walk the region when we use
	// offset coordinates.  So let's map our corners from axial to
	// offset, then walk the region the same as the square region.
	// We'll convert the coordinates back to axial before putting them
	// in the array.
	bottom_left = axial_to_oddr(bottom_left);
	top_right = axial_to_oddr(top_right);

	// Also, unlike square cells, the location of the corner of the
	// region within a cell matters for hex cells, specifically the x
	// coordinate.  If you pick a point anywhere within a hex cell,
	// and draw a line down along the z-axis, that line will intercept
	// either the cell to the southwest or southeast of the clicked
	// cell.
	//
	// For both the left and right sides of the region, we need to
	// determine which of the southwest/southeast cells fall within
	// the region.  We do this by adjusting the x-min and x-max for the
	// even and odd rows independently.  We use the following table to
	// determine the modifier for the rows for both the minimum x
	// value (in bottom_left.x), and the maximum x value (in
	// top_right.x).
	//
	// Given an x coordinate in local space:
	// | cell z coord | x > cell_center.x | odd mod | even mod |
	// | even         | false             |  -1     | 0        |
	// | even         | true              |   0     | 0        |
	// | odd          | false             |   0     | 0        |
	// | odd          | true              |   0     | 1        |

	// adjustment applied to the min x value for odd and even cells
	int x_min_delta[2] = { 0, 0 };

	// if we start on an odd row, and the region starts to the right
	// of center, we want to skip the even cells at x == a.x.
	if ((bottom_left.z & 1) == 1 && p_a.x > left_x_center) {
		x_min_delta[0] = 1;
	}
	// if we start on an even row, and the region starts to the left
	// of center, we want to include the odd cells at x = a.x - 1.
	else if ((bottom_left.z & 1) == 0 && p_a.x <= left_x_center) {
		x_min_delta[1] = -1;
	}

	// same as above, but for the max x values
	int x_max_delta[2] = { 0, 0 };
	if ((top_right.z & 1) == 1 && p_b.x > right_x_center) {
		x_max_delta[0] = 1;
	} else if ((top_right.z & 1) == 0 && p_b.x <= right_x_center) {
		x_max_delta[1] = -1;
	}
	for (int z = bottom_left.z; z <= top_right.z; z++) {
		for (int y = bottom_left.y; y <= top_right.y; y++) {
			int min_x = bottom_left.x + x_min_delta[z & 1];
			int max_x = top_right.x + x_max_delta[z & 1];
			for (int x = min_x; x <= max_x; x++) {
				Vector3i oddr = Vector3i(x, y, z);
				Vector3i axial = oddr_to_axial(oddr);
				cells.push_back(axial);
			}
		}
	}

	return cells;
}

TypedArray<Vector3i> HexMap::_local_region_to_map(Vector3 p_a,
		Vector3 p_b) const {
	TypedArray<Vector3i> out;
	for (const HexMapCellId &cell : local_region_to_map(p_a, p_b)) {
		out.append((Vector3i)cell);
	}
	return out;
}

void HexMap::_octant_transform(const OctantKeyOld &p_key) {
	// ERR_FAIL_COND(!octant_map.has(p_key));
	// OctantOld &g = *octant_map[p_key];
	// PhysicsServer3D::get_singleton()->body_set_state(g.static_body,
	// 		PhysicsServer3D::BODY_STATE_TRANSFORM,
	// 		get_global_transform());
	//
	// if (g.collision_debug_instance.is_valid()) {
	// 	RS::get_singleton()->instance_set_transform(
	// 			g.collision_debug_instance, get_global_transform());
	// }
	//
	// // update transform for NavigationServer regions and navigation
	// // debugmesh instances
	// for (const KeyValue<CellKey, OctantOld::NavigationCell> &E :
	// 		g.navigation_cell_ids) {
	// 	if (bake_navigation) {
	// 		if (E.value.region.is_valid()) {
	// 			NavigationServer3D::get_singleton()->region_set_transform(
	// 					E.value.region,
	// 					get_global_transform() * E.value.xform);
	// 		}
	// 		if (E.value.navigation_mesh_debug_instance.is_valid()) {
	// 			RS::get_singleton()->instance_set_transform(
	// 					E.value.navigation_mesh_debug_instance,
	// 					get_global_transform() * E.value.xform);
	// 		}
	// 	}
	// }
	//
	// for (int i = 0; i < g.multimesh_instances.size(); i++) {
	// 	RS::get_singleton()->instance_set_transform(
	// 			g.multimesh_instances[i].instance, get_global_transform());
	// }
}

bool HexMap::_octant_update(const OctantKeyOld &p_key) {
	ERR_FAIL_COND_V(!octant_map.has(p_key), false);
	OctantOld &g = *octant_map[p_key];
	if (!g.dirty) {
		return false;
	}

	// erase body shapes
	PhysicsServer3D::get_singleton()->body_clear_shapes(g.static_body);

	// erase body shapes debug
	if (g.collision_debug.is_valid()) {
		RS::get_singleton()->mesh_clear(g.collision_debug);
	}

	// erase navigation
	for (KeyValue<CellKey, OctantOld::NavigationCell> &E :
			g.navigation_cell_ids) {
		if (E.value.region.is_valid()) {
			NavigationServer3D::get_singleton()->free_rid(E.value.region);
			E.value.region = RID();
		}
		if (E.value.navigation_mesh_debug_instance.is_valid()) {
			RS::get_singleton()->free_rid(
					E.value.navigation_mesh_debug_instance);
			E.value.navigation_mesh_debug_instance = RID();
		}
	}
	g.navigation_cell_ids.clear();

	// erase multimeshes

	for (int i = 0; i < g.multimesh_instances.size(); i++) {
		RS::get_singleton()->free_rid(g.multimesh_instances[i].instance);
		RS::get_singleton()->free_rid(g.multimesh_instances[i].multimesh);
	}
	g.multimesh_instances.clear();

	if (g.cells.size() == 0) {
		// octant no longer needed
		_octant_clean_up(p_key);
		return true;
	}

	PackedVector3Array collision_debug_vectors;

	/*
	 * foreach item in this octant,
	 * set item's multimesh's instance count to number of cells which have
	 * this item and set said multimesh bounding box to one containing all
	 * cells which have this item
	 */

	HashMap<int, List<Pair<Transform3D, CellKey>>> multimesh_items;

	for (const CellKey &E : g.cells) {
		ERR_CONTINUE(!cell_map.has(E));
		const Cell &c = cell_map[E];

		// XXX MeshLibrary::has_item() not exported; need to find another
		// way
		if (!mesh_library.is_valid()) {
			continue;
		}

		HexMapCellId map_pos(E);
		Transform3D cell_transform;

		cell_transform.basis = TileOrientation(c.rot);
		cell_transform.set_origin(cell_id_to_local(map_pos));
		cell_transform.basis.scale(
				Vector3(cell_scale, cell_scale, cell_scale));
		if (baked_meshes.size() == 0) {
			if (mesh_library->get_item_mesh(c.item).is_valid()) {
				if (!multimesh_items.has(c.item)) {
					multimesh_items[c.item] =
							List<Pair<Transform3D, CellKey>>();
				}

				Pair<Transform3D, CellKey> p;
				p.first = cell_transform *
						mesh_library->get_item_mesh_transform(c.item);
				p.second = E;
				multimesh_items[c.item].push_back(p);
			}
		}

		Array shapes = mesh_library->get_item_shapes(c.item);
		while (shapes.size() > 0) {
			Object *obj = shapes.pop_front();
			Shape3D *shape = Object::cast_to<Shape3D>(obj);
			Transform3D shape_transform = shapes.pop_front();
			Transform3D local_transform = cell_transform * shape_transform;

			PhysicsServer3D::get_singleton()->body_add_shape(
					g.static_body, shape->get_rid(), local_transform);

			if (g.collision_debug.is_valid()) {
				Ref<ArrayMesh> debug_mesh = shape->get_debug_mesh();

				// // XXX figuring out how to draw the debug shapes
				// if (!debug_material.is_valid()) {
				// 	debug_material = debug_mesh->surface_get_material(0);
				// }

				PackedVector3Array debug_vectors =
						shape->get_debug_mesh()->get_faces();
				for (const Vector3 &v : debug_vectors) {
					collision_debug_vectors.append(local_transform.xform(v));
				}
			}
		}

		// Vector<MeshLibrary::ShapeData> shapes =
		// 		mesh_library->get_item_shapes(c.item);
		// // add the item's shape at given xform to octant's static_body
		// for (int i = 0; i < shapes.size(); i++) {
		// 	// add the item's shape
		// 	if (!shapes[i].shape.is_valid()) {
		// 		continue;
		// 	}
		// 	PhysicsServer3D::get_singleton()->body_add_shape(
		// 			g.static_body, shapes[i].shape->get_rid(),
		// 			xform * shapes[i].local_transform);
		// 	if (g.collision_debug.is_valid()) {
		// 		shapes.write[i].shape->add_vertices_to_array(
		// 				col_debug, xform * shapes[i].local_transform);
		// 	}
		// }

		// add the item's navigation_mesh at given xform to GridMap's
		// Navigation ancestor
		Ref<NavigationMesh> navigation_mesh =
				mesh_library->get_item_navigation_mesh(c.item);
		if (navigation_mesh.is_valid()) {
			OctantOld::NavigationCell nm;
			nm.xform = cell_transform *
					mesh_library->get_item_navigation_mesh_transform(c.item);
			nm.navigation_layers =
					mesh_library->get_item_navigation_layers(c.item);

			if (bake_navigation) {
				RID region =
						NavigationServer3D::get_singleton()->region_create();
				NavigationServer3D::get_singleton()->region_set_owner_id(
						region, get_instance_id());
				NavigationServer3D::get_singleton()
						->region_set_navigation_layers(
								region, nm.navigation_layers);
				NavigationServer3D::get_singleton()
						->region_set_navigation_mesh(region, navigation_mesh);
				NavigationServer3D::get_singleton()->region_set_transform(
						region, get_global_transform() * nm.xform);
				if (is_inside_tree()) {
					if (map_override.is_valid()) {
						NavigationServer3D::get_singleton()->region_set_map(
								region, map_override);
					} else {
						NavigationServer3D::get_singleton()->region_set_map(
								region, get_world_3d()->get_navigation_map());
					}
				}
				nm.region = region;

#ifdef DEBUG_ENABLED
				// XXX navigation_mesh->get_debug_mesh() does not exist
				//
				// // add navigation debugmesh visual instances if debug is
				// enabled SceneTree *st = get_tree(); if (st &&
				// st->is_debugging_navigation_hint()) { 	if
				// (!nm.navigation_mesh_debug_instance.is_valid()) {
				// RID navigation_mesh_debug_rid =
				// 				navigation_mesh->get_debug_mesh()->get_rid();
				// 		nm.navigation_mesh_debug_instance =
				// 				RS::get_singleton()->instance_create();
				// 		RS::get_singleton()->instance_set_base(
				// 				nm.navigation_mesh_debug_instance,
				// navigation_mesh_debug_rid);
				// 	}
				// 	if (is_inside_tree()) {
				// 		RS::get_singleton()->instance_set_scenario(
				// 				nm.navigation_mesh_debug_instance,
				// 				get_world_3d()->get_scenario());
				// 		RS::get_singleton()->instance_set_transform(
				// 				nm.navigation_mesh_debug_instance,
				// 				get_global_transform() * nm.xform);
				// 	}
				// }
#endif // DEBUG_ENABLED
			}
			g.navigation_cell_ids[E] = nm;
		}
	}

#ifdef DEBUG_ENABLED
	if (bake_navigation) {
		_update_octant_navigation_debug_edge_connections_mesh(p_key);
	}
#endif // DEBUG_ENABLED

	// update multimeshes, only if not baked
	if (baked_meshes.size() == 0) {
		for (const KeyValue<int, List<Pair<Transform3D, CellKey>>> &E :
				multimesh_items) {
			OctantOld::MultimeshInstance mmi;

			RID mm = RS::get_singleton()->multimesh_create();
			RS::get_singleton()->multimesh_allocate_data(
					mm, E.value.size(), RS::MULTIMESH_TRANSFORM_3D);
			RS::get_singleton()->multimesh_set_mesh(
					mm, mesh_library->get_item_mesh(E.key)->get_rid());

			int idx = 0;
			for (const Pair<Transform3D, CellKey> &F : E.value) {
				RS::get_singleton()->multimesh_instance_set_transform(
						mm, idx, F.first);
#ifdef TOOLS_ENABLED

				OctantOld::MultimeshInstance::Item it;
				it.index = idx;
				it.transform = F.first;
				it.key = F.second;
				mmi.items.push_back(it);
#endif

				idx++;
			}

			RID instance = RS::get_singleton()->instance_create();
			RS::get_singleton()->instance_set_base(instance, mm);

			if (is_inside_tree()) {
				RS::get_singleton()->instance_set_scenario(
						instance, get_world_3d()->get_scenario());
				RS::get_singleton()->instance_set_transform(
						instance, get_global_transform());
			}

			mmi.multimesh = mm;
			mmi.instance = instance;

			g.multimesh_instances.push_back(mmi);
		}
	}

	if (!collision_debug_vectors.is_empty()) {
		Array arr;
		arr.resize(RS::ARRAY_VERTEX);
		arr[RS::ARRAY_VERTEX] = collision_debug_vectors;
	}

	if (!collision_debug_vectors.is_empty()) {
		Array arr;
		arr.resize(RS::ARRAY_MAX);
		arr[RS::ARRAY_VERTEX] = collision_debug_vectors;

		RS::get_singleton()->mesh_add_surface_from_arrays(
				g.collision_debug, RS::PRIMITIVE_LINES, arr);
		// SceneTree *st = get_tree();
		// if (st) {
		// 	RS::get_singleton()->mesh_surface_set_material(
		// 			g.collision_debug, 0, debug_material->get_rid());
		// }
	}

	g.dirty = false;

	return false;
}

void HexMap::_update_physics_bodies_collision_properties() {
	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		PhysicsServer3D::get_singleton()->body_set_collision_layer(
				E.value->static_body, collision_layer);
		PhysicsServer3D::get_singleton()->body_set_collision_mask(
				E.value->static_body, collision_mask);
		PhysicsServer3D::get_singleton()->body_set_collision_priority(
				E.value->static_body, collision_priority);
	}
}

// PhysicsMaterial.computed_friction() & computed_bounce() not exposed in
// godot-cpp
#define computed_friction(mat)                                                \
	((mat)->is_rough() ? -mat->get_friction() : mat->get_friction())
#define computed_bounce(mat)                                                  \
	((mat)->is_absorbent() ? -mat->get_bounce() : mat->get_bounce())
void HexMap::_update_physics_bodies_characteristics() {
	physics_body_friction = 1.0;
	physics_body_bounce = 0.0;
	if (physics_material.is_valid()) {
		physics_body_friction = physics_material->is_rough()
				? -physics_material->get_friction()
				: physics_material->get_friction();
		physics_body_bounce = physics_material->is_absorbent()
				? -physics_material->get_bounce()
				: physics_material->get_bounce();
	}
	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		PhysicsServer3D::get_singleton()->body_set_param(E.value->static_body,
				PhysicsServer3D::BODY_PARAM_FRICTION,
				physics_body_friction);
		PhysicsServer3D::get_singleton()->body_set_param(E.value->static_body,
				PhysicsServer3D::BODY_PARAM_BOUNCE,
				physics_body_bounce);
	}
}

void HexMap::_octant_enter_world(const OctantKeyOld &p_key) {
	ERR_FAIL_COND(!octant_map.has(p_key));
	OctantOld &g = *octant_map[p_key];
	PhysicsServer3D::get_singleton()->body_set_state(g.static_body,
			PhysicsServer3D::BODY_STATE_TRANSFORM,
			get_global_transform());
	PhysicsServer3D::get_singleton()->body_set_space(
			g.static_body, get_world_3d()->get_space());

	if (g.collision_debug_instance.is_valid()) {
		RS::get_singleton()->instance_set_scenario(
				g.collision_debug_instance, get_world_3d()->get_scenario());
		RS::get_singleton()->instance_set_transform(
				g.collision_debug_instance, get_global_transform());
	}

	for (int i = 0; i < g.multimesh_instances.size(); i++) {
		RS::get_singleton()->instance_set_scenario(
				g.multimesh_instances[i].instance,
				get_world_3d()->get_scenario());
		RS::get_singleton()->instance_set_transform(
				g.multimesh_instances[i].instance, get_global_transform());
	}

	if (bake_navigation && mesh_library.is_valid()) {
		for (KeyValue<CellKey, OctantOld::NavigationCell> &F :
				g.navigation_cell_ids) {
			if (cell_map.has(F.key) && F.value.region.is_valid() == false) {
				Ref<NavigationMesh> navigation_mesh =
						mesh_library->get_item_navigation_mesh(
								cell_map[F.key].item);
				if (navigation_mesh.is_valid()) {
					RID region = NavigationServer3D::get_singleton()
										 ->region_create();
					NavigationServer3D::get_singleton()->region_set_owner_id(
							region, get_instance_id());
					NavigationServer3D::get_singleton()
							->region_set_navigation_layers(
									region, F.value.navigation_layers);
					NavigationServer3D::get_singleton()
							->region_set_navigation_mesh(
									region, navigation_mesh);
					NavigationServer3D::get_singleton()->region_set_transform(
							region, get_global_transform() * F.value.xform);
					if (map_override.is_valid()) {
						NavigationServer3D::get_singleton()->region_set_map(
								region, map_override);
					} else {
						NavigationServer3D::get_singleton()->region_set_map(
								region, get_world_3d()->get_navigation_map());
					}

					F.value.region = region;
				}
			}
		}

#ifdef DEBUG_ENABLED
		if (bake_navigation) {
			if (!g.navigation_debug_edge_connections_instance.is_valid()) {
				g.navigation_debug_edge_connections_instance =
						RenderingServer::get_singleton()->instance_create();
			}
			if (!g.navigation_debug_edge_connections_mesh.is_valid()) {
				g.navigation_debug_edge_connections_mesh =
						Ref<ArrayMesh>(memnew(ArrayMesh));
			}

			_update_octant_navigation_debug_edge_connections_mesh(p_key);
		}
#endif // DEBUG_ENABLED
	}
}

void HexMap::_octant_exit_world(const OctantKeyOld &p_key) {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	ERR_FAIL_NULL(PhysicsServer3D::get_singleton());
	ERR_FAIL_NULL(NavigationServer3D::get_singleton());

	ERR_FAIL_COND(!octant_map.has(p_key));
	OctantOld &g = *octant_map[p_key];
	PhysicsServer3D::get_singleton()->body_set_state(g.static_body,
			PhysicsServer3D::BODY_STATE_TRANSFORM,
			get_global_transform());
	PhysicsServer3D::get_singleton()->body_set_space(g.static_body, RID());

	if (g.collision_debug_instance.is_valid()) {
		RS::get_singleton()->instance_set_scenario(
				g.collision_debug_instance, RID());
	}

	for (int i = 0; i < g.multimesh_instances.size(); i++) {
		RS::get_singleton()->instance_set_scenario(
				g.multimesh_instances[i].instance, RID());
	}

	for (KeyValue<CellKey, OctantOld::NavigationCell> &F :
			g.navigation_cell_ids) {
		if (F.value.region.is_valid()) {
			NavigationServer3D::get_singleton()->free_rid(F.value.region);
			F.value.region = RID();
		}
		if (F.value.navigation_mesh_debug_instance.is_valid()) {
			RS::get_singleton()->free_rid(
					F.value.navigation_mesh_debug_instance);
			F.value.navigation_mesh_debug_instance = RID();
		}
	}

#ifdef DEBUG_ENABLED
	if (bake_navigation) {
		if (g.navigation_debug_edge_connections_instance.is_valid()) {
			RenderingServer::get_singleton()->free_rid(
					g.navigation_debug_edge_connections_instance);
			g.navigation_debug_edge_connections_instance = RID();
		}
		if (g.navigation_debug_edge_connections_mesh.is_valid()) {
			RenderingServer::get_singleton()->free_rid(
					g.navigation_debug_edge_connections_mesh->get_rid());
		}
	}
#endif // DEBUG_ENABLED
}

void HexMap::_octant_clean_up(const OctantKeyOld &p_key) {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	ERR_FAIL_NULL(PhysicsServer3D::get_singleton());
	ERR_FAIL_NULL(NavigationServer3D::get_singleton());

	ERR_FAIL_COND(!octant_map.has(p_key));
	OctantOld &g = *octant_map[p_key];

	if (g.collision_debug.is_valid()) {
		RS::get_singleton()->free_rid(g.collision_debug);
	}
	if (g.collision_debug_instance.is_valid()) {
		RS::get_singleton()->free_rid(g.collision_debug_instance);
	}

	PhysicsServer3D::get_singleton()->free_rid(g.static_body);

	// Erase navigation
	for (const KeyValue<CellKey, OctantOld::NavigationCell> &E :
			g.navigation_cell_ids) {
		if (E.value.region.is_valid()) {
			NavigationServer3D::get_singleton()->free_rid(E.value.region);
		}
		if (E.value.navigation_mesh_debug_instance.is_valid()) {
			RS::get_singleton()->free_rid(
					E.value.navigation_mesh_debug_instance);
		}
	}
	g.navigation_cell_ids.clear();

#ifdef DEBUG_ENABLED
	if (bake_navigation) {
		if (g.navigation_debug_edge_connections_instance.is_valid()) {
			RenderingServer::get_singleton()->free_rid(
					g.navigation_debug_edge_connections_instance);
			g.navigation_debug_edge_connections_instance = RID();
		}
		if (g.navigation_debug_edge_connections_mesh.is_valid()) {
			RenderingServer::get_singleton()->free_rid(
					g.navigation_debug_edge_connections_mesh->get_rid());
		}
	}
#endif // DEBUG_ENABLED

	// erase multimeshes

	for (int i = 0; i < g.multimesh_instances.size(); i++) {
		RS::get_singleton()->free_rid(g.multimesh_instances[i].instance);
		RS::get_singleton()->free_rid(g.multimesh_instances[i].multimesh);
	}
	g.multimesh_instances.clear();
}

void HexMap::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_POSTINITIALIZE:
			// XXX workaround for connect during initialize
			break;

		case NOTIFICATION_ENTER_WORLD: {
			last_transform = get_global_transform();

			for (auto &pair : octants) {
				pair.value->enter_world();
			}
			// for (int i = 0; i < baked_meshes.size(); i++) {
			// 	RS::get_singleton()->instance_set_scenario(
			// 			baked_meshes[i].instance,
			// 			get_world_3d()->get_scenario());
			// 	RS::get_singleton()->instance_set_transform(
			// 			baked_meshes[i].instance, get_global_transform());
			// }
		} break;

		case NOTIFICATION_ENTER_TREE: {
#ifdef DEBUG_ENABLED
			if (bake_navigation &&
					NavigationServer3D::get_singleton()->get_debug_enabled()) {
				_update_navigation_debug_edge_connections();
			}
#endif // DEBUG_ENABLED
			_update_visibility();
		} break;

		case NOTIFICATION_TRANSFORM_CHANGED: {
			Transform3D new_xform = get_global_transform();
			if (new_xform == last_transform) {
				break;
			}

			for (auto &pair : octants) {
				pair.value->update_transforms();
			}

			last_transform = new_xform;

			// for (int i = 0; i < baked_meshes.size(); i++) {
			// 	RS::get_singleton()->instance_set_transform(
			// 			baked_meshes[i].instance, get_global_transform());
			// }
		} break;

		case NOTIFICATION_EXIT_WORLD: {
			for (auto &pair : octants) {
				pair.value->exit_world();
			}

			//_queue_octants_dirty(MAP_DIRTY_INSTANCES|MAP_DIRTY_TRANSFORMS);
			//_update_octants_callback();
			//_update_area_instances();
			// for (int i = 0; i < baked_meshes.size(); i++) {
			// 	RS::get_singleton()->instance_set_scenario(
			// 			baked_meshes[i].instance, RID());
			// }
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			_update_visibility();
		} break;
	}
}

void HexMap::_update_visibility() {
	if (!is_inside_tree()) {
		return;
	}

	for (KeyValue<OctantKeyOld, OctantOld *> &e : octant_map) {
		OctantOld *octant = e.value;
		for (int i = 0; i < octant->multimesh_instances.size(); i++) {
			const OctantOld::MultimeshInstance &mi =
					octant->multimesh_instances[i];
			RS::get_singleton()->instance_set_visible(
					mi.instance, is_visible_in_tree());
		}
	}

	for (int i = 0; i < baked_meshes.size(); i++) {
		RS::get_singleton()->instance_set_visible(
				baked_meshes[i].instance, is_visible_in_tree());
	}
}

void HexMap::_queue_octants_dirty() {
	if (awaiting_update) {
		return;
	}

	callable_mp(this, &HexMap::_update_octants_callback).call_deferred();
	awaiting_update = true;
}

void HexMap::_recreate_octant_data() {
	recreating_octants = true;
	HashMap<CellKey, Cell> cell_copy = cell_map;
	_clear_internal();
	for (const KeyValue<CellKey, Cell> &E : cell_copy) {
		set_cell_item(CellId(E.key), E.value.item, E.value.rot);
	}
	recreating_octants = false;
}

void HexMap::_clear_internal() {
	for (auto &octant_pair : octants) {
		Octant *octant = octant_pair.value;
		if (is_inside_tree()) {
			octant->exit_world();
		}
		delete octant;
	}
	octants.clear();

	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		if (is_inside_tree()) {
			_octant_exit_world(E.key);
		}

		_octant_clean_up(E.key);
		memdelete(E.value);
	}

	octant_map.clear();
	cell_map.clear();
}

void HexMap::clear() {
	_clear_internal();
	clear_baked_meshes();
}

void HexMap::_update_octants_callback() {
	if (!awaiting_update) {
		return;
	}

	Vector<OctantKey> empty_octants;
	for (const auto pair : octants) {
		Octant *octant = pair.value;
		if (!octant->is_dirty()) {
			continue;
		}

		octant->update();
		if (octant->is_empty()) {
			empty_octants.push_back(pair.key);
		}
	}

	for (const auto key : empty_octants) {
		delete octants[key];
		octants.erase(key);
	}

	List<OctantKeyOld> to_delete;
	for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
		if (_octant_update(E.key)) {
			to_delete.push_back(E.key);
		}
	}

	while (to_delete.front()) {
		memdelete(octant_map[to_delete.front()->get()]);
		octant_map.erase(to_delete.front()->get());
		to_delete.pop_front();
	}

	_update_visibility();
	awaiting_update = false;
}

void HexMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"),
			&HexMap::set_collision_layer);
	ClassDB::bind_method(
			D_METHOD("get_collision_layer"), &HexMap::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"),
			&HexMap::set_collision_mask);
	ClassDB::bind_method(
			D_METHOD("get_collision_mask"), &HexMap::get_collision_mask);

	ClassDB::bind_method(
			D_METHOD("set_collision_mask_value", "layer_number", "value"),
			&HexMap::set_collision_mask_value);
	ClassDB::bind_method(D_METHOD("get_collision_mask_value", "layer_number"),
			&HexMap::get_collision_mask_value);

	ClassDB::bind_method(
			D_METHOD("set_collision_layer_value", "layer_number", "value"),
			&HexMap::set_collision_layer_value);
	ClassDB::bind_method(D_METHOD("get_collision_layer_value", "layer_number"),
			&HexMap::get_collision_layer_value);

	ClassDB::bind_method(D_METHOD("set_collision_priority", "priority"),
			&HexMap::set_collision_priority);
	ClassDB::bind_method(D_METHOD("get_collision_priority"),
			&HexMap::get_collision_priority);

	ClassDB::bind_method(D_METHOD("set_physics_material", "material"),
			&HexMap::set_physics_material);
	ClassDB::bind_method(
			D_METHOD("get_physics_material"), &HexMap::get_physics_material);

	ClassDB::bind_method(D_METHOD("set_bake_navigation", "bake_navigation"),
			&HexMap::set_bake_navigation);
	ClassDB::bind_method(
			D_METHOD("is_baking_navigation"), &HexMap::is_baking_navigation);

	ClassDB::bind_method(D_METHOD("set_navigation_map", "navigation_map"),
			&HexMap::set_navigation_map);
	ClassDB::bind_method(
			D_METHOD("get_navigation_map"), &HexMap::get_navigation_map);

	ClassDB::bind_method(D_METHOD("set_mesh_library", "mesh_library"),
			&HexMap::set_mesh_library);
	ClassDB::bind_method(
			D_METHOD("get_mesh_library"), &HexMap::get_mesh_library);

	ClassDB::bind_method(
			D_METHOD("set_cell_size", "size"), &HexMap::set_cell_size);
	ClassDB::bind_method(D_METHOD("get_cell_size"), &HexMap::get_cell_size);

	ClassDB::bind_method(
			D_METHOD("set_cell_scale", "scale"), &HexMap::set_cell_scale);
	ClassDB::bind_method(D_METHOD("get_cell_scale"), &HexMap::get_cell_scale);

	ClassDB::bind_method(
			D_METHOD("set_octant_size", "size"), &HexMap::set_octant_size);
	ClassDB::bind_method(
			D_METHOD("get_octant_size"), &HexMap::get_octant_size);

	ClassDB::bind_method(
			D_METHOD("set_cell_item", "position", "item", "orientation"),
			&HexMap::_set_cell_item,
			DEFVAL(0));
	ClassDB::bind_method(
			D_METHOD("get_cell_item", "position"), &HexMap::_get_cell_item);
	ClassDB::bind_method(D_METHOD("get_cell_item_orientation", "position"),
			&HexMap::_get_cell_item_orientation);

	ClassDB::bind_method(
			D_METHOD("local_region_to_map", "local_point_a", "local_point_b"),
			&HexMap::_local_region_to_map);

	ClassDB::bind_method(D_METHOD("local_to_cell_id", "local_position"),
			&HexMap::_local_to_cell_id);
	ClassDB::bind_method(D_METHOD("cell_id_to_local", "cell_id"),
			&HexMap::_cell_id_to_local);

	ClassDB::bind_method(
			D_METHOD("set_center_x", "enable"), &HexMap::set_center_x);
	ClassDB::bind_method(D_METHOD("get_center_x"), &HexMap::get_center_x);
	ClassDB::bind_method(
			D_METHOD("set_center_y", "enable"), &HexMap::set_center_y);
	ClassDB::bind_method(D_METHOD("get_center_y"), &HexMap::get_center_y);
	ClassDB::bind_method(
			D_METHOD("set_center_z", "enable"), &HexMap::set_center_z);
	ClassDB::bind_method(D_METHOD("get_center_z"), &HexMap::get_center_z);

	ClassDB::bind_method(D_METHOD("clear"), &HexMap::clear);

	ClassDB::bind_method(D_METHOD("get_used_cells"), &HexMap::get_used_cells);
	ClassDB::bind_method(D_METHOD("get_used_cells_by_item", "item"),
			&HexMap::get_used_cells_by_item);

	ClassDB::bind_method(D_METHOD("get_meshes"), &HexMap::get_meshes);
	ClassDB::bind_method(
			D_METHOD("get_bake_meshes"), &HexMap::get_bake_meshes);
	ClassDB::bind_method(D_METHOD("get_bake_mesh_instance", "idx"),
			&HexMap::get_bake_mesh_instance);

	ClassDB::bind_method(
			D_METHOD("clear_baked_meshes"), &HexMap::clear_baked_meshes);
	ClassDB::bind_method(D_METHOD("make_baked_meshes",
								 "gen_lightmap_uv",
								 "lightmap_uv_texel_size"),
			&HexMap::make_baked_meshes,
			DEFVAL(false),
			DEFVAL(0.1));

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT,
						 "mesh_library",
						 PROPERTY_HINT_RESOURCE_TYPE,
						 "MeshLibrary"),
			"set_mesh_library",
			"get_mesh_library");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT,
						 "physics_material",
						 PROPERTY_HINT_RESOURCE_TYPE,
						 "PhysicsMaterial"),
			"set_physics_material",
			"get_physics_material");
	ADD_GROUP("Cell", "cell_");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3,
						 "cell_size",
						 PROPERTY_HINT_NONE,
						 "suffix:m"),
			"set_cell_size",
			"get_cell_size");
	ADD_PROPERTY(PropertyInfo(Variant::INT,
						 "cell_octant_size",
						 PROPERTY_HINT_RANGE,
						 "1,1024,1"),
			"set_octant_size",
			"get_octant_size");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cell_center_x"),
			"set_center_x",
			"get_center_x");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cell_center_y"),
			"set_center_y",
			"get_center_y");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "cell_center_z"),
			"set_center_z",
			"get_center_z");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cell_scale"),
			"set_cell_scale",
			"get_cell_scale");
	ADD_GROUP("Collision", "collision_");
	ADD_PROPERTY(PropertyInfo(Variant::INT,
						 "collision_layer",
						 PROPERTY_HINT_LAYERS_3D_PHYSICS),
			"set_collision_layer",
			"get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT,
						 "collision_mask",
						 PROPERTY_HINT_LAYERS_3D_PHYSICS),
			"set_collision_mask",
			"get_collision_mask");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "collision_priority"),
			"set_collision_priority",
			"get_collision_priority");
	ADD_GROUP("Navigation", "");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "bake_navigation"),
			"set_bake_navigation",
			"is_baking_navigation");

	BIND_CONSTANT(INVALID_CELL_ITEM);

	ADD_SIGNAL(MethodInfo(
			"cell_size_changed", PropertyInfo(Variant::VECTOR3, "cell_size")));
	ADD_SIGNAL(MethodInfo("cell_shape_changed",
			PropertyInfo(Variant::INT,
					"cell_shape",
					PROPERTY_HINT_ENUM,
					"Square,Hexagon")));
	ADD_SIGNAL(MethodInfo("mesh_library_changed"));
	ADD_SIGNAL(MethodInfo(
			"cell_changed", PropertyInfo(Variant::VECTOR3I, "cell")));
}

void HexMap::set_cell_scale(float p_scale) {
	cell_scale = p_scale;
	_recreate_octant_data();
}

float HexMap::get_cell_scale() const { return cell_scale; }

Array HexMap::get_used_cells() const {
	Array a;
	a.resize(cell_map.size());
	int i = 0;
	for (const KeyValue<CellKey, Cell> &E : cell_map) {
		HexMapCellId cell_id(E.key);
		a[i++] = static_cast<Ref<HexMapCellIdRef>>(cell_id);
	}

	return a;
}

TypedArray<Vector3i> HexMap::get_used_cells_by_item(int p_item) const {
	Array a;
	for (const KeyValue<CellKey, Cell> &E : cell_map) {
		if ((int)E.value.item == p_item) {
			HexMapCellId cell_id(E.key);
			a.push_back(static_cast<Ref<HexMapCellIdRef>>(cell_id));
		}
	}

	return a;
}

Array HexMap::get_meshes() const {
	if (mesh_library.is_null()) {
		return Array();
	}

	Array meshes;

	for (const KeyValue<CellKey, Cell> &E : cell_map) {
		int id = E.value.item;
		Ref<Mesh> mesh = mesh_library->get_item_mesh(id);
		if (mesh.is_null()) {
			continue;
		}

		CellKey ik = E.key;

		Vector3 cellpos = cell_id_to_local(HexMapCellId(ik));

		Transform3D xform;

		xform.basis = TileOrientation(E.value.rot);

		xform.set_origin(cellpos);
		xform.basis.scale(Vector3(cell_scale, cell_scale, cell_scale));

		meshes.push_back(xform * mesh_library->get_item_mesh_transform(id));
		meshes.push_back(mesh);
	}

	return meshes;
}

Vector3 HexMap::_get_offset() const {
	return Vector3(cell_size.x * 0.5 * int(center_x),
			cell_size.y * 0.5 * int(center_y),
			cell_size.z * 0.5 * int(center_z));
}

void HexMap::clear_baked_meshes() {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	for (int i = 0; i < baked_meshes.size(); i++) {
		RS::get_singleton()->free_rid(baked_meshes[i].instance);
	}
	baked_meshes.clear();

	_recreate_octant_data();
}

void HexMap::make_baked_meshes(bool p_gen_lightmap_uv,
		float p_lightmap_uv_texel_size) {
	if (!mesh_library.is_valid()) {
		return;
	}

	// generate
	HashMap<OctantKeyOld, HashMap<Ref<Material>, Ref<SurfaceTool>>>
			surface_map;

	for (KeyValue<CellKey, Cell> &E : cell_map) {
		CellKey key = E.key;

		int item = E.value.item;
		Ref<Mesh> mesh = mesh_library->get_item_mesh(item);
		if (!mesh.is_valid()) {
			continue;
		}

		Vector3 cellpos = cell_id_to_local(key);
		Vector3 ofs = _get_offset();

		Transform3D xform;

		xform.basis = TileOrientation(E.value.rot);
		xform.set_origin(cellpos * cell_size + ofs);
		xform.basis.scale(Vector3(cell_scale, cell_scale, cell_scale));

		OctantKeyOld ok;
		ok.x = key.q / octant_size;
		ok.y = key.r / octant_size;
		ok.z = key.y / octant_size;

		if (!surface_map.has(ok)) {
			surface_map[ok] = HashMap<Ref<Material>, Ref<SurfaceTool>>();
		}

		HashMap<Ref<Material>, Ref<SurfaceTool>> &mat_map = surface_map[ok];

		for (int i = 0; i < mesh->get_surface_count(); i++) {
			if (mesh->_surface_get_primitive_type(i) !=
					Mesh::PRIMITIVE_TRIANGLES) {
				continue;
			}

			Ref<Material> surf_mat = mesh->surface_get_material(i);
			if (!mat_map.has(surf_mat)) {
				Ref<SurfaceTool> st;
				st.instantiate();
				st->begin(Mesh::PRIMITIVE_TRIANGLES);
				st->set_material(surf_mat);
				mat_map[surf_mat] = st;
			}

			mat_map[surf_mat]->append_from(mesh, i, xform);
		}
	}

	for (KeyValue<OctantKeyOld, HashMap<Ref<Material>, Ref<SurfaceTool>>> &E :
			surface_map) {
		Ref<ArrayMesh> mesh;
		mesh.instantiate();
		for (KeyValue<Ref<Material>, Ref<SurfaceTool>> &F : E.value) {
			F.value->commit(mesh);
		}

		BakedMesh bm;
		bm.mesh = mesh;
		bm.instance = RS::get_singleton()->instance_create();
		RS::get_singleton()->instance_set_base(
				bm.instance, bm.mesh->get_rid());
		RS::get_singleton()->instance_attach_object_instance_id(
				bm.instance, get_instance_id());
		if (is_inside_tree()) {
			RS::get_singleton()->instance_set_scenario(
					bm.instance, get_world_3d()->get_scenario());
			RS::get_singleton()->instance_set_transform(
					bm.instance, get_global_transform());
		}

		if (p_gen_lightmap_uv) {
			mesh->lightmap_unwrap(
					get_global_transform(), p_lightmap_uv_texel_size);
		}
		baked_meshes.push_back(bm);
	}

	_recreate_octant_data();
}

Array HexMap::get_bake_meshes() {
	if (!baked_meshes.size()) {
		make_baked_meshes(true);
	}

	Array arr;
	for (int i = 0; i < baked_meshes.size(); i++) {
		arr.push_back(baked_meshes[i].mesh);
		arr.push_back(Transform3D());
	}

	return arr;
}

RID HexMap::get_bake_mesh_instance(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, baked_meshes.size(), RID());
	return baked_meshes[p_idx].instance;
}

HexMap::HexMap() {
	set_notify_transform(true);

	// copied from SceneTree::get_debug_collision_material()
	collision_debug_mat.instantiate();
	collision_debug_mat->set_shading_mode(
			StandardMaterial3D::SHADING_MODE_UNSHADED);
	collision_debug_mat->set_transparency(
			StandardMaterial3D::TRANSPARENCY_ALPHA);
	collision_debug_mat->set_flag(
			StandardMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);
	collision_debug_mat->set_flag(
			StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	collision_debug_mat->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
	collision_debug_mat->set_albedo(
			ProjectSettings::get_singleton()->get_setting_with_override(
					"debug/shapes/collision/shape_color"));

#ifdef DEBUG_ENABLED
	NavigationServer3D::get_singleton()->connect("map_changed",
			callable_mp(this, &HexMap::_navigation_map_changed));
	NavigationServer3D::get_singleton()->connect("navigation_debug_changed",
			callable_mp(
					this, &HexMap::_update_navigation_debug_edge_connections));
#endif // DEBUG_ENABLED
}

#ifdef DEBUG_ENABLED
void HexMap::_update_navigation_debug_edge_connections() {
	if (bake_navigation) {
		for (const KeyValue<OctantKeyOld, OctantOld *> &E : octant_map) {
			_update_octant_navigation_debug_edge_connections_mesh(E.key);
		}
	}
}

void HexMap::_navigation_map_changed(RID p_map) {
	if (bake_navigation && is_inside_tree() &&
			p_map == get_world_3d()->get_navigation_map()) {
		_update_navigation_debug_edge_connections();
	}
}
#endif // DEBUG_ENABLED

HexMap::~HexMap() {
	clear();
#ifdef DEBUG_ENABLED
	NavigationServer3D::get_singleton()->disconnect("map_changed",
			callable_mp(this, &HexMap::_navigation_map_changed));
	NavigationServer3D::get_singleton()->disconnect("navigation_debug_changed",
			callable_mp(
					this, &HexMap::_update_navigation_debug_edge_connections));
#endif // DEBUG_ENABLED
}

#ifdef DEBUG_ENABLED
void HexMap::_update_octant_navigation_debug_edge_connections_mesh(
		const OctantKeyOld &p_key) {
	ERR_FAIL_COND(!octant_map.has(p_key));
	OctantOld &g = *octant_map[p_key];

	if (!NavigationServer3D::get_singleton()->get_debug_enabled()) {
		if (g.navigation_debug_edge_connections_instance.is_valid()) {
			RS::get_singleton()->instance_set_visible(
					g.navigation_debug_edge_connections_instance, false);
		}
		return;
	}

	if (!is_inside_tree()) {
		return;
	}

	if (!bake_navigation) {
		if (g.navigation_debug_edge_connections_instance.is_valid()) {
			RS::get_singleton()->instance_set_visible(
					g.navigation_debug_edge_connections_instance, false);
		}
		return;
	}

	if (!g.navigation_debug_edge_connections_instance.is_valid()) {
		g.navigation_debug_edge_connections_instance =
				RenderingServer::get_singleton()->instance_create();
	}

	if (!g.navigation_debug_edge_connections_mesh.is_valid()) {
		g.navigation_debug_edge_connections_mesh =
				Ref<ArrayMesh>(memnew(ArrayMesh));
	}

	g.navigation_debug_edge_connections_mesh->clear_surfaces();

	float edge_connection_margin =
			NavigationServer3D::get_singleton()
					->map_get_edge_connection_margin(
							get_world_3d()->get_navigation_map());
	float half_edge_connection_margin = edge_connection_margin * 0.5;

	PackedVector3Array vertex_array;

	for (KeyValue<CellKey, OctantOld::NavigationCell> &F :
			g.navigation_cell_ids) {
		if (cell_map.has(F.key) && F.value.region.is_valid()) {
			int connections_count =
					NavigationServer3D::get_singleton()
							->region_get_connections_count(F.value.region);
			if (connections_count == 0) {
				continue;
			}

			for (int i = 0; i < connections_count; i++) {
				Vector3 connection_pathway_start =
						NavigationServer3D::get_singleton()
								->region_get_connection_pathway_start(
										F.value.region, i);
				Vector3 connection_pathway_end =
						NavigationServer3D::get_singleton()
								->region_get_connection_pathway_end(
										F.value.region, i);

				Vector3 direction_start_end =
						connection_pathway_start.direction_to(
								connection_pathway_end);
				Vector3 direction_end_start =
						connection_pathway_end.direction_to(
								connection_pathway_start);

				Vector3 start_right_dir =
						direction_start_end.cross(Vector3(0, 1, 0));
				Vector3 start_left_dir = -start_right_dir;

				Vector3 end_right_dir =
						direction_end_start.cross(Vector3(0, 1, 0));
				Vector3 end_left_dir = -end_right_dir;

				Vector3 left_start_pos = connection_pathway_start +
						(start_left_dir * half_edge_connection_margin);
				Vector3 right_start_pos = connection_pathway_start +
						(start_right_dir * half_edge_connection_margin);
				Vector3 left_end_pos = connection_pathway_end +
						(end_right_dir * half_edge_connection_margin);
				Vector3 right_end_pos = connection_pathway_end +
						(end_left_dir * half_edge_connection_margin);

				vertex_array.push_back(right_end_pos);
				vertex_array.push_back(left_start_pos);
				vertex_array.push_back(right_start_pos);

				vertex_array.push_back(left_end_pos);
				vertex_array.push_back(right_end_pos);
				vertex_array.push_back(right_start_pos);
			}
		}
	}

	if (vertex_array.size() == 0) {
		return;
	}

	// XXX need method to get the navigation edge connections material
	//
	// Ref<StandardMaterial3D> edge_connections_material =
	// 		NavigationServer3D::get_singleton()->
	// 				->get_debug_navigation_edge_connections_material();

	Array mesh_array;
	mesh_array.resize(Mesh::ARRAY_MAX);
	mesh_array[Mesh::ARRAY_VERTEX] = vertex_array;

	g.navigation_debug_edge_connections_mesh->add_surface_from_arrays(
			Mesh::PRIMITIVE_TRIANGLES, mesh_array);
	// XXX missing edge connect material
	// g.navigation_debug_edge_connections_mesh->surface_set_material(
	// 		0, edge_connections_material);

	RS::get_singleton()->instance_set_base(
			g.navigation_debug_edge_connections_instance,
			g.navigation_debug_edge_connections_mesh->get_rid());
	RS::get_singleton()->instance_set_visible(
			g.navigation_debug_edge_connections_instance,
			is_visible_in_tree());
	if (is_inside_tree()) {
		RS::get_singleton()->instance_set_scenario(
				g.navigation_debug_edge_connections_instance,
				get_world_3d()->get_scenario());
	}

	bool enable_edge_connections =
			NavigationServer3D::get_singleton()->get_debug_enabled();
	if (!enable_edge_connections) {
		RS::get_singleton()->instance_set_visible(
				g.navigation_debug_edge_connections_instance, false);
	}
}
#endif // DEBUG_ENABLED
