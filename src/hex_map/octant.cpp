#include "octant.h"
#include "godot_cpp/classes/mesh_library.hpp"
#include "godot_cpp/classes/physics_server3d.hpp"
#include "godot_cpp/classes/rendering_server.hpp"
#include "godot_cpp/classes/scene_tree.hpp"
#include "godot_cpp/classes/shape3d.hpp"
#include "godot_cpp/classes/world3d.hpp"
#include "godot_cpp/templates/pair.hpp"
#include "godot_cpp/templates/vector.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "hex_map.h"
#include "profiling.h"
#include <cassert>

void HexMapOctant::build_collision_debug_mesh() {
	ERR_FAIL_COND(!collision_debug_mesh.is_valid());

	RenderingServer *rs = RenderingServer::get_singleton();
	Ref<MeshLibrary> &mesh_library = hex_map.mesh_library;

	rs->mesh_clear(collision_debug_mesh);

	PackedVector3Array vertex_array;

	for (const CellKey &cell_key : cells) {
		auto *cell = hex_map.cell_map.getptr(cell_key);
		ERR_CONTINUE_MSG(cell == nullptr, "nonexistent cell in octant");

		const Array shapes = hex_map.mesh_library->get_item_shapes(cell->item);
		if (shapes.is_empty()) {
			continue;
		}

		// figure out the transform for the center of the cell with rotation
		Transform3D cell_transform;
		cell_transform.set_origin(hex_map.cell_id_to_local(cell_key));
		cell_transform.basis = cell->get_basis();

		// go through all the shapes for the cell mesh and add the faces to our
		// vertex array
		for (int i = 0; i < shapes.size(); i += 2) {
			Shape3D *shape = Object::cast_to<Shape3D>(shapes[i]);
			Transform3D shape_transform = shapes[i + 1];
			Transform3D local_transform = cell_transform * shape_transform;
			for (const Vector3 &v : shape->get_debug_mesh()->get_faces()) {
				vertex_array.append(local_transform.xform(v));
			}
		}
	}

	if (vertex_array.is_empty()) {
		return;
	}
	Array surface_arrays;
	surface_arrays.resize(RS::ARRAY_MAX);
	surface_arrays[RS::ARRAY_VERTEX] = vertex_array;
	rs->mesh_add_surface_from_arrays(
			collision_debug_mesh, RS::PRIMITIVE_LINES, surface_arrays);
}

void HexMapOctant::build_meshes() {
	UtilityFunctions::print("building meshes");
	RenderingServer *rs = RenderingServer::get_singleton();
	Ref<MeshLibrary> &mesh_library = hex_map.mesh_library;

	for (const auto &multimesh : multimeshes) {
		rs->free_rid(multimesh.multimesh_instance);
		rs->free_rid(multimesh.multimesh);
	}
	multimeshes.clear();

	// break the cells out into mesh RID & transform
	HashMap<RID, Vector<Transform3D>> mesh_cells;
	for (const CellKey &cell_key : cells) {
		auto *cell = hex_map.cell_map.getptr(cell_key);
		ERR_CONTINUE_MSG(cell == nullptr, "nonexistent cell in octant");

		Ref<Mesh> mesh = mesh_library->get_item_mesh(cell->item);
		if (!mesh.is_valid()) {
			continue;
		}
		RID mesh_rid = mesh->get_rid();

		Vector<Transform3D> *cells = mesh_cells.getptr(mesh_rid);
		if (!mesh_cells.has(mesh_rid)) {
			auto iter = mesh_cells.insert(mesh_rid, Vector<Transform3D>());
			cells = &iter->value;
		}

		Transform3D cell_transform;
		cell_transform.set_origin(hex_map.cell_id_to_local(cell_key));
		cell_transform.basis = cell->get_basis();

		cells->push_back(cell_transform *
				mesh_library->get_item_mesh_transform(cell->item));
	}

	for (const auto &pair : mesh_cells) {
		// create the multimesh
		RID multimesh = rs->multimesh_create();
		rs->multimesh_set_mesh(multimesh, pair.key);
		rs->multimesh_allocate_data(
				multimesh, pair.value.size(), RS::MULTIMESH_TRANSFORM_3D);

		// copy all the transforms into it
		const Vector<Transform3D> &transforms = pair.value;
		for (int i = 0; i < transforms.size(); i++) {
			rs->multimesh_instance_set_transform(multimesh, i, transforms[i]);
		}

		// create an instance of the multimesh
		RID instance = rs->instance_create2(
				multimesh, hex_map.get_world_3d()->get_scenario());

		multimeshes.push_back(MultiMesh{ multimesh, instance });
	}
	UtilityFunctions::print("done building meshes");
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

void HexMapOctant::enter_world() {
	ERR_FAIL_COND(!hex_map.is_inside_tree());

	RenderingServer *rs = RenderingServer::get_singleton();
	SceneTree *st = hex_map.get_tree();

	// if debugging collisions, create collision debug mesh and show it
	if (st->is_debugging_collisions_hint()) {
		assert(!collision_debug_mesh.is_valid() && "mesh should not exist");
		assert(!collision_debug_mesh_instance.is_valid() &&
				"mesh instance should not exist");

		collision_debug_mesh = rs->mesh_create();
		rs->mesh_surface_set_material(collision_debug_mesh,
				0,
				hex_map.collision_debug_mat->get_rid());
		{
			auto profiler = profiling_begin("build_collision_debug_mesh()");
			build_collision_debug_mesh();
		}
		collision_debug_mesh_instance = rs->instance_create2(
				collision_debug_mesh, hex_map.get_world_3d()->get_scenario());
	}

	update_transforms();
}

void HexMapOctant::exit_world() {
	RenderingServer *rs = RenderingServer::get_singleton();

	for (const MultiMesh &multimesh : multimeshes) {
		rs->free_rid(multimesh.multimesh_instance);
		rs->free_rid(multimesh.multimesh);
	}
	multimeshes.clear();

	rs->free_rid(collision_debug_mesh_instance);
	collision_debug_mesh_instance = RID();

	rs->free_rid(collision_debug_mesh);
	collision_debug_mesh = RID();
}

void HexMapOctant::update_transforms() {
	// XXX update the transform
}

void HexMapOctant::update() {
	auto profiler = profiling_begin("build_meshes()");
	build_meshes();
}

void HexMapOctant::add_cell(const CellKey cell_key) {
	cells.insert(cell_key);
	dirty = true;
}

void HexMapOctant::remove_cell(const CellKey cell_key) {
	cells.erase(cell_key);
	dirty = true;
}

HexMapOctant::HexMapOctant(HexMap &hex_map) : hex_map(hex_map) {
	auto prof = profiling_begin("HexMapOctant");
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();

	physics_body = ps->body_create();
	ps->body_set_mode(physics_body, PhysicsServer3D::BODY_MODE_STATIC);
	ps->body_attach_object_instance_id(
			physics_body, hex_map.get_instance_id());
	update_collision_properties();
	update_physics_params();
}

HexMapOctant::~HexMapOctant() {
	auto prof = profiling_begin("~HexMapOctant");
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	ps->free_rid(physics_body);
}
