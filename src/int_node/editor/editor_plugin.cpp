#include "core/space.h"
#include "godot_cpp/classes/standard_material3d.hpp"
#include "godot_cpp/variant/callable_method_pointer.hpp"
#include "tiled_node/tiled_node.h"
#ifdef TOOLS_ENABLED

#include <godot_cpp/classes/packed_scene.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "editor_plugin.h"

bool HexMapIntNodeEditorPlugin::_handles(Object *p_object) const {
    return p_object->is_class("HexMapInt");
}

void HexMapIntNodeEditorPlugin::_edit(Object *p_object) {
    HexMapNodeEditorPlugin::_edit(p_object);

    if (hex_map) {
        // get IntNode properly casted
        int_node = Object::cast_to<HexMapIntNode>(p_object);

        // XXX Move this loading to the constructor when gui stablizes
        // load the bottom panel scene and add it to the editor
        Ref<PackedScene> panel_scene = ResourceLoader::get_singleton()->load(
                "res://addons/hexmap/gui/"
                "int_node_editor_bottom_panel.tscn");
        bottom_panel = (Control *)panel_scene->instantiate();
        bottom_panel->set("int_map", int_node);

        type_selector = bottom_panel->get_node_or_null("%TypeSelector");
        if (type_selector) {
            type_selector->connect("types_changed",
                    callable_mp(this,
                            &HexMapIntNodeEditorPlugin::update_mesh_library));
            type_selector->connect("selection_changed",
                    callable_mp(this,
                            static_cast<void (HexMapIntNodeEditorPlugin::*)(
                                    int)>(&HexMapIntNodeEditorPlugin::
                                            rebuild_cursor)));
        }
        add_control_to_bottom_panel(bottom_panel, "HexMapInt");
        make_bottom_panel_item_visible(bottom_panel);

        // create a TiledNode to visualize the contents of the IntNode
        tile_node = memnew(HexMapTiledNode);
        tile_node->set_space(int_node->get_space());
        // the cell meshes we create in update_mesh_library() are centered on
        // origin, so set the TileNode to center the mesh in the cell.
        tile_node->set_center_y(true);
        update_mesh_library();

        // XXX editor_cursor is getting hung up on center-y, when set to false
        // on the IntNode editor_cursor doesn't line up with the real grid.

        // populate the tiled node with our cells
        for (const auto &iter : int_node->cell_map) {
            tile_node->set_cell(iter.key, iter.value);
        }
        add_child(tile_node);

        editor_cursor->set_cells_visibility_callback(callable_mp(
                tile_node, &HexMapTiledNode::set_cells_visibility_callback));

        int_node->connect("editor_plugin_cell_changed",
                callable_mp(
                        this, &HexMapIntNodeEditorPlugin::update_tiled_node));

    } else {
        int_node->disconnect("editor_plugin_cell_changed",
                callable_mp(
                        this, &HexMapIntNodeEditorPlugin::update_tiled_node));
        int_node = nullptr;
        if (tile_node != nullptr) {
            remove_child(tile_node);
            memfree(tile_node);
            tile_node = nullptr;
        }

        type_selector = nullptr;
        if (bottom_panel != nullptr) {
            remove_control_from_bottom_panel(bottom_panel);
            memfree(bottom_panel);
            bottom_panel = nullptr;
        }

        set_process(false);
        return;
    }
}

void HexMapIntNodeEditorPlugin::update_mesh_library() {
    mesh_library.instantiate();
    const HexMapSpace &space = int_node->get_space();

    for (const auto &iter : int_node->get_cell_types()) {
        Ref<StandardMaterial3D> material;
        material.instantiate();
        material->set_albedo(iter.value.color);
        if (iter.value.color.a < 1.0) {
            material->set_transparency(StandardMaterial3D::TRANSPARENCY_ALPHA);
        }
        //material->set_shading_mode(
        //        StandardMaterial3D::SHADING_MODE_PER_VERTEX);
        // material->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
        Ref<StandardMaterial3D> line_material;
        line_material.instantiate();
        line_material->set_albedo(iter.value.color.inverted());
        line_material->set_shading_mode(
                StandardMaterial3D::SHADING_MODE_UNSHADED);
        line_material->set_transparency(
                StandardMaterial3D::TRANSPARENCY_ALPHA);
        line_material->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);

        Ref<Mesh> mesh = space.build_cell_mesh();
        mesh->surface_set_material(0, material);
        mesh->surface_set_material(1, line_material);
        mesh_library->create_item(iter.key);
        mesh_library->set_item_name(iter.key, iter.value.name);
        mesh_library->set_item_mesh(iter.key, mesh);
    }

    if (tile_node) {
        tile_node->set_mesh_library(mesh_library);
    }

    if (editor_cursor) {
        editor_cursor->set_mesh_library(mesh_library);
        rebuild_cursor();
    }
}

void HexMapIntNodeEditorPlugin::update_tiled_node(Vector3i cell_id_v,
        int type) {
    ERR_FAIL_COND_MSG(tile_node == nullptr, "tiled node not allocated");
    tile_node->set_cell(cell_id_v, type);
}

void HexMapIntNodeEditorPlugin::rebuild_cursor(int type) {
    ERR_FAIL_COND_MSG(bottom_panel == nullptr, "bottom panel not allocated");
    editor_cursor->clear_tiles();
    if (type >= 0) {
        editor_cursor->set_tile(HexMapCellId(), type);
    }
}

void HexMapIntNodeEditorPlugin::rebuild_cursor() {
    ERR_FAIL_COND_MSG(type_selector == nullptr, "type_selector not set");
    Variant type = type_selector->get("selected_type");
    rebuild_cursor(type);
}

#endif // TOOLS_ENABLED
