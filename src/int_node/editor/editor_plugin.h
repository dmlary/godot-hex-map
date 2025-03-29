#include "tiled_node/tiled_node.h"
#ifdef TOOLS_ENABLED

#pragma once

#include <godot_cpp/classes/control.hpp>
#include <godot_cpp/classes/editor_plugin.hpp>
#include <godot_cpp/classes/mesh_library.hpp>

#include "../int_node.h"
#include "core/editor/editor_plugin.h"

using namespace godot;

// XXX move palette to bottom panel for HexGrid
class HexMapIntNodeEditorPlugin : public HexMapNodeEditorPlugin {
    GDCLASS(HexMapIntNodeEditorPlugin, HexMapNodeEditorPlugin);

public:
    virtual bool _handles(Object *object) const override;
    virtual void _edit(Object *p_object) override;

    HexMapIntNodeEditorPlugin() {};
    ~HexMapIntNodeEditorPlugin() {};

protected:
    // void _notification(int p_what);
    static void _bind_methods() {};

private:
    /// HexMapIntNode being edited
    HexMapIntNode *int_node = nullptr;

    /// HexMapTiledNode used to render the contents of `int_node`
    HexMapTiledNode *tile_node = nullptr;

    /// Generated MeshLibrary used in `tile_node`, contains hex cells with
    /// colors defined by `int_node->get_cell_types()`.
    Ref<MeshLibrary> mesh_library;

    /// gui bottom panel holding all user controls for this plugin
    Control *bottom_panel = nullptr;

    /// gui cell type selector widget
    Node *type_selector = nullptr;

    /// update the `MeshLibrary` used in the `HexMapTiledNode` editor display
    void update_mesh_library();

    /// update the `HexMapTiledNode` based on a cell change
    void update_tiled_node(Vector3i cell_id_v, int type);

    /// rebuild the cursor based on the gui state
    void rebuild_cursor() override;

    /// rebuild the cursor using the specific selected tile type
    void rebuild_cursor(int type);
};

#endif
