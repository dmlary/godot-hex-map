#pragma once

#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/variant/color.hpp>

#include "core/cell_id.h"
#include "core/hex_map_node.h"

using namespace godot;

class HexMapIntNodeEditorPlugin;

class HexMapIntNode : public HexMapNode {
    using HexMapInt = HexMapIntNode;
    GDCLASS(HexMapInt, HexMapNode);

    friend HexMapIntNodeEditorPlugin;

public:
    struct CellType {
        String name;
        Color color;
    };
    using TypeMap = HashMap<unsigned, CellType>;

    /// define a new cell type to be used in the map
    unsigned _add_cell_type(const String, const Color);

    /// remove a tile type
    void remove_cell_type(unsigned id);

    /// update an existing cell type
    bool update_cell_type(unsigned id, const String, const Color);

    /// get a const reference to the types set for this hexmap node
    inline const TypeMap &get_cell_types() const { return cell_types; };

    /// return an Array of Dictionary with value, name, color keys
    /// XXX dictionary may not be possible as there's no simple "insert"/"set"
    /// function
    Array _get_cell_types() const;

    void set_cell(const HexMapCellId &,
            int tile,
            HexMapTileOrientation orientation = 0) override;
    CellInfo get_cell(const HexMapCellId &) const override;
    Array get_cell_ids_v() const override;
    void set_cell_visibility(const HexMapCellId &cell_id,
            bool visibility) override {};

protected:
    // void _notification(int p_what);
    static void _bind_methods();

private:
    unsigned max_type;
    TypeMap cell_types;
    HashMap<HexMapCellId::Key, int> cell_map;
};
