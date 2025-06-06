#pragma once

#include <cassert>
#include <climits>
#include <godot_cpp/classes/mesh_library.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/templates/hash_map.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "core/tile_orientation.h"
#include "int_node/int_node.h"
#include "tiled_node/tiled_node.h"

using namespace godot;

class HexMapAutoTiledNodeEditorPlugin;

class HexMapAutoTiledNode : public Node3D {
    using HexMapAutoTiled = HexMapAutoTiledNode;
    GDCLASS(HexMapAutoTiled, Node3D);

    friend HexMapAutoTiledNodeEditorPlugin;

public:
    // forward declare the gdscript wrapper class for the following Rule class
    class HexMapTileRule;

    /// class defining a single rule that matches some pattern of tile types
    /// in the IntNode, and if it matches, draws a specific tile & orientation.
    class Rule {
        friend HexMapAutoTiledNode;

        /// number of cells contained in the rule pattern
        static const unsigned PATTERN_CELLS = 35;

        /// rule id used to denote that the rule does not have an id
        static const uint16_t ID_NOT_SET = USHRT_MAX;

    public:
        /// rule cell states
        enum CellState : uint8_t {
            /// cell is ignored when matching this rule
            RULE_CELL_STATE_DISABLED = 0,
            /// cell must be empty when matcheng
            RULE_CELL_STATE_EMPTY,
            /// cell must not be empty when matcheng
            RULE_CELL_STATE_NOT_EMPTY,
            /// cell must have type specified in type field
            RULE_CELL_STATE_TYPE,
            /// cell must not have type specified in type field
            RULE_CELL_STATE_NOT_TYPE,
            /// invalid cell offset, used in get_cell()
            RULE_CELL_INVALID_OFFSET = 0xff,
        };

        /// internal state for a cell in the rule pattern
        struct Cell {
            /// state for this cell in the pattern
            CellState state = RULE_CELL_STATE_DISABLED;

            /// type for this cell
            uint16_t type = 0;

            /// return this class as a Dictionary
            Dictionary to_dict() const;

            /// get an instance of Cell from a Dictonary returned by to_dict()
            static Cell from_dict(const Dictionary);
        };

        /// get a reference to a copy of this value; used for packing into a
        /// Variant type
        Ref<HexMapTileRule> to_ref() const;

        /// get the details for a specific cell in the rule pattern
        Cell get_cell(HexMapCellId cell_id) const;

        /// clear a cell in the pattern so that it is ignored when matching
        void clear_cell(HexMapCellId offset);

        /// require a cell to be a specific type when matching
        void
        set_cell_type(HexMapCellId offset, unsigned type, bool invert = false);

        /// require a cell be empty when matching
        void set_cell_empty(HexMapCellId offset, bool invert = false);

        /// check if the rule matches the provided cell values
        /// @param[in] [cell_values] values for the surrounding cells
        /// @param[out] [orientation] first orientation found where rule
        ///     matches
        /// @return true if rule matches
        inline bool match(const int32_t cell_values[PATTERN_CELLS],
                HexMapTileOrientation &orientation) const;

    private:
        // clang-format off

        /// offset of each cell in the rule pattern from the origin cell
        // NOTE: the order of these is important; we depend on it in
        // update_internal(), and in match().
        //
        // We intentionally order the radius = 0 (q = 0, r = 0) cells first
        // because rotating the pattern won't matter if one of those cells does
        // not match
        //
        // After that we have the radius = 1 cells, starting at the center and
        // moving up and down from the center layer.  We do this to reduce the
        // number of cells we need to fetch to match the rule.  If we don't
        // have anything set for the radius=2 cells, we can avoid fetching them.
        //
        // Finally, we have the radius=2, y=0 cells.
        static constexpr HexMapCellId CellOffsets[PATTERN_CELLS] = {
            // radius = 0, whole column, y = 0, 1, -1, 2, -2
            HexMapCellId( 0,  0,  0),   // 0: origin
            HexMapCellId( 0,  0,  1),
            HexMapCellId( 0,  0, -1),
            HexMapCellId( 0,  0,  2),
            HexMapCellId( 0,  0, -2),

            // radius = 1, starting at southwest corner, counter-clockwise
            HexMapCellId(-1,  1, 0),    // 5
            HexMapCellId( 0,  1, 0),
            HexMapCellId( 1,  0, 0),
            HexMapCellId( 1, -1, 0),
            HexMapCellId( 0, -1, 0),
            HexMapCellId(-1,  0, 0),    // 10

            // radius = 1, y = 1
            HexMapCellId(-1,  1, 1),    // 11
            HexMapCellId( 0,  1, 1),
            HexMapCellId( 1,  0, 1),
            HexMapCellId( 1, -1, 1),
            HexMapCellId( 0, -1, 1),
            HexMapCellId(-1,  0, 1),    // 16

            // radius = 1, y = -1
            HexMapCellId(-1,  1, -1),   // 17
            HexMapCellId( 0,  1, -1),
            HexMapCellId( 1,  0, -1),
            HexMapCellId( 1, -1, -1),
            HexMapCellId( 0, -1, -1),
            HexMapCellId(-1,  0, -1),   // 22

            // radius = 2, starting at southwest corner, counter-clockwise
            HexMapCellId(-2,  2, 0),    // 23
            HexMapCellId(-1,  2, 0),
            HexMapCellId( 0,  2, 0),
            HexMapCellId( 1,  1, 0),
            HexMapCellId( 2,  0, 0),    // 27
            HexMapCellId( 2, -1, 0),
            HexMapCellId( 2, -2, 0),
            HexMapCellId( 1, -2, 0),
            HexMapCellId( 0, -2, 0),    // 31
            HexMapCellId(-1, -1, 0),
            HexMapCellId(-2,  0, 0),
            HexMapCellId(-2,  1, 0),    // 34
        };

        /// Used to rotate the cell pattern based on TileOrientation, each
        /// array is a specific tile orientation, and the values within the
        /// array are the pattern index for each cell we're matching against.
        /// So effectively pattern[PatternIndex[rotation][i]] == cells[i] for
        /// any upright rotation value.
        static_assert(HexMapTileOrientation::Upright0 == 0);
        static_assert(HexMapTileOrientation::Upright60 == 1);
        static_assert(HexMapTileOrientation::Upright120 == 2);
        static_assert(HexMapTileOrientation::Upright180 == 3);
        static_assert(HexMapTileOrientation::Upright240 == 4);
        static_assert(HexMapTileOrientation::Upright300 == 5);
        static constexpr uint8_t PatternIndex[6][PATTERN_CELLS] = {
            // Upright0
            { 0, 1, 2, 3, 4,            // q = r = 0
              5, 6, 7, 8, 9, 10,        // radius = 1, y = 0
              11, 12, 13, 14, 15, 16,   // radius = 1, y = 1
              17, 18, 19, 20, 21, 22,   // radius = 1, y = -1
              23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, // r=2, y=0
            },
            // Upright60
            { 0, 1, 2, 3, 4,            // q = r = 0
              10, 5, 6, 7, 8, 9,        // radius = 1, y = 0
              16, 11, 12, 13, 14, 15,   // radius = 1, y = 1
              22, 17, 18, 19, 20, 21,   // radius = 1, y = -1
              33, 34, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,   // r=2, y=0
            },
            // Upright120
            { 0, 1, 2, 3, 4,            // q = r = 0
              9, 10, 5, 6, 7, 8,        // radius = 1, y = 0
              15, 16, 11, 12, 13, 14,   // radius = 1, y = 1
              21, 22, 17, 18, 19, 20,   // radius = 1, y = -1
              31, 32, 33, 34, 23, 24, 25, 26, 27, 28, 29, 30,   // r=2, y=0
            },
            // Upright180
            { 0, 1, 2, 3, 4,            // q = r = 0
              8, 9, 10, 5, 6, 7,        // radius = 1, y = 0
              14, 15, 16, 11, 12, 13,   // radius = 1, y = 1
              20, 21, 22, 17, 18, 19,   // radius = 1, y = -1
              29, 30, 31, 32, 33, 34, 23, 24, 25, 26, 27, 28,   // r=2, y=0
            },
            // Upright240
            { 0, 1, 2, 3, 4,            // q = r = 0
              7, 8, 9, 10, 5, 6,        // radius = 1, y = 0
              13, 14, 15, 16, 11, 12,   // radius = 1, y = 1
              19, 20, 21, 22, 17, 18,   // radius = 1, y = -1
              27, 28, 29, 30, 31, 32, 33, 34, 23, 24, 25, 26,   // r=2, y=0
            },
            // Upright300
            { 0, 1, 2, 3, 4,            // q = r = 0
              6, 7, 8, 9, 10, 5,        // radius = 1, y = 0
              12, 13, 14, 15, 16, 11,   // radius = 1, y = 1
              18, 19, 20, 21, 22, 17,   // radius = 1, y = -1
              25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 23, 24,   // r=2, y=0
            },
        };
        // clang-format on

        /// get the pattern index for a given cell offset
        /// @returns [int] index, or -1 for invalid offset
        inline int get_pattern_index(HexMapCellId offset) const;

        /// update internal state (cell_mask, search_pad, etc) when the pattern
        /// is changed.
        void update_internal();

        /// internal rule id, used for ordering
        uint16_t id = ID_NOT_SET;

        /// tile to set when rule matches
        int16_t tile = -1;

        /// bit to denote whether the rule is enabled
        bool enabled = true;

        /// cell pattern to match against
        /// @see CellOffsets
        Cell pattern[PATTERN_CELLS];

        /// bitmask of neighbor cells that must be checked to match this rule,
        /// including all rotations
        uint64_t cell_mask = 0;

        /// highest set index in pattern
        int pattern_index_max = -1;

        /// How to pad the cell id search space to be able to match this rule
        ///
        /// apply_rules() only considers those cells with a value set.  To
        /// support rules with an empty cell at the origin cell in the pattern,
        /// we need to expand the cells processed by apply_rules().  We want
        /// to limit the number of additional cells we need to process, so we
        /// find the nearest neighbor cell that must be set for this rule to
        /// match.
        ///
        /// We then apply this pad to every defined cell id to make it possible
        /// to match rules with an empty cell.
        struct {
            /// change in y layer to the nearest defined cell
            int8_t delta_y : 4;
            /// radius of nearest defined cell; 0 means in the center column,
            /// 1 is inner-most ring, 2 means outer ring with radius = 2.  This
            /// value will be set to -1 when the origin cell is set in the
            /// pattern.
            int8_t radius : 4;
        } search_pad = { -1, -1 };
    };

    /// GDScript wrapper for Rule class
    class HexMapTileRule : public RefCounted {
        GDCLASS(HexMapTileRule, RefCounted);
        friend HexMapAutoTiledNode;

    public:
        // property getter/setters
        int get_tile() const;
        void set_tile(int value);
        unsigned get_id() const;
        void set_id(unsigned value);
        bool get_enabled() const;
        void set_enabled(bool value);

        void clear_cell(const Ref<hex_bind::HexMapCellId> &);
        void set_cell_type(const Ref<hex_bind::HexMapCellId> &,
                unsigned type,
                bool not_);
        void set_cell_empty(const Ref<hex_bind::HexMapCellId> &, bool not_);
        Variant get_cell(const Ref<hex_bind::HexMapCellId> &) const;
        Array get_cells() const;
        Ref<HexMapAutoTiledNode::HexMapTileRule> duplicate() const;

        HexMapTileRule() {};
        HexMapTileRule(const Rule &rule) : inner(rule) {};

        /// actual Rule value
        Rule inner;

        String _to_string() const;

    protected:
        static void _bind_methods();
    };

    HexMapAutoTiledNode();
    ~HexMapAutoTiledNode();

    // property setters & getters
    void set_mesh_library(const Ref<MeshLibrary> &);
    Ref<MeshLibrary> get_mesh_library() const;
    void set_mesh_origin(HexMapTiledNode::MeshOrigin);
    HexMapTiledNode::MeshOrigin get_mesh_origin() const;
    Vector3 get_mesh_origin_vec() const;
    Dictionary get_rules() const;
    Variant get_rule(uint16_t id) const;
    Array get_rules_order() const;
    void set_rules_order(Array);

    /// add a new rule; id of new rule will be returned
    /// @param [rule] Rule to be added
    ///
    /// If rule.id is ID_NOT_SET, then the next available rule id will be used.
    /// If rule.id has any other value, this method will use the supplied id.
    /// If the rule id is already in use, ID_NOT_SET will be returned.
    unsigned add_rule(const Rule &);
    unsigned add_rule(const Ref<HexMapTileRule> &);
    /// update an existing rule
    void update_rule(const Rule &);
    void update_rule(const Ref<HexMapTileRule> &);
    /// delete a rule by id
    void delete_rule(uint16_t);

    /// get the HexMapTiledNode that contains the results of the rules
    /// @return HexMapTiledNode
    HexMapTiledNode *get_tiled_node() const;

    // signal callbacks
    void on_int_node_hex_space_changed();

protected:
    static void _bind_methods();
    void _notification(int p_what);

    // save/load support
    void _get_property_list(List<PropertyInfo> *p_list) const;
    bool _get(const StringName &p_name, Variant &r_ret) const;
    bool _set(const StringName &p_name, const Variant &p_value);

private:
    void on_int_node_cells_changed(Array);
    void apply_rules();

    Ref<MeshLibrary> mesh_library;
    HexMapTiledNode *tiled_node;
    HexMapIntNode *int_node;

    /// all defined rules
    HashMap<uint16_t, Rule> rules;

    /// order in which the rules should be applied
    Vector<int> rules_order;
};
