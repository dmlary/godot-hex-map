#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/variant/string.hpp>

#include "cell_id.h"
#include "iter.h"

// HexMap cell iterator using oddr coordinates
struct HexMapIterCube : public HexMapIter {
public:
    // This requires unit position values to determine fall-off of left or
    // right
    HexMapIterCube(Vector3 a, Vector3 b);
    HexMapIterCube(const HexMapIterCube &) = default;

    friend bool operator==(const HexMapIterCube &a, const HexMapIterCube &b) {
        return a.pos == b.pos;
    };
    friend bool operator!=(const HexMapIterCube &a, const HexMapIterCube &b) {
        return a.pos != b.pos;
    };

    HexMapCellId operator*() const;
    HexMapIterCube &operator++();
    HexMapIterCube begin() const;
    HexMapIterCube end() const;

    // gdscript integration methods
    operator godot::String() const;
    bool _iter_init();
    bool _iter_next();
    HexMapCellId _iter_get() const;
    HexMapIter *clone() const;

    // added for testing
    friend std::ostream &operator<<(std::ostream &os,
            const HexMapIterCube &value);

private:
    Vector3i min;
    Vector3i max;
    Vector3i pos;
    int min_x_shift[2] = { 0, 0 };
    int max_x_shift[2] = { 0, 0 };
    int max_x;
};
