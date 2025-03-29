#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/variant/string.hpp>

#include "cell_id.h"
#include "iter.h"

// HexMap cell iterator using axial coordinates to define a volume to iterate
struct HexMapIterAxial : public HexMapIter {
public:
    HexMapIterAxial(const HexMapCellId center,
            unsigned int radius,
            const HexMapPlanes &planes = HexMapPlanes::All);
    HexMapIterAxial(const HexMapIterAxial &) = default;

    friend bool operator==(const HexMapIterAxial &a,
            const HexMapIterAxial &b) {
        return a.cell == b.cell;
    };
    friend bool operator!=(const HexMapIterAxial &a,
            const HexMapIterAxial &b) {
        return a.cell != b.cell;
    };

    HexMapCellId operator*() const { return cell; }
    const HexMapCellId &operator->() { return cell; }
    HexMapIterAxial &operator++();
    HexMapIterAxial begin();
    HexMapIterAxial end();

    // gdscript integration methods
    operator godot::String() const;
    bool _iter_init();
    bool _iter_next();
    HexMapCellId _iter_get() const;
    HexMapIter *clone() const;

    // for doctests
    friend std::ostream &operator<<(std::ostream &os,
            const HexMapIterAxial &value);

    friend HexMapIterRadial;

private:
    void advance();
    void advance_until_valid();

    int y_min, y_max;
    int q_min, q_max;
    int r_min, r_max;
    int s_min, s_max;
    HexMapCellId center, cell;
};
