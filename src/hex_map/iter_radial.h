#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/variant/string.hpp>

#include "cell_id.h"
#include "hex_map.h"
#include "hex_map/iter.h"
#include "iter_axial.h"

// HexMap cell iterator using axial coordinates to define a volume to iterate
struct HexMapIterRadial : public HexMapIter {
public:
    HexMapIterRadial(const HexMapCellId center,
            unsigned int radius,
            bool exclude_center = false,
            const HexMap::Planes &planes = HexMap::Planes::All);
    HexMapIterRadial(HexMapIterAxial iter,
            unsigned int radius,
            bool exclude_center) :
            axial_iter(iter),
            radius(radius),
            exclude_center(exclude_center) {};
    HexMapIterRadial(const HexMapIterRadial &) = default;

    friend bool operator==(const HexMapIterRadial &a,
            const HexMapIterRadial &b) {
        return a.axial_iter == b.axial_iter;
    };
    friend bool operator!=(const HexMapIterRadial &a,
            const HexMapIterRadial &b) {
        return a.axial_iter != b.axial_iter;
    };

    HexMapCellId operator*() const { return axial_iter.cell; }
    const HexMapCellId &operator->() { return axial_iter.cell; }
    HexMapIterRadial &operator++();
    HexMapIterRadial begin();
    HexMapIterRadial end();

    // gdscript integration methods
    operator godot::String() const;
    bool _iter_init();
    bool _iter_next();
    HexMapCellId _iter_get() const;
    HexMapIter *clone() const;

    // added for doctests
    friend std::ostream &operator<<(std::ostream &os,
            const HexMapIterRadial &value);

private:
    void advance_until_valid();

    HexMapIterAxial axial_iter;
    unsigned int radius;
    bool exclude_center;
};
