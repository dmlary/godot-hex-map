#pragma once

#include "godot_cpp/classes/ref_counted.hpp"
#include "godot_cpp/classes/wrapped.hpp"
#include "godot_cpp/core/defs.hpp"
#include "godot_cpp/templates/hashfuncs.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"
#include "godot_cpp/variant/variant.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "planes.h"
#include <climits>
#include <cstdint>

using namespace godot;

namespace hex_bind {
class HexMapCellId;
class HexMapIter;
} //namespace hex_bind

class HexMapIterRadial;

class HexMapCellId {
public:
    // To use HexMapCellId as a key for HashMap or HashSet
    union Key {
        struct {
            int16_t q, r, y, zero;
        };
        uint64_t key = 0;

        _FORCE_INLINE_ Key(const HexMapCellId &cell_id) {
            q = cell_id.q;
            r = cell_id.r;
            y = cell_id.y;
        }
        _FORCE_INLINE_ Key(uint16_t q, uint16_t r, uint16_t y) {
            this->q = q;
            this->r = r;
            this->y = y;
        }
        _FORCE_INLINE_ Key(uint64_t key) : key(key) {}
        Key() {};

        _FORCE_INLINE_ bool operator<(const Key &other) const {
            return key < other.key;
        }
        _FORCE_INLINE_ bool operator==(const Key &other) const {
            return key == other.key;
        }
        _FORCE_INLINE_ operator HexMapCellId() const {
            return HexMapCellId(q, r, y);
        }
        _FORCE_INLINE_ operator uint64_t() const { return key; }
        _FORCE_INLINE_ explicit operator Vector3i() const {
            return Vector3i(q, y, r);
        }
        _FORCE_INLINE_ Key get_cell_above() const { return Key(q, r, y + 1); }
    };

    // axial coordinates
    int32_t q, r;

    // y coordinate
    int32_t y;

    HexMapCellId() : q(0), r(0), y(0) {};
    HexMapCellId(int q, int r, int y) : q(q), r(r), y(y) {};
    _FORCE_INLINE_ HexMapCellId(const Vector3i v) : q(v.x), r(v.z), y(v.y) {};

    static HexMapCellId from_oddr(Vector3i oddr) {
        int q = oddr.x - (oddr.z - (oddr.z & 1)) / 2;
        return HexMapCellId(q, oddr.z, oddr.y);
    }

    // vector3i is the fastest way to get to a Variant type that isn't malloc
    // heavy (like Ref<HexMapCellIdRef>)
    inline operator Vector3i() const { return Vector3i(q, y, r); }
    inline operator Variant() const { return (Vector3i) * this; }
    inline operator Ref<hex_bind::HexMapCellId>() const;
    operator String() const;

    HexMapCellId operator+(const HexMapCellId &other) const {
        return HexMapCellId(q + other.q, r + other.r, y + other.y);
    }
    HexMapCellId operator-(const HexMapCellId &other) const {
        return HexMapCellId(q - other.q, r - other.r, y - other.y);
    }
    bool operator<(const HexMapCellId &other) const {
        return Key(*this) < Key(other);
    }

    friend bool operator==(const HexMapCellId &a, const HexMapCellId &b) {
        return a.q == b.q && a.r == b.r && a.y == b.y;
    }
    friend bool operator!=(const HexMapCellId &a, const HexMapCellId &b) {
        return a.q != b.q || a.r != b.r || a.y != b.y;
    }

    // calcualate s coordinate for converting from axial to cube coordinates
    inline int s() const { return -q - r; }

    // Check to see if the coordinates are within the 16-bit range
    inline bool in_bounds() const {
        return (q >= SHRT_MIN && q <= SHRT_MAX) &&
                (r >= SHRT_MIN && r <= SHRT_MAX) &&
                (y >= SHRT_MIN && y <= SHRT_MAX);
    };

    // calculate the distance between two cells in cell units
    unsigned distance(const HexMapCellId &) const;

    // get all cells within radius of this cell, along the provided planes
    HexMapIterRadial get_neighbors(unsigned int radius = 1,
            const HexMapPlanes &planes = HexMapPlanes::All) const;

    // get the pixel center of this cell assuming the cell is a unit cell with
    // height = 1, radius = 1.  Use HexMap.map_to_local() for center scaled
    // by cell size.
    Vector3 unit_center() const;

    // get a cell id for a point on a unit hex grid (height=1, radius-1)
    static HexMapCellId from_unit_point(const Vector3 &point);

    // convert to odd-r coordinates
    // https://www.redblobgames.com/grids/hexagons/#conversions-offset
    _FORCE_INLINE_ Vector3i to_oddr() const {
        int x = q + (r - (r & 1)) / 2;
        return Vector3i(x, y, r);
    }

    static const HexMapCellId Origin;
    static const HexMapCellId Invalid;
};

// added for testing
std::ostream &operator<<(std::ostream &os, const HexMapCellId &value);

namespace hex_bind {

// wrapper to return a HexMapCellId to GDscript
class HexMapCellId : public RefCounted {
    GDCLASS(HexMapCellId, RefCounted)

    using CellId = ::HexMapCellId;
    CellId cell_id;

public:
    HexMapCellId(const CellId &cell_id) : cell_id(cell_id) {};
    HexMapCellId() {};

    // c++ helpers
    inline operator const CellId &() const { return cell_id; }
    inline Ref<hex_bind::HexMapCellId> operator+(const CellId &delta) const {
        return cell_id + delta;
    };

    // GDScript field accessors
    int _get_q();
    int _get_r();
    int _get_s();
    int _get_y();

    // GDScript static constructors
    static Ref<hex_bind::HexMapCellId>
    _from_coordinates(int p_q, int p_r, int p_y);
    static Ref<hex_bind::HexMapCellId> _from_vector(Vector3i p_vector);
    static Ref<hex_bind::HexMapCellId> _from_int(uint64_t p_hash);

    // gdscript does not support equality for RefCounted objects.  We need some
    // way to use cell ids in arrays, hashes, etc.  We provide the hash method
    // for generating a type that gdscript can handle for equality.
    uint64_t _as_int();
    Vector3i _as_vector();

    // directional helpers
    Ref<hex_bind::HexMapCellId> _east() const;
    Ref<hex_bind::HexMapCellId> _northeast() const;
    Ref<hex_bind::HexMapCellId> _northwest() const;
    Ref<hex_bind::HexMapCellId> _west() const;
    Ref<hex_bind::HexMapCellId> _southwest() const;
    Ref<hex_bind::HexMapCellId> _southeast() const;
    Ref<hex_bind::HexMapCellId> _down() const;
    Ref<hex_bind::HexMapCellId> _up() const;

    String _to_string() const;
    bool _equals(const Ref<hex_bind::HexMapCellId> other) const;
    Vector3 _unit_center() const;
    Ref<hex_bind::HexMapIter> _get_neighbors(unsigned int radius = 1) const;

protected:
    static void _bind_methods();
};

} //namespace hex_bind

inline HexMapCellId::operator Ref<hex_bind::HexMapCellId>() const {
    return Ref<hex_bind::HexMapCellId>(memnew(hex_bind::HexMapCellId(*this)));
}
