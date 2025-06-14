#pragma once

#include <climits>
#include <cstdint>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/wrapped.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/templates/hashfuncs.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "planes.h"

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

    constexpr HexMapCellId() : q(0), r(0), y(0) {};
    constexpr HexMapCellId(int q, int r, int y) : q(q), r(r), y(y) {};
    _FORCE_INLINE_ HexMapCellId(const Vector3i v) : q(v.x), r(v.z), y(v.y) {};

    // vector3i is the fastest way to get to a Variant type that isn't malloc
    // heavy (like Ref<HexMapCellIdRef>)
    inline operator Vector3i() const { return Vector3i(q, y, r); }
    inline operator Variant() const { return (Vector3i) * this; }
    operator String() const;

    /// return a Ref<T> wrapped copy of this HexMapCellId
    Ref<hex_bind::HexMapCellId> to_ref() const;
    inline Vector3i to_vec() const { return static_cast<Vector3i>(*this); }

    HexMapCellId operator+(const HexMapCellId &other) const {
        return HexMapCellId(q + other.q, r + other.r, y + other.y);
    }
    HexMapCellId operator-(const HexMapCellId &other) const {
        return HexMapCellId(q - other.q, r - other.r, y - other.y);
    }
    HexMapCellId operator*(int m) const {
        return HexMapCellId(q * m, r * m, y * m);
    }
    bool operator<(const HexMapCellId &other) const {
        return Key(*this) < Key(other);
    }
    HexMapCellId operator-() const { return HexMapCellId(-q, -r, -y); }

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
            const HexMapPlanes &planes = HexMapPlanes::All,
            bool include_center = false) const;

    // get the pixel center of this cell assuming the cell is a unit cell with
    // height = 1, radius = 1.  Use HexMap.get_cell_center() for center scaled
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
    static HexMapCellId from_oddr(Vector3i oddr) {
        // To adjust for using right-hand coordinate system, we need to flip
        // Z here.
        int q = oddr.x - (oddr.z - (oddr.z & 1)) / 2;
        return HexMapCellId(q, oddr.z, oddr.y);
    }

    // rotate the cell about the y-axis relative to another cell
    // @param steps [int] 60 degree steps to rotate
    // @param center [HexMapCellId] cell to rotate about; default origin
    // @return [HexMapCellId]
    HexMapCellId rotate(int steps,
            HexMapCellId center = HexMapCellId(0, 0, 0)) const;

    static const HexMapCellId ZERO;
    static const HexMapCellId INVALID;
};

// added for testing
std::ostream &operator<<(std::ostream &os, const HexMapCellId &value);

namespace hex_bind {
// wrapper to return a HexMapCellId to GDscript
class HexMapCellId : public RefCounted {
    GDCLASS(HexMapCellId, RefCounted)

    using CellId = ::HexMapCellId;

public:
    CellId inner;

    HexMapCellId(const CellId &cell_id) : inner(cell_id) {};
    HexMapCellId() {};

    // GDScript field accessors
    int get_q();
    void set_q(int);
    int get_r();
    void set_r(int);
    int get_y();
    void set_y(int);
    int get_s();

    // GDScript static constructors
    static Ref<hex_bind::HexMapCellId> at(int p_q, int p_r, int p_y);
    static Ref<hex_bind::HexMapCellId> from_vec(Vector3i p_vector);
    static Ref<hex_bind::HexMapCellId> from_int(uint64_t p_hash);

    // gdscript does not support custom equality for objects.  We need some
    // way to use cell ids in arrays, hashes, etc.  We provide the as_int()
    // method for generating a type that gdscript can handle for equality.
    uint64_t as_int();
    Vector3i as_vec();

    // duplication because this is a ref
    Ref<hex_bind::HexMapCellId> duplicate() const;

    // maths
    Ref<hex_bind::HexMapCellId> add(Ref<hex_bind::HexMapCellId>) const;
    Ref<hex_bind::HexMapCellId> subtract(Ref<hex_bind::HexMapCellId>) const;
    Ref<hex_bind::HexMapCellId> inverse() const;

    // rotation
    Ref<hex_bind::HexMapCellId> rotate(int steps,
            Ref<hex_bind::HexMapCellId> center =
                    Ref<hex_bind::HexMapCellId>()) const;

    // directional helpers
    Ref<hex_bind::HexMapCellId> east() const;
    Ref<hex_bind::HexMapCellId> northeast() const;
    Ref<hex_bind::HexMapCellId> northwest() const;
    Ref<hex_bind::HexMapCellId> west() const;
    Ref<hex_bind::HexMapCellId> southwest() const;
    Ref<hex_bind::HexMapCellId> southeast() const;
    Ref<hex_bind::HexMapCellId> down() const;
    Ref<hex_bind::HexMapCellId> up() const;

    String _to_string() const;
    bool equals(const Ref<hex_bind::HexMapCellId> other) const;
    Vector3 unit_center() const;
    Ref<hex_bind::HexMapIter> get_neighbors(unsigned int radius = 1,
            bool include_self = false) const;

protected:
    static void _bind_methods();
};
} //namespace hex_bind
