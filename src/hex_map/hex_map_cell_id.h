#pragma once

#include "godot_cpp/classes/ref_counted.hpp"
#include "godot_cpp/classes/wrapped.hpp"
#include "godot_cpp/templates/hashfuncs.hpp"
#include "godot_cpp/variant/variant.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "planes.h"
#include <cstdint>

using namespace godot;

class HexMapCellIdRef;
class HexMapIterAxial;
class HexMapIterAxialRef;

class HexMapCellId {
public:
	// To use HexMapCellId as a key for HashMap or HashSet
	union Key {
		struct {
			int16_t q, r, y;
		};
		uint64_t key;

		Key(const HexMapCellId &cell_id) :
				q(cell_id.q), r(cell_id.r), y(cell_id.y) {}
		Key() : key(0){};

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
	};

	// axial coordinates
	int q, r;

	// y coordinate
	int y;

	HexMapCellId() : q(0), r(0), y(0){};
	HexMapCellId(int q, int r, int y) : q(q), r(r), y(y){};
	HexMapCellId(Vector3i v) : q(v.x), r(v.z), y(v.y){};

	// XXX remove this; temporary until we fully switch from Vector3i to
	// HexMapCellId
	inline operator Vector3i() const { return Vector3i(q, y, r); }
	inline operator Variant() const { return Ref<HexMapCellIdRef>(*this); }
	inline operator Ref<HexMapCellIdRef>() const;
	operator String() const;

	HexMapCellId operator+(const HexMapCellId &other) const {
		return HexMapCellId(q + other.q, r + other.r, y + other.y);
	}
	HexMapCellId operator-(const HexMapCellId &other) const {
		return HexMapCellId(q - other.q, r - other.r, y - other.y);
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
		return ((q >> 15) == 0 || (q >> 15) == -1) &&
				((r >> 15) == 0 || (r >> 15) == -1) &&
				((y >> 15) == 0 || (y >> 15) == -1);
	};

	// calculate the distance between two cells in cell units
	unsigned distance(const HexMapCellId &) const;

	// get all cells within radius of this cell, along the provided planes
	HexMapIterAxial get_neighbors(unsigned int radius = 1,
			const HexMapPlanes &planes = HexMapPlanes::All) const;

	// get the pixel center of this cell assuming the cell is a unit cell with
	// height = 1, radius = 1.  Use HexMap.map_to_local() for center scaled
	// by cell size.
	Vector3 unit_center() const;

	// get a cell id for a point on a unit hex grid (height=1, radius-1)
	static HexMapCellId from_unit_point(const Vector3 &point);

	// convert to odd-r coordinates
	// https://www.redblobgames.com/grids/hexagons/#conversions-offset
	_FORCE_INLINE_ Vector3 to_oddr() const {
		int x = q + (r - (r & 1)) / 2;
		return Vector3i(x, y, r);
	}

	static const HexMapCellId Origin;
	static const HexMapCellId Invalid;
};

// added for testing
std::ostream &operator<<(std::ostream &os, const HexMapCellId &value);

// wrapper to return a HexMapCellId to GDscript
class HexMapCellIdRef : public RefCounted {
	GDCLASS(HexMapCellIdRef, RefCounted)

	uint64_t INVALID_CELL_ID_HASH = 0xffff000000000000;

	HexMapCellId cell_id;

public:
	HexMapCellIdRef(const HexMapCellId &cell_id) : cell_id(cell_id){};
	HexMapCellIdRef(){};

	// c++ helpers
	inline operator const HexMapCellId &() const { return cell_id; }
	inline Ref<HexMapCellIdRef> operator+(const HexMapCellId &delta) const {
		return cell_id + delta;
	};

	// GDScript field accessors
	int _get_q();
	int _get_r();
	int _get_s();
	int _get_y();

	// GDScript static constructors
	static Ref<HexMapCellIdRef> _from_values(int p_q, int p_r, int p_y);
	static Ref<HexMapCellIdRef> _from_vector(Vector3i p_vector);
	static Ref<HexMapCellIdRef> _from_hash(uint64_t p_hash);

	// gdscript does not support equality for RefCounted objects.  We need some
	// way to use cell ids in arrays, hashes, etc.  We provide the hash method
	// for generating a type that gdscript can handle for equality.
	uint64_t _hash();
	Vector3i _as_vector();

	// directional helpers
	Ref<HexMapCellIdRef> _east() const;
	Ref<HexMapCellIdRef> _northeast() const;
	Ref<HexMapCellIdRef> _northwest() const;
	Ref<HexMapCellIdRef> _west() const;
	Ref<HexMapCellIdRef> _southwest() const;
	Ref<HexMapCellIdRef> _southeast() const;
	Ref<HexMapCellIdRef> _down() const;
	Ref<HexMapCellIdRef> _up() const;

	String _to_string() const;
	bool _equals(const Ref<HexMapCellIdRef> other) const;
	Ref<HexMapIterAxialRef> _get_neighbors(unsigned int radius = 1) const;

protected:
	static void _bind_methods();
};

inline HexMapCellId::operator Ref<HexMapCellIdRef>() const {
	return Ref<HexMapCellIdRef>(memnew(HexMapCellIdRef(*this)));
}
