#include "core/iter_cube.h"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector3i.hpp"
#include <ostream>

namespace godot {
std::ostream &operator<<(std::ostream &os, const Vector3 &value);
std::ostream &operator<<(std::ostream &os, const Vector3i &value);
} //namespace godot

std::ostream &operator<<(std::ostream &os, const HexMapIterCube &value);
std::ostream &operator<<(std::ostream &os,
        const std::set<HexMapCellId> &value);
std::ostream &operator<<(std::ostream &os,
        const std::vector<HexMapCellId> &value);
