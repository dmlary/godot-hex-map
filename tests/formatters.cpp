#include "formatters.h"
#include "core/iter_axial.h"
#include "core/iter_radial.h"
#include <ostream>

namespace godot {
std::ostream &operator<<(std::ostream &os, const Vector3 &value) {
    // clang-format off
		os << "{ .x = " << value.x
		   << ", .y = " << value.y
		   << ", .z = " << value.z << "}";
    // clang-format on
    return os;
}

std::ostream &operator<<(std::ostream &os, const Vector3i &value) {
    // clang-format off
		os << "{ .x = " << value.x
		   << ", .y = " << value.y
		   << ", .z = " << value.z << "}";
    // clang-format on
    return os;
}
} //namespace godot

std::ostream &operator<<(std::ostream &os, const HexMapCellId &value) {
    // clang-format off
    os << "{ .q = "
       << value.q
       << ", .r = "
       << value.r
       << ", .y = "
       << value.y
       << " }";
    // clang-format on
    return os;
}

std::ostream &operator<<(std::ostream &os, const int value[2]) {
    os << "{ " << value[0] << ", " << value[1] << " }";
    return os;
}
std::ostream &operator<<(std::ostream &os, const HexMapIterCube &value) {
    // clang-format off
    os << "{ .min = " << value.min
       << "(" << HexMapCellId::from_oddr(value.min) << ")"
       << ", .max = " << value.max
       << "(" << HexMapCellId::from_oddr(value.max) << ")"
       << ", .pos = " << value.pos
       << "(" << HexMapCellId::from_oddr(value.pos) << ")"
       << ", .min_x_shift = " << value.min_x_shift
       << ", .max_x_shift = " << value.max_x_shift
       << "}";
    // clang-format on
    return os;
}

std::ostream &operator<<(std::ostream &os, const HexMapIterRadial &value) {
    // clang-format off
    os << "{ .axial_iter = " << value.axial_iter
       << ", .radius = " << value.radius
       << " }";
    // clang-format on

    return os;
}

std::ostream &operator<<(std::ostream &os, const HexMapIterAxial &value) {
    // clang-format off
    os << "{ .center = " << value.center
       << ", .cell = " << value.cell
       << ", .q = [" << value.q_min << ", " << value.q_max
       << "], .r = [" << value.r_min << ", " << value.r_max
       << "], .s = [" << value.s_min << ", " << value.s_max
       << "], .y = [" << value.y_min << ", " << value.y_max
       << "] }";
    // clang-format on

    return os;
}

std::ostream &operator<<(std::ostream &os,
        const std::set<HexMapCellId> &value) {
    os << "{ ";
    for (const HexMapCellId &cell : value) {
        os << cell << ", ";
    }
    os << "}";

    return os;
}

std::ostream &operator<<(std::ostream &os,
        const std::vector<HexMapCellId> &value) {
    os << "{ ";
    for (const HexMapCellId &cell : value) {
        os << cell << ", ";
    }
    os << "}";

    return os;
}
