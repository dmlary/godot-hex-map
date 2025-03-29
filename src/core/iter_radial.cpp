#include "iter_radial.h"
#include "cell_id.h"

inline void HexMapIterRadial::advance_until_valid() {
    while ((exclude_center && axial_iter.cell == axial_iter.center) ||
            axial_iter.center.distance(axial_iter.cell) > radius) {
        ++axial_iter;
        if (axial_iter.cell == HexMapCellId::Invalid) {
            break;
        }
    }
}

HexMapIterRadial::HexMapIterRadial(const HexMapCellId center,
        unsigned int radius,
        bool exclude_center,
        const HexMapPlanes &planes) :
        axial_iter(center, radius, planes),
        radius(radius),
        exclude_center(exclude_center) {
    advance_until_valid();
}

// prefix increment
HexMapIterRadial &HexMapIterRadial::operator++() {
    ++axial_iter;
    advance_until_valid();
    return *this;
}

HexMapIterRadial HexMapIterRadial::begin() {
    HexMapIterRadial iter(axial_iter.begin(), radius, exclude_center);
    iter.advance_until_valid();
    return iter;
}

HexMapIterRadial HexMapIterRadial::end() {
    HexMapIterRadial iter = *this;
    iter.axial_iter = axial_iter.end();
    return iter;
}

HexMapIterRadial::operator String() const {
    // clang-format off
    return (String)"{ " +
        ".center = " + axial_iter.center.operator String() + ", " +
        ".radius = " + itos(radius) + ", " +
        ".cell = " + axial_iter.cell.operator String() + ", " +
        ".q = [" + itos(axial_iter.q_min) + ".." +
                   itos(axial_iter.q_max) + "], " +
        ".r = [" + itos(axial_iter.r_min) + ".." +
                   itos(axial_iter.r_max) + "], " +
        ".s = [" + itos(axial_iter.s_min) + ".." +
                   itos(axial_iter.s_max) + "], " +
        ".y = [" + itos(axial_iter.y_min) + ".." +
                   itos(axial_iter.y_max) + "], " +
    "}";
    // clang-format on
}

bool HexMapIterRadial::_iter_init() {
    *this = begin();
    return axial_iter.cell != HexMapCellId::Invalid;
}

bool HexMapIterRadial::_iter_next() {
    HexMapCellId cell = *operator++();
    return cell != HexMapCellId::Invalid;
}

HexMapCellId HexMapIterRadial::_iter_get() const { return axial_iter.cell; }

HexMapIter *HexMapIterRadial::clone() const {
    return new HexMapIterRadial(*this);
}
