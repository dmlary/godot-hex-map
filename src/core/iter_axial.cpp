#include "iter_axial.h"
#include "cell_id.h"

void HexMapIterAxial::advance() {
    if (cell.r < r_max) {
        cell.r++;
    } else if (cell.q < q_max) {
        cell.r = r_min;
        cell.q++;
    } else if (cell.y < y_max) {
        cell.r = r_min;
        cell.q = q_min;
        cell.y++;
    } else {
        cell = HexMapCellId::Invalid;
    }
}

void HexMapIterAxial::advance_until_valid() {
    while (cell.s() < s_min || cell.s() > s_max || !cell.in_bounds()) {
        advance();
        if (cell == HexMapCellId::Invalid) {
            break;
        }
    }
}

HexMapIterAxial::HexMapIterAxial(const HexMapCellId center,
        unsigned int radius,
        const HexMapPlanes &planes) :
        center(center) {
    if (planes.y) {
        y_min = center.y - radius;
        y_max = center.y + radius;
    } else {
        y_min = y_max = center.y;
    }
    // XXX prune min/max to only include valid space

    if (planes.q) {
        q_min = center.q - radius;
        q_max = center.q + radius;
    } else {
        q_min = q_max = center.q;
    }

    if (planes.r) {
        r_min = center.r - radius;
        r_max = center.r + radius;
    } else {
        r_min = r_max = center.r;
    }

    if (planes.s) {
        s_min = center.s() - radius;
        s_max = center.s() + radius;
    } else {
        s_min = s_max = center.s();
    }

    cell = HexMapCellId(q_min, r_min, y_min);
    advance_until_valid();
}

// prefix increment
HexMapIterAxial &HexMapIterAxial::operator++() {
    advance();
    advance_until_valid();
    return *this;
}

HexMapIterAxial HexMapIterAxial::begin() {
    HexMapIterAxial iter = *this;
    iter.cell = HexMapCellId(q_min, r_min, y_min);
    iter.advance_until_valid();
    return iter;
}

HexMapIterAxial HexMapIterAxial::end() {
    HexMapIterAxial iter = *this;
    iter.cell = HexMapCellId::Invalid;
    return iter;
}

HexMapIterAxial::operator String() const {
    // clang-format off
    return (String)"{ " +
        ".center = " + center.operator String() + ", " +
        ".cell = " + cell.operator String() + ", " +
        ".q = [" + itos(q_min) + ".." + itos(q_max) + "], " +
        ".r = [" + itos(r_min) + ".." + itos(r_max) + "], " +
        ".s = [" + itos(s_min) + ".." + itos(s_max) + "], " +
        ".y = [" + itos(y_min) + ".." + itos(y_max) + "], " +
    "}";
    // clang-format on
}

bool HexMapIterAxial::_iter_init() {
    *this = begin();
    return cell != HexMapCellId::Invalid;
}

bool HexMapIterAxial::_iter_next() {
    HexMapCellId cell = *operator++();
    return cell != HexMapCellId::Invalid;
}

HexMapCellId HexMapIterAxial::_iter_get() const { return cell; }

HexMapIter *HexMapIterAxial::clone() const {
    return new HexMapIterAxial(*this);
}
