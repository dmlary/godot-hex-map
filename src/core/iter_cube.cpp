#include "iter_cube.h"
#include "cell_id.h"

HexMapIterCube::HexMapIterCube(Vector3 a, Vector3 b) {
    if (a > b) {
        Vector3 tmp = a;
        a = b;
        b = tmp;
    }

    HexMapCellId min_cell = HexMapCellId::from_unit_point(a);
    HexMapCellId max_cell = HexMapCellId::from_unit_point(b);
    real_t min_x_center = min_cell.unit_center().x;
    real_t max_x_center = max_cell.unit_center().x;

    min = pos = min_cell.to_oddr();
    max = max_cell.to_oddr();

    // The location of the corner of the region within a cell matters for hex
    // cells, specifically the x coordinate.  If you pick a point anywhere
    // within a hex cell, and draw a line down along the z-axis, that line will
    // intercept either the cell to the southwest or southeast of the clicked
    // cell.
    //
    // For both the left and right sides of the region, we need to
    // determine if the southwest/southeast-most cells fall within
    // the region.  We do this by adjusting the x-min and x-max for the
    // even and odd rows independently.  We use the following table to
    // determine the modifier for the rows for both the minimum x
    // value (in a.x), and the maximum x value (in b.x).
    //
    // Given an x coordinate in local space:
    // | cell z coord | x > cell_center.x | odd mod | even mod |
    // | even         | false             |  -1     | 0        |
    // | even         | true              |   0     | 0        |
    // | odd          | false             |   0     | 0        |
    // | odd          | true              |   0     | 1        |

    // if we start on an odd row, and the x coordinate of the region start
    // point is to the right of the center of the starting cell, we want to
    // start with min.x + 1 when processing even rows.
    if ((min.z & 1) == 1 && a.x >= min_x_center) {
        min_x_shift[0] = 1;
    }
    // if we start on an even row, and the x coordinate of the region start
    // point is to the left of the center of the starting cell, we want to
    // start with min.x - 1 when processing odd rows.
    else if ((min.z & 1) == 0 && a.x < min_x_center) {
        min_x_shift[1] = -1;
    }

    // same as above, but for the maximum x values
    if ((max.z & 1) == 1 && b.x > max_x_center) {
        max_x_shift[0] = 1;
    } else if ((max.z & 1) == 0 && b.x <= max_x_center) {
        max_x_shift[1] = -1;
    }

    // now set the max_x for the current position
    max_x = max.x + max_x_shift[pos.z & 1];

    if (!min_cell.in_bounds()) {
        operator++();
    }
}

HexMapCellId HexMapIterCube::operator*() const {
    if (pos == Vector3i(INT_MAX, INT_MAX, INT_MAX)) {
        return HexMapCellId::Invalid;
    } else {
        return HexMapCellId::from_oddr(pos);
    }
}

HexMapIterCube &HexMapIterCube::operator++() {
    do {
        if (pos.x < max_x) {
            pos.x++;
        } else if (pos.z < max.z) {
            pos.z++;
            pos.x = min.x + min_x_shift[pos.z & 1];
            max_x = max.x + max_x_shift[pos.z & 1];
        } else if (pos.y < max.y) {
            pos.x = min.x;
            pos.z = min.z;
            pos.y++;
        } else {
            pos = Vector3i(INT_MAX, INT_MAX, INT_MAX);
            break;
        }
    } while (!HexMapCellId::from_oddr(pos).in_bounds());

    return *this;
}

HexMapIterCube HexMapIterCube::begin() const {
    HexMapIterCube iter = *this;
    iter.pos = min;
    if (!HexMapCellId::from_oddr(iter.pos).in_bounds()) {
        iter.operator++();
    }
    return iter;
};

HexMapIterCube HexMapIterCube::end() const {
    HexMapIterCube iter = *this;
    iter.pos = Vector3i(INT_MAX, INT_MAX, INT_MAX);
    return iter;
};

HexMapIterCube::operator String() const {
    // clang-format off
    return (String)"{" +
        "\n    .min = " + min.operator String() +
            " (" + HexMapCellId::from_oddr(min)  + "), " +
        "\n    .max = " + max.operator String() +
            " (" + HexMapCellId::from_oddr(max)  + "), " +
        "\n    .pos = " + pos.operator String() +
            " (" + HexMapCellId::from_oddr(pos)  + "), " +
        "\n    .min_x_shift = [" + itos(min_x_shift[0]) +
            ", " + itos(min_x_shift[1]) + "], " +
        "\n    .max_x_shift = [" + itos(max_x_shift[0]) +
            ", " + itos(max_x_shift[1]) + "], " +
    "\n}";
    // clang-format on
}

bool HexMapIterCube::_iter_init() {
    *this = begin();
    return **this != HexMapCellId::Invalid;
}

bool HexMapIterCube::_iter_next() {
    HexMapCellId cell = *operator++();
    return cell != HexMapCellId::Invalid;
}

HexMapCellId HexMapIterCube::_iter_get() const { return **this; }

HexMapIter *HexMapIterCube::clone() const { return new HexMapIterCube(*this); }
