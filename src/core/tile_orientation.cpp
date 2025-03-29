#include "tile_orientation.h"
#include "godot_cpp/variant/string.hpp"

#define Math_PI_3 (Math_PI / 3.0)

HexMapTileOrientation::operator Basis() const {
    Basis basis;
    switch (value) {
        case Upright0:
            // noop
            break;
        case Upright60:
            basis.rotate(Vector3(0, 1, 0), Math_PI_3);
            break;
        case Upright120:
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 2);
            break;
        case Upright180:
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 3);
            break;
        case Upright240:
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 4);
            break;
        case Upright300:
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 5);
            break;
        case Flipped0:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            break;
        case Flipped60:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            basis.rotate(Vector3(0, 1, 0), Math_PI_3);
            break;
        case Flipped120:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 2);
            break;
        case Flipped180:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 3);
            break;
        case Flipped240:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 4);
            break;
        case Flipped300:
            basis.rotate(Vector3(1, 0, 0), Math_PI);
            basis.rotate(Vector3(0, 1, 0), Math_PI_3 * 5);
            break;
    }
    return basis;
}

HexMapTileOrientation HexMapTileOrientation::operator+(
        const HexMapTileOrientation &other) const {
    HexMapTileOrientation out(value);
    // if we're flipped, flip, and then rotate however manu steps
    if (other.value >= HexMapTileOrientation::Flipped0) {
        out.flip();
    }
    out.rotate(other.value % 6);
    return out;
}

void HexMapTileOrientation::rotate(int steps) {
    int base = value >= Flipped0 ? 6 : 0;
    int offset = value - base + steps;
    if (offset < 0) {
        offset = 6 + (offset % 6);
    } else if (offset >= 6) {
        offset = offset % 6;
    }

    value = (Value)(base + offset);
}

void HexMapTileOrientation::flip() {
    switch (value) {
        case Upright0:
        case Upright60:
        case Upright120:
        case Upright180:
        case Upright240:
        case Upright300:
            value = (Value)(value + 6);
            break;
        case Flipped0:
        case Flipped60:
        case Flipped120:
        case Flipped180:
        case Flipped240:
        case Flipped300:
            value = (Value)(value - 6);
            break;
    }
}
