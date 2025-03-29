#pragma once

#include "godot_cpp/variant/variant.hpp"
#include <cstdint>
#include <godot_cpp/variant/basis.hpp>

using namespace godot;

class HexMapTileOrientation {
public:
    enum Value : uint8_t {
        Upright0, // No flip, 0 degrees rotation
        Upright60, // No flip, 60 degrees rotation
        Upright120, // No flip, 120 degrees rotation
        Upright180, // No flip, 180 degrees rotation
        Upright240, // No flip, 240 degrees rotation
        Upright300, // No flip, 300 degrees rotation

        Flipped0, // Flipped, 0 degrees rotation
        Flipped60, // Flipped, 60 degrees rotation
        Flipped120, // Flipped, 120 degrees rotation
        Flipped180, // Flipped, 180 degrees rotation
        Flipped240, // Flipped, 240 degrees rotation
        Flipped300 // Flipped, 300 degrees rotation
    };

    HexMapTileOrientation() = default;
    constexpr HexMapTileOrientation(Value v) : value(v) {}
    constexpr HexMapTileOrientation(int v) : value(static_cast<Value>(v)) {}
    HexMapTileOrientation(Variant v) : value(static_cast<Value>((int)v)) {}
    explicit inline operator int() const { return value; }
    constexpr bool operator==(HexMapTileOrientation other) const {
        return value == other.value;
    }
    constexpr bool operator!=(HexMapTileOrientation other) const {
        return value != other.value;
    }
    constexpr bool operator!=(int other) const { return value != other; }
    HexMapTileOrientation operator+(const HexMapTileOrientation &other) const;

    operator Basis() const;
    operator Variant() const { return Variant(value); }

    /// rotate the orientation
    /// @param steps nevative values rotate clockwise, positive rotate
    /// counter-clockwise
    void rotate(int steps);
    void flip();

private:
    Value value = Upright0;
};
