#include "core/cell_id.h"
#include "doctest.h"

TEST_CASE("HexMapCellId::operator+()") {
    CHECK(HexMapCellId(0, 0, 0) + HexMapCellId(1, 2, 3) ==
            HexMapCellId(1, 2, 3));
    CHECK(HexMapCellId(0, 0, 0) + HexMapCellId(-1, -2, -3) ==
            HexMapCellId(-1, -2, -3));
}

TEST_CASE("HexMapCellId::operator-()") {
    CHECK(HexMapCellId(0, 0, 0) - HexMapCellId(1, 2, 3) ==
            HexMapCellId(-1, -2, -3));
    CHECK(HexMapCellId(0, 0, 0) - HexMapCellId(-1, -2, -3) ==
            HexMapCellId(1, 2, 3));
}

TEST_CASE("HexMapCellId::distance(const HexMapCellId &other)") {
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 0, 0)) == 0);

    // movement along an axis
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(10, 0, 0)) == 10); // q
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(-10, 0, 0)) == 10);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 10, 0)) == 10); // r
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, -10, 0)) == 10);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 0, 10)) == 10); // y
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 0, -10)) == 10);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(10, -10, 0)) == 10); // s
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(-10, 10, 0)) == 10);

    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(1, 1, 0)) == 2);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(1, 1, 1)) == 3);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(1, 1, 2)) == 4);

    // up/down neighbors off axis
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(1, 0, 1)) == 2);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 1, 1)) == 2);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(1, 0, -1)) == 2);

    // diagonal along the r axis
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 2, 2)) == 4);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 3, 3)) == 6);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(0, 4, 4)) == 8);

    // diagonal not on any particular axis
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(2, -1, 5)) == 7);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(4, -2, 2)) == 6);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(4, -2, 2)) == 6);
    CHECK(HexMapCellId(0, 0, 0).distance(HexMapCellId(123, -24, 21)) ==
            123 + 21);
}

TEST_CASE("HexMapCellId::operator Vector3i() reversability") {
    HexMapCellId hex(1, 2, 3);
    Vector3i vector = hex;
    HexMapCellId after = vector;
    CHECK_MESSAGE(after == hex,
            "vector round trip did not work",
            hex,
            vector,
            after);
}
