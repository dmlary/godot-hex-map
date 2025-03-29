#include "core/cell_id.h"
#include "core/iter_radial.h"
#include "doctest.h"
#include "formatters.h"
#include <sys/signal.h>
#include <algorithm>
#include <csignal>

using CellId = HexMapCellId;

TEST_CASE("HexMapIterRadial") {
    struct TestCase {
        HexMapCellId center;
        unsigned int radius;
        bool exclude_center = false;
        std::set<CellId> cells;
        bool trap = false; // simplify debugging specific test cases
    };
    std::vector<TestCase> cases = {
        {
                .center = HexMapCellId(),
                .radius = 0,
                .cells = { CellId(0, 0, 0) },
        },
        {
                .center = HexMapCellId(),
                .radius = 0,
                .exclude_center = true,
                .cells = { },
        },
        {
                .center = HexMapCellId(),
                .radius = 1,
                .cells = {
                    CellId(0, 0, -1),

                    CellId(0, 0, 0),
                    CellId(1,0,0),
                    CellId(-1,0,0),
                    CellId(0,1,0),
                    CellId(-1,1,0),
                    CellId(0,-1,0),
                    CellId(1,-1,0),

                    CellId(0, 0, 1),
                },
        },
    };

    for (auto &i : cases) {
        CAPTURE(i.center);
        CAPTURE(i.radius);
        CAPTURE(i.exclude_center);
        if (i.trap) {
            raise(SIGTRAP);
        }
        HexMapIterRadial iter(i.center, i.radius, i.exclude_center);
        CAPTURE(iter);
        CAPTURE(*iter);

        std::set<CellId> cells;
        for (const auto &cell : iter) {
            cells.insert(cell);
        }
        INFO("expected := ", i.cells);
        INFO("   found := ", cells);

        std::vector<CellId> missing;
        std::set_difference(i.cells.begin(),
                i.cells.end(),
                cells.begin(),
                cells.end(),
                std::inserter(missing, missing.begin()));
        INFO(" missing := ", missing);

        std::vector<CellId> extra;
        std::set_difference(cells.begin(),
                cells.end(),
                i.cells.begin(),
                i.cells.end(),
                std::inserter(extra, extra.begin()));
        INFO("   extra := ", extra);

        CHECK(cells == i.cells);
    }
}
