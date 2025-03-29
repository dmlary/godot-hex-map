#include "core/cell_id.h"
#include "core/iter_axial.h"
#include "core/planes.h"
#include "doctest.h"
#include "formatters.h"
#include <sys/signal.h>
#include <algorithm>
#include <climits>
#include <csignal>

using CellId = HexMapCellId;

TEST_CASE("HexMapIterAxial") {
    struct TestCase {
        HexMapCellId center;
        unsigned int radius;
        HexMapPlanes planes = HexMapPlanes::All;
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
                .radius = 1,
                .cells = {
		    CellId(0, 0, -1),
		    CellId(1,0,-1),
		    CellId(-1,0,-1),
		    CellId(0,1,-1),
		    CellId(-1,1,-1),
		    CellId(0,-1,-1),
		    CellId(1,-1,-1),

		    CellId(0, 0, 0),
		    CellId(1,0,0),
		    CellId(-1,0,0),
		    CellId(0,1,0),
		    CellId(-1,1,0),
		    CellId(0,-1,0),
		    CellId(1,-1,0),

		    CellId(0, 0, 1),
		    CellId(1,0,1),
		    CellId(-1,0,1),
		    CellId(0,1,1),
		    CellId(-1,1,1),
		    CellId(0,-1,1),
		    CellId(1,-1,1),
		},
        },
	// bounds checking
        {
                .center = HexMapCellId(SHRT_MAX + 10, 0, 0),
                .radius = 1,
                .cells = {},
        },
        {
                .center = HexMapCellId(0, 0, SHRT_MAX + 1),
                .radius = 1,
                .cells = {
		    CellId(0, 0, SHRT_MAX),
		    CellId(1,0,SHRT_MAX),
		    CellId(-1,0,SHRT_MAX),
		    CellId(0,1,SHRT_MAX),
		    CellId(-1,1,SHRT_MAX),
		    CellId(0,-1,SHRT_MAX),
		    CellId(1,-1,SHRT_MAX),
		},
        },
        {
                .center = HexMapCellId(SHRT_MAX+1, 0, 0),
                .radius = 1,
                .cells = {
		    CellId(SHRT_MAX, 0, -1),
		    CellId(SHRT_MAX, 1, -1),
		    CellId(SHRT_MAX, 0, 0),
		    CellId(SHRT_MAX, 1, 0),
		    CellId(SHRT_MAX, 0, 1),
		    CellId(SHRT_MAX, 1, 1),
		},
        },
    };

    for (auto &i : cases) {
        CAPTURE(i.center);
        CAPTURE(i.radius);
        if (i.trap) {
            raise(SIGTRAP);
        }
        HexMapIterAxial iter(i.center, i.radius, i.planes);
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
