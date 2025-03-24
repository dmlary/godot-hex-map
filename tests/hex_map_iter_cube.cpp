#include "doctest.h"
#include "formatters.h"
#include "godot_cpp/variant/vector3.hpp"
#include "hex_map/hex_map.h"
#include "hex_map/iter_cube.h"
#include <sys/signal.h>
#include <algorithm>
#include <climits>
#include <csignal>

using CellId = HexMap::CellId;

TEST_CASE("HexMapIterCube") {
    struct TestCase {
        Vector3 min, max;
        std::set<CellId> cells;
        bool trap = false; // simplify debugging specific test cases
    };
    std::vector<TestCase> cases = {
        {
                .min = Vector3(),
                .max = Vector3(),
                .cells = { CellId() },
        },
        {
                .min = Vector3(1, 0, 0),
                .max = Vector3(1, 0, 0),
                .cells = { CellId(1, 0, 0) },
        },
        {
                .min = Vector3(0, 0, 0),
                .max = Vector3(1, 0, 0),
                .cells = { CellId(0, 0, 0), CellId(1, 0, 0) },
        },
        {
                .min = Vector3(0, 0, 0),
                .max = Vector3(1, 0, 0),
                .cells = { CellId(0, 0, 0), CellId(1, 0, 0) },
        },
		// ensure we do the proper thing when the min point is left of the
		// center of the starting cell.  We should include the cell to the
		// southwest and southeast of the origin cell.
        {
                .min = Vector3(-0.1, 0, 0),
                .max = Vector3(0.1, 0, 1.5),
                .cells = {
					CellId(0, 0, 0),
					CellId(-1, 1, 0),
					CellId(0, 1, 0),
				},
        },
		// similar to above, but start and end points to the left of 0 to only
		// select origin and southwest cell.
        {
                .min = Vector3(-0.1, 0, 0),
                .max = Vector3(-0.05, 0, 1.5),
                .cells = {
					CellId(0, 0, 0),
					CellId(-1, 1, 0),
				},
        },
		// same, but to the right of center this time
        {
                .min = Vector3(0.1, 0, 0),
                .max = Vector3(0.2, 0, 1.5),
                .cells = {
					CellId(0, 0, 0),
					CellId(0, 1, 0),
				},
        },

		// similar to the above but starting on an odd cell
        {
                .min = CellId(-1, 1, 0).unit_center(),
                .max = CellId(-1, 2, 0).unit_center(),
                .cells = {
					CellId(-1, 1, 0),
					CellId(-1, 2, 0),
				},
        },


		// try some y-axis things
        {
                .min = Vector3(0, 0, 0),
                .max = Vector3(0, 2, 0),
                .cells = {
					CellId(0,0,0),
					CellId(0,0,1),
					CellId(0,0,2),
				},
        },
        {
                .min = Vector3(0, 0, 0),
                .max = Vector3(0, -2, 0),
                .cells = {
					CellId(0,0,0),
					CellId(0,0,-1),
					CellId(0,0,-2),
				},
        },

		// let's do a full hex prism, all axis, goooo
        {
                .min = Vector3(-1, -1, -1),
                .max = Vector3(1,1,1),
                .cells = {
					CellId(0,0,0),
					CellId(0,0,1),
					CellId(0,0,-1),

					CellId(1,0,0),
					CellId(-1,0,0),
					CellId(0,1,0),
					CellId(-1,1,0),
					CellId(0,-1,0),
					CellId(1,-1,0),

					CellId(1,0,1),
					CellId(-1,0,1),
					CellId(0,1,1),
					CellId(-1,1,1),
					CellId(0,-1,1),
					CellId(1,-1,1),

					CellId(1,0,-1),
					CellId(-1,0,-1),
					CellId(0,1,-1),
					CellId(-1,1,-1),
					CellId(0,-1,-1),
					CellId(1,-1,-1),
				},
        },

		// similar to the above, but we want to extend the selection to create
		// a cube instead of a hexagon shape
        {
                .min = Vector3(-Math_SQRT3 - 0.1, -1, -1),
                .max = Vector3(Math_SQRT3 + 0.1, 1, 1),
                .cells = {
					CellId(0,0,0),
					CellId(0,0,1),
					CellId(0,0,-1),

					// hex y = 0
					CellId(1,0,0),
					CellId(-1,0,0),
					CellId(0,1,0),
					CellId(-1,1,0),
					CellId(0,-1,0),
					CellId(1,-1,0),

					// corners y = 0
					CellId(-1,-1,0),
					CellId(1,1,0),
					CellId(-2,1,0),
					CellId(2,-1,0),

					// hex y=1
					CellId(1,0,1),
					CellId(-1,0,1),
					CellId(0,1,1),
					CellId(-1,1,1),
					CellId(0,-1,1),
					CellId(1,-1,1),

					// corners y = 1
					CellId(-1,-1,1),
					CellId(1,1,1),
					CellId(-2,1,1),
					CellId(2,-1,1),

					// hex y = -1
					CellId(1,0,-1),
					CellId(-1,0,-1),
					CellId(0,1,-1),
					CellId(-1,1,-1),
					CellId(0,-1,-1),
					CellId(1,-1,-1),

					// corners y = -1
					CellId(-1,-1,-1),
					CellId(1,1,-1),
					CellId(-2,1,-1),
					CellId(2,-1,-1),
				},
        },

		// bounds checking
        {
                .min = CellId(SHRT_MAX+1, 0, 0).unit_center(),
                .max = CellId(SHRT_MAX+2, 0, 0).unit_center(),
                .cells = {},
		},
        {
                .min = CellId(SHRT_MIN - 1, SHRT_MIN - 1, SHRT_MIN)
							.unit_center(),
                .max = CellId(SHRT_MIN, SHRT_MIN, SHRT_MIN) .unit_center(),
                .cells = {
					CellId(SHRT_MIN, SHRT_MIN, SHRT_MIN),
				},
		},
        {
                .min = CellId(SHRT_MAX, SHRT_MAX, SHRT_MAX) .unit_center(),
                .max = CellId(SHRT_MAX + 1, SHRT_MAX + 1, SHRT_MAX)
							.unit_center(),
                .cells = {
					CellId(SHRT_MAX, SHRT_MAX, SHRT_MAX),
				},
		},

    };

    for (auto &i : cases) {
        CAPTURE(i.min);
        CAPTURE(i.max);
        if (i.trap) {
            raise(SIGTRAP);
        }
        HexMapIterCube iter(i.min, i.max);
        CAPTURE(iter);

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
