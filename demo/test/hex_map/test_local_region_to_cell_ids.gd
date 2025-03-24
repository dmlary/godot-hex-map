extends HexMapTest

var region_params = ParameterFactory.named_parameters(
    ['a', 'b', 'cells'], [
	[ Vector3(), Vector3(), [CellId(0,0,0)] ],
	[
		Vector3(-0.1,0,-0.1), Vector3(0.1, 0, 0.1),
		[CellId(0,0,0)]
	],
	[
		Vector3(0 ,0, 0), cell_center(0, 1, 0),
		[CellId(0,0,0), CellId(0, 1, 0)]
	],
	[
		Vector3(-0.1, 0, 0), Vector3(0.1, 0, 1.5),
		[CellId(0,0,0), CellId(-1, 1, 0), CellId(0, 1, 0)]
	],
	[
		Vector3(-0.1, 0, 0), Vector3(-0.01, 0, 1.5),
		[CellId(0,0,0), CellId(-1, 1, 0)]
	],
	[
		Vector3(0.1, 0, 0), Vector3(0.1, 0, 1.5),
		[CellId(0,0,0), CellId(0, 1, 0)]
	],
	[
		cell_center(0, -3, 0), cell_center(1, -1, 0),
		[
		CellId(0, -3, 0), CellId(1, -3, 0), CellId(2, -3, 0),
			CellId(0,-2,0), CellId(1, -2, 0),
		CellId(-1,-1, 0), CellId(0, -1, 0), CellId(1, -1, 0),
		]
	],
	[
		cell_center(0, -3, 1), cell_center(1, -1, 2),
		[
		CellId(0, -3, 1), CellId(1, -3, 1), CellId(2, -3, 1),
			CellId(0,-2,1), CellId(1, -2, 1),
		CellId(-1,-1, 1), CellId(0, -1, 1), CellId(1, -1, 1),

		CellId(0, -3, 2), CellId(1, -3, 2), CellId(2, -3, 2),
			CellId(0,-2,2), CellId(1, -2, 2),
		CellId(-1,-1, 2), CellId(0, -1, 2), CellId(1, -1, 2),
		]
	],
    ])

func test_local_region_to_cell_ids(params=use_parameters(region_params)):
	var hex_map = autofree(HexMap.new())
	var cells = []
	var iter = hex_map.local_region_to_cell_ids(params.a, params.b)
	assert_cells_eq(iter, params.cells)
	pass
