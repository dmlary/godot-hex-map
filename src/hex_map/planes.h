#pragma once

struct HexMapPlanes {
	bool y : 1;
	bool q : 1;
	bool r : 1;
	bool s : 1;

	static const HexMapPlanes All;
	static const HexMapPlanes QRS;
	static const HexMapPlanes YQR;
	static const HexMapPlanes YRS;
	static const HexMapPlanes YQS;
};
