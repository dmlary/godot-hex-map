#include "planes.h"

const HexMapPlanes HexMapPlanes::All{
	.y = true,
	.q = true,
	.r = true,
	.s = true,
};
const HexMapPlanes HexMapPlanes::QRS{
	.y = false,
	.q = true,
	.r = true,
	.s = true,
};
const HexMapPlanes HexMapPlanes::YRS{
	.y = true,
	.q = false,
	.r = true,
	.s = true,
};
const HexMapPlanes HexMapPlanes::YQS{
	.y = true,
	.q = true,
	.r = false,
	.s = true,
};
const HexMapPlanes HexMapPlanes::YQR{
	.y = true,
	.q = true,
	.r = true,
	.s = false,
};
