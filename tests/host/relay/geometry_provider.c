#include "relay_protocol.h"
static const struct relay_geometry cases[]
	= {{11, 5, 16, 120, 4096, 15, 0}, {16, 3, 15, 120, 300, 1, 0}, {16, 3, 16, 120, 320, 1, 0}};
const struct relay_geometry *relay_geometry_active = &cases[0];
void select_geometry(unsigned index)
{
	relay_geometry_active = &cases[index];
}
