#ifndef __OPT_H
#define __OPT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "math_fn.h"

struct OpticalUnion
{
	float fcenter_x,
				fcenter_y;

	// -1 if not found current gate...
	int16_t ygate_x,
					ygate_y;
	int16_t bgate_x,
					bgate_y;

	float pos_x,
				pos_y;
	
	uint8_t last_status,
					is_rotated;
};

uint8_t estimate_position(struct OpticalUnion *optu);
uint8_t check_ouzone(struct OpticalUnion *optu, float *direction);

#ifdef __cplusplus
}
#endif

#endif
