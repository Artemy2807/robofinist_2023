#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "math_fn.h"
#include <stdint.h>

struct PIDUnion 
{
	float kp, ki, kd;
	uint8_t angle_wrap;
	float input, setpoint, output;
	float last_input, i_term;
	
	float out_min, out_max;
	uint32_t last_time;
};

void angle_wrap(float *x);
void constrain(float *x, float min_, float max_);
uint8_t compute(struct PIDUnion *pid, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
