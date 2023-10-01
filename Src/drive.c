#include "drive.h"

void brake_m(__IO uint32_t *pin_a, __IO uint32_t *pin_b, uint8_t mode)
{
	switch(mode)
	{
		case COAST_MODE:
		{
			(*pin_a) = 0;
			(*pin_b) = 0;
			break;
		}
		
		case BRAKE_MODE:
		{
			(*pin_a) = MAX_DUTY;
			(*pin_b) = MAX_DUTY;
			break;
		}
	}
}

void vel_m(__IO uint32_t *pin_a, __IO uint32_t *pin_b, int16_t vel)
{
	int16_t abs_vel = ABS(vel);
	if(abs_vel > MAX_DUTY)
	{
		return;
	}
	
	if(abs_vel < MIN_DUTY)
	{
		brake_m(pin_a, pin_b, BRAKE_MODE);
		return;
	}
	
	if(vel < 0)
	{
		(*pin_a) = MAX_DUTY;
		(*pin_b) = (MAX_DUTY + vel);
	}
	else
	{
		(*pin_a) = (MAX_DUTY - vel);
		(*pin_b) = MAX_DUTY;
	}
}

void stop_m(void)
{
	brake_m(&MOTORA_A, &MOTORA_B, BRAKE_MODE);
	brake_m(&MOTORB_A, &MOTORB_B, BRAKE_MODE);
	brake_m(&MOTORC_A, &MOTORC_B, BRAKE_MODE);
	brake_m(&MOTORD_A, &MOTORD_B, BRAKE_MODE);
}

void move(uint8_t vel_abs, float theta, int16_t angular)
{
	float vel_f = (float)vel_abs;
	int16_t y_component = vel_f * sinf(RAD(45.0f - theta));
	int16_t x_component = vel_f * -cosf(RAD(theta - 45.0f));
	
	float ratio = 1.0f;
	int16_t a = -x_component + angular,
					b = y_component + angular,
					c = x_component + angular,
					d = -y_component + angular;
	int16_t max_vel = max(abs(a), max(abs(b), max(abs(c), abs(d))));
	if(max_vel > 255)
	{
		ratio = 255.0f / (float)max_vel;
	}
	
	a = (int16_t)((float)a * ratio);
	b = (int16_t)((float)b * ratio);
	c = (int16_t)((float)c * ratio);
	d = (int16_t)((float)d * ratio);
	
	vel_m(&MOTORA_A, &MOTORA_B, a);
	vel_m(&MOTORB_A, &MOTORB_B, b);
	vel_m(&MOTORC_A, &MOTORC_B, c);
	vel_m(&MOTORD_A, &MOTORD_B, d);
}
