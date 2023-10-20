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
		brake_m(pin_a, pin_b, COAST_MODE);
		return;
	}
	
	if(vel < 0)
	{
		(*pin_a) = (-vel);
		(*pin_b) = 0;
	}
	else
	{
		(*pin_a) = 0;
		(*pin_b) = vel;
	}
}

void stop_m(void)
{
	brake_m(&MOTORA_A, &MOTORA_B, COAST_MODE);
	brake_m(&MOTORB_A, &MOTORB_B, COAST_MODE);
	brake_m(&MOTORC_A, &MOTORC_B, COAST_MODE);
	brake_m(&MOTORD_A, &MOTORD_B, COAST_MODE);
}

void move(uint8_t vel_abs, float theta, int16_t angular)
{
	float vel_f = (float)vel_abs;
	int16_t y_component = vel_f * sinf(RAD(45.0f - theta));
	int16_t x_component = vel_f * -cosf(RAD(theta - 45.0f));
	
	//x_component += 150 * sign(x_component);
	//y_component += 150 * sign(y_component);
	
	float ratio = 1.0f;
	// + angular
	
	//float k = vel_f / max(abs(y_component), abs(x_component));
	
	int16_t a = -x_component + angular,
					b = y_component + angular,
					c = x_component + angular,
					d = -y_component + angular;
	a += 150 * sign(a);
	b += 150 * sign(b);
	c += 150 * sign(c);
	d += 150 * sign(d);
	int16_t max_vel = max(abs(a), max(abs(b), max(abs(c), abs(d))));
	
	/*
	ratio = vel_f / (float)max_vel;
	
	float angular_f = (float)angular;
	if((abs(vel_f) + abs(angular_f)) > 255.0f)
	{
		ratio *= (255.0f - abs(angular_f)) / abs(vel_f);
	}
	
	a = (int16_t)((float)a * ratio);
	b = (int16_t)((float)b * ratio);
	c = (int16_t)((float)c * ratio);
	d = (int16_t)((float)d * ratio);
	
	a += angular;
	b += angular;
	c += angular;
	d += angular;
	max_vel = max(abs(a), max(abs(b), max(abs(c), abs(d))));
	
	if(max_vel > 255)
	{
		ratio = 255.0f / (float)(max_vel);
	}
	else
	{
		ratio = 1.0f;
	}
	*/
	
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
