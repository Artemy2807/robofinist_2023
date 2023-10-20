#ifndef __DRIVE_H
#define __DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "math_fn.h"

#define COAST_MODE 	(uint8_t)0
#define BRAKE_MODE 	(uint8_t)1
#define MIN_DUTY		(uint32_t)150
#define MAX_DUTY		(uint32_t)255

// A motor
#define MOTORA_A		TIM2->CCR1
#define MOTORA_B		TIM2->CCR2
// B motor
#define MOTORB_A		TIM12->CCR2
#define MOTORB_B		TIM8->CCR2
// C motor
#define MOTORC_A		TIM2->CCR3
#define MOTORC_B		TIM2->CCR4
// D motor
#define MOTORD_A		TIM8->CCR4
#define MOTORD_B		TIM8->CCR3

void brake_m(__IO uint32_t *pin_a, __IO uint32_t *pin_b, uint8_t mode);
void vel_m(__IO uint32_t *pin_a, __IO uint32_t *pin_b, int16_t vel);

void stop_m(void);
void move(uint8_t vel_abs, float theta, int16_t angular);

#ifdef __cplusplus
}
#endif

#endif
