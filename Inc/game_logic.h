#ifndef __GAMELOGIC_H
#define __GAMELOGIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// ================ DRIBLER LOGIC
#define DRIBLER_OFF			(965)
#define DRIBLER_ON			(935)
#define DRIBLER_TIMEOUT	(1000) // msec
#define DRIBLER_CTRL		(TIM1->CCR2)

inline void off_dribler(void);
inline void on_dribler(void);
inline uint8_t is_dribler_on(void);

void try_change_dribler_state(uint8_t state);
// ================ DRIBLER LOGIC

#ifdef __cplusplus
}
#endif

#endif
