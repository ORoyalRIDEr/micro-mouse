#ifndef HCSR04
#define HCSR04

#include <stdint.h>
#include <stm32f0xx_hal.h>

enum distances {DIST_FRONT, DIST_BACK, DIST_LEFT, DIST_RIGHT};

void HCSR04_Init (TIM_HandleTypeDef *hcsr04_timer);
void HCSR04_Measure (void);
void HCSR04_Read (uint32_t distances[]);

#endif // HCSR04