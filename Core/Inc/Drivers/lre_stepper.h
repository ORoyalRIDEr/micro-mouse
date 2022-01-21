#ifndef LRE_STEPPER_H
#define LRE_STEPPER_H

#include "stm32f0xx_hal.h"

void engine_timer_callback(void);

void rotate(int16_t speed);
void forward(int16_t speed);

void get_engine_odometry(int16_t ret_steps[]);

#endif // LRE_STEPPER_H