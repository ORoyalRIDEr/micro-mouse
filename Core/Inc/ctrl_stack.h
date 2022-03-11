#ifndef CTRL_STACK_H
#define CTRL_STACK_H

#include "stm32f0xx_hal.h"

#define MAIN_CTRL_FREQ  1000  // Hz
#define SUB_CTRL_FREQ   20    // Hz

/* These controllers can be added or removed individually */
enum ctrl_modes_t {CTRL_ORIENTATION, CTRL_POS, EST_SLAM, CTRL_EST_ALL=0xFFFFFFFF};

void ctrl_callback(TIM_HandleTypeDef* timer);
void ctrl_set_mode (enum ctrl_modes_t mode);
void ctrl_unset_mode (enum ctrl_modes_t mode);
uint32_t get_cpu_usage(void);

#endif // CTRL_STACK_H