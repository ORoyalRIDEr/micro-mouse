#include "stm32f0xx_hal.h"

void lre_stepper_setStep(uint8_t step);

void lre_stepper_setStep_side(uint8_t step, uint8_t right);

void rotate(uint32_t step);
void forward(uint32_t times);