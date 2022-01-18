#ifndef ZS040_H
#define ZS040_H

/*
* Driver for Bluetooth device ZS-040
*/

#include <stdint.h>
#include <stm32f0xx_hal.h>

/* Init bluetooth device. Must be called prior to any other library call */
void ZS040_init(UART_HandleTypeDef *huart, void(*callback)(char*));

/* Print string to bluetooth device. */
void ZS040_print(char* str);
/* Print string to bluetooth device in DMA mode. This is way
faster for longer texts. But it MUST NOT USED IN INTERRUPT functions,
because the interrupts would block each other.*/
void ZS040_print_DMA(char* str);

#endif // ZS040_H