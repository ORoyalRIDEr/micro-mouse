#ifndef PROGRAMS_H
#define PROGRAMS_H

#include <stdint.h>
#include <main.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Ecl/state_estimator.h>
#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

void parking(void);
void follow_left_wall(void);

void sample_map(uint8_t map[map_size*2][map_size]);

#endif // PROGRAMS_H