#ifndef PROGRAMS_H
#define PROGRAMS_H

#include <stdint.h>
#include <main.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Ecl/state_estimator.h>
#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#define MAP_SIDE 15

void parking(void);
void follow_left_wall(void);

void sample_map(void);
void sample_route(void);

void print_map(void);
#endif // PROGRAMS_H