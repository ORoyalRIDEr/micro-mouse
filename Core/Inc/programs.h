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
void write_wall(uint8_t x, uint8_t y, uint8_t wall_n, uint8_t wall_e, uint8_t wall_s, uint8_t wall_w);
uint8_t* get_map(void);
uint8_t* get_route(void);
uint8_t* get_position(void);
uint8_t get_heading(void);

#endif // PROGRAMS_H