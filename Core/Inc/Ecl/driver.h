#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>

enum driving_dir_t
{
    N,
    E,
    S,
    W
};

void drive_route(uint8_t route[], uint8_t routeLength, int32_t speed, uint8_t mapping_enable, uint8_t recalc_map);

void driver_callback(int32_t dist[]);

void get_position(uint8_t pos[]);
enum driving_dir_t get_heading(void);

void set_chunk(uint8_t x, uint8_t y);

#endif // DRIVER_H