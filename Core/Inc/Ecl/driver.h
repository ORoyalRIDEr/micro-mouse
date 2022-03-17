#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>

void drive_route(uint8_t route[][2], uint8_t routeLength, int32_t speed);

void driver_callback(uint32_t dist[]);

#endif // DRIVER_H