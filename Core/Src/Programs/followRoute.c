#include <programs.h>

#include "stm32f0xx_hal.h"

#include <Ecl/state_estimator.h>
#include <Ecl/position_ctrl.h>
#include <Ecl/driver.h>

void follow_route(uint8_t routeLength, int32_t speed)
{
    //uint8_t *route[2] = get_route();
    uint8_t route[][2] = {
        {3, 0},
        {3, 1},
        {1, 1},
        {1, 2},
        {0, 2},
        {0, 3},
        {2, 3},
        {2, 5},
        {3, 5},
        {3, 6},
        {4, 6},
        {4, 5},
        {5, 5},
        {5, 4},
        {4, 4},
        {4, 2},
        {3, 2},
        {3, 3},
        {3, 2},
        {4, 2},
        {4, 4},
        {5, 4},
        {5, 6},
        {6, 6},
        {6, 5},

        //{2, 1},
        //{1, 1}
    };
    routeLength = 25;

    drive_route(route, routeLength, speed);
}