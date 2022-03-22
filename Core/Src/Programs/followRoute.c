#include <programs.h>

#include "stm32f0xx_hal.h"

#include <Ecl/state_estimator.h>
#include <Ecl/position_ctrl.h>
#include <Ecl/driver.h>

void follow_route(uint8_t routeLength, int32_t speed)
{
    // uint8_t *route[2] = get_route();
    uint8_t route[][2] = {
        /*{3, 0},
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
        {6, 5}*/
        /*{2, 0},
        {0, 0}*/
        {6, 0},
        {4, 0},
        {6, 0}};
    // routeLength = 25;
    routeLength = 2;

    drive_route((uint8_t*) route, routeLength, speed, 0);
}

void drive_to_cell(uint8_t cell[], int32_t speed, uint8_t mapping_enable)
{
    uint8_t pos[2];
    get_position(pos);
    uint8_t path_length = path_to_cell(cell[1], cell[0], pos[0], pos[1]);
    if (path_length > 0)
    {
        uint8_t *route = get_route();
        drive_route(route, path_length, speed, mapping_enable);
    }
}