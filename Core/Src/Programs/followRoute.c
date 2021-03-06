#include <programs.h>

#include "stm32f0xx_hal.h"

#include <Ecl/state_estimator.h>
#include <Ecl/position_ctrl.h>
#include <Ecl/driver.h>

#include <Drivers/I3G4250D_gyro.h>

#include <Lib/cmath.h>

void follow_route(uint8_t routeLength, int32_t speed)
{
    // uint8_t *route[2] = get_route();
    set_chunk(0, 0);
    I3G4250D_gyro_SetHeading(deg2rad1000(90) * 1000);
    uint8_t route[][2] = {
        {0, 0},
        {0, 1},
        {2, 1},
        {0, 1},
        {0, 0}};
    // routeLength = 25;
    routeLength = 2;

    drive_route((uint8_t *)route, routeLength, speed, 1, 0);
}

void move_cell_rel(int8_t fwd, int8_t right, int32_t speed)
{
    uint8_t pos[2];
    get_position(pos);
    enum driving_dir_t cur_head = get_heading();

    switch (cur_head)
    {
    case N:
        pos[1] -= fwd;
        pos[0] += right;
        break;
    case E:
        pos[1] += right;
        pos[0] += fwd;
        break;
    case S:
        pos[1] += fwd;
        pos[0] -= right;
        break;
    default:
        pos[1] -= right;
        pos[0] -= fwd;
        break;
    }

    uint8_t tmp = pos[1];
    pos[1] = pos[0];
    pos[0] = tmp;

    cprintf("Target: (%i,%i)\n\r", pos[0], pos[1]);

    drive_to_cell(pos, speed, 1);
}

void drive_to_cell(uint8_t cell[], int32_t speed, uint8_t mapping_enable)
{
    if (!mapping_enable)
        load_true_map();
    uint8_t pos[2];
    get_position(pos);
    uint8_t path_length = path_to_cell(cell[1], cell[0], pos[0], pos[1]);
    if (path_length > 0)
    {
        uint8_t *route = get_route();
        drive_route(route, path_length, speed, mapping_enable, 1);
    }
}