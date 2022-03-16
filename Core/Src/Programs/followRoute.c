#include <programs.h>

#include "stm32f0xx_hal.h"

#include <Ecl/state_estimator.h>
#include <Ecl/position_ctrl.h>

void follow_route(uint8_t routeLength, int32_t speed)
{
    //uint8_t *route[2] = get_route();
    uint8_t route[][2] = {
        {2, 0}
        //{2, 1},
        //{1, 1}
    };
    routeLength = 1;

    for (uint8_t i=0; i<routeLength; i++) {
        int32_t target[] = {
            route[i][0] * CELL_SIZE + CELL_SIZE/2,
            route[i][1] * CELL_SIZE + CELL_SIZE/2
        };
        cprintf("target: (%i, %i)\n\r", target[0], target[1]);
        pos_ctrl_setpoint(target, speed);

        while (!pos_ctrl_target_reached()) {
            /*int32_t pos[2], V, heading;
            get_state(pos, &V, &heading);
            cprintf("Heading: %i\n\r", heading*180/3141/1000);*/
            HAL_Delay(10);
        };

        //print_maze();
    }
}