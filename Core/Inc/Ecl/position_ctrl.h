#ifndef POSITION_CTRL_H
#define POSITION_CTRL_H

#include <stdint.h>

void pos_ctrl_setpoint(int32_t position[], int32_t speed);
void pos_ctrl_callback(void);
/* returns 1 if current target setpoint is reached */
uint8_t pos_ctrl_target_reached();

#endif // POSITION_CTRL_H