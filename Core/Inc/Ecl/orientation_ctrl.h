#ifndef ORIENTATION_CTRL_H
#define ORIENTATION_CTRL_H

#include <stdint.h>

enum orient_mode_t
{
    FWD, // Turn by moving forward
    BWD, // Turn by moving backward
    REL  // Turn relative to current speed
};

void orientation_ctrl_setpoint(int32_t orientation, enum orient_mode_t mode);
void orientation_ctrl_callback(void);
int32_t orientation_ctrl_get_setpoint(void);

#endif // ORIENTATION_CTRL_H