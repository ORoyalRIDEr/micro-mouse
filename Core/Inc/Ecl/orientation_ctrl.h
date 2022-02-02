#ifndef ORIENTATION_CTRL_H
#define ORIENTATION_CTRL_H

#include <stdint.h>

void orientation_ctrl_setpoint(int32_t orientation);
void orientation_ctrl_callback(void);

#endif // ORIENTATION_CTRL_H