#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdint.h>

/* 
Notes on coordinate system:
- The origin of is the location at startup
- x-axis points to where rover pointed at startup
- y-axis points 90° to the right of x-axis
- units are in micro meters 
*/


/* Get estimated position, velocity and heading of position
@input pos: array where x and y position is stored
@input V: pointer where the absolute velocity in um/s is stored.
@input Psi: pointer where the yaw angle in 1000*radian is stored.
*/
void get_state(int32_t pos[], int32_t* V, int32_t* Psi);

/* Executes an estimation step. This is called in the control
stack */
void estimator_callback(void);

#endif // STATE_ESTIMATOR_H