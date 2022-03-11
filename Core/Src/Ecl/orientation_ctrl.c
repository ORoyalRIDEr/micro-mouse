#include <Ecl/orientation_ctrl.h>

#include <Lib/cmath.h>
#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>

#define K_GAIN_INV 4
#define MAX_TURN_RATE 50

int32_t orient_setpoint = 0;

void orientation_ctrl_callback(void)
{
    int32_t pos[2], V, Psi;
    get_state(pos, &V, &Psi);

    int32_t delta = orient_setpoint - Psi / 1000;
    // adjust delta to always choose the shortest direction to target
    if (delta > deg2rad1000(180))
        delta = delta - deg2rad1000(360);
    if (delta < -deg2rad1000(180))
        delta = delta + deg2rad1000(360);

    int16_t turn_rate = delta / K_GAIN_INV;
    if (turn_rate > MAX_TURN_RATE)
        turn_rate = MAX_TURN_RATE;
    else if (turn_rate < -MAX_TURN_RATE)
        turn_rate = -MAX_TURN_RATE;

    rotate(turn_rate);
}

void orientation_ctrl_setpoint(int32_t orientation)
{
    orient_setpoint = orientation;
}