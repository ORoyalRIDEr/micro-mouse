#include <Ecl/orientation_ctrl.h>

#include <Lib/cmath.h>
#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>

#define K_GAIN_INV 10
#define MAX_TURN_RATE 40
#define MAX_OFF_90 8 // maximum degree, the commanded signal can be different from a multiple of 90°

int32_t orient_setpoint = 0;
enum orient_mode_t orient_mode = REL;

void orientation_ctrl_callback(void)
{
    int32_t pos[2], V, Psi;
    get_state(pos, &V, &Psi);

    int32_t delta = orient_setpoint - Psi / 1000;
    // adjust delta to always choose the shortest direction to target
    if (delta > deg2rad1000(180))
        delta = delta - deg2rad1000(360);
    else if (delta < -deg2rad1000(180))
        delta = delta + deg2rad1000(360);

    int16_t turn_rate = delta / K_GAIN_INV;
    if (turn_rate > MAX_TURN_RATE)
        turn_rate = MAX_TURN_RATE;
    else if (turn_rate < -MAX_TURN_RATE)
        turn_rate = -MAX_TURN_RATE;

    if (orient_mode == FWD)
        forward(absolute(turn_rate)); // with rotate function, this leads to one wheel standing still, while the other one moves forward
    else if (orient_mode == BWD)
        forward(-absolute(turn_rate));
    rotate(turn_rate);
}

void orientation_ctrl_setpoint(int32_t orientation, enum orient_mode_t mode)
{
    if (orientation > deg2rad1000(180))
        orientation = orientation - deg2rad1000(360);
    else if (orientation < -deg2rad1000(180))
        orientation = orientation + deg2rad1000(360);

    // limit to MAX_OFF_90
    int32_t dir90 = -deg2rad1000(180);
    for (uint8_t i = 0; i < 5; i++)
    {
        uint32_t diffabs = absolute(orientation - dir90);
        if (diffabs <= deg2rad1000(45))
            break;
        dir90 += deg2rad1000(90);
    }
    if (dir90 - orientation > deg2rad1000(MAX_OFF_90))
        orientation = dir90 - deg2rad1000(MAX_OFF_90);
    else if (dir90 - orientation < -deg2rad1000(MAX_OFF_90))
        orientation = dir90 + deg2rad1000(MAX_OFF_90);

    orient_setpoint = orientation;

    orient_mode = mode;
}

int32_t orientation_ctrl_get_setpoint()
{
    return orient_setpoint;
}