#include <Ecl/position_ctrl.h>

#include <Lib/cmath.h>
#include <Lib/printf.h>

#include <Drivers/lre_stepper.h>

#include <Ecl/orientation_ctrl.h>
#include <Ecl/state_estimator.h>

#define POS_CTRL_PHASE_THRESHOLD 87   // rad*1000; minimum heading offset at which driving starts
#define POS_CTRL_ACCEPT_RADIUS 10000  // um; stop driving when setpoint is reached that close
#define POS_CTRL_APPR_THRESHOLD 50000 // um; from this distance on the orientation is not corrected anymore; this avoids the "Tüteneffekt" (https://depositonce.tu-berlin.de/bitstream/11303/6260/3/behrend_ferdinand.pdf)
#define POS_CTRL_STOP_X_BEFORE 20000  // um; stop this amount in front of target; this is used to place the middle of the robot at the target, not the center of the coordinate system which lays between the wheels

int32_t pos_setpoint[] = {0, 0};
int32_t pos_ctrl_speed = 0;
enum pos_ctrl_phase_t
{
    HEAD,     // head to target direction
    DRIVE,    // drive to target
    APPROACH, // drive without orientation correction
    REACHED   // target is reached
} pos_ctrl_phase = HEAD;
int32_t pos_ctrl_Psi_cmd = 0;

void head_to_target()
{
    int32_t pos[2], V, Psi;
    get_state(pos, &V, &Psi);

    pos_ctrl_Psi_cmd = atan21000(
        (pos_setpoint[1] - pos[1]) / 1000,
        (pos_setpoint[0] - pos[0]) / 1000);

    // cprintf("Psi cmd %i\n\r", pos_ctrl_Psi_cmd);
    // cprintf("\t(%i,%i) -> (%i,%i)\n\r", pos[0] / 1000, pos[1] / 1000, pos_setpoint[0] / 1000, pos_setpoint[1] / 1000);
    orientation_ctrl_setpoint(pos_ctrl_Psi_cmd);
}

void pos_ctrl_setpoint(int32_t position[], int32_t speed)
{
    pos_setpoint[0] = position[0];
    pos_setpoint[1] = position[1];
    pos_ctrl_speed = speed;
    pos_ctrl_phase = HEAD;

    head_to_target();
    // cprintf("\tGo to (%i,%i)\n\r", pos_setpoint[0]/1000, pos_setpoint[1]/1000);
}

void pos_ctrl_callback(void)
{
    int32_t pos[2], V, Psi;
    get_state(pos, &V, &Psi);
    int32_t dPsi = pos_ctrl_Psi_cmd - Psi / 1000;
    // cprintf("\tdPsi %i\n\r", dPsi);

    if (pos_ctrl_phase == HEAD)
    {
        head_to_target();

        if (absolute(dPsi) <= POS_CTRL_PHASE_THRESHOLD)
            pos_ctrl_phase = DRIVE;
    }
    else if (pos_ctrl_phase == DRIVE || pos_ctrl_phase == APPROACH)
    {
        // TODO: implement nice velocity control
        forward(pos_ctrl_speed);
        if (pos_ctrl_phase == DRIVE)
            head_to_target();

        int32_t manhattenDist =
            absolute(pos_setpoint[0] - pos[0]) +
            absolute(pos_setpoint[1] - pos[1]) -
            POS_CTRL_STOP_X_BEFORE; // see comment at definition of this makro

        if (manhattenDist < POS_CTRL_APPR_THRESHOLD)
        {
            if (manhattenDist < POS_CTRL_ACCEPT_RADIUS)
            {
                pos_ctrl_phase = REACHED;
                forward(0);
            }
            else
                pos_ctrl_phase = APPROACH;
        }
    }
}

uint8_t pos_ctrl_target_reached()
{
    return (pos_ctrl_phase == REACHED);
}