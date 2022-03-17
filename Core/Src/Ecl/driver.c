#include <Ecl/driver.h>
#include <Ecl/orientation_ctrl.h>
#include <Ecl/state_estimator.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#include <Lib/cmath.h>
#include <Lib/printf.h>

#define DRIVER_PHASE_THRESHOLD 87 // rad*1000; minimum heading offset at which driving starts
#define FLW_WALL_TARGET 50000     // um; target distance for wall that is currently followed
#define FLW_WALL_MIN_THRESHOLD 40000
#define FLW_WALL_MAX_THRESHOLD 90000
#define INV_K_FLW_WALL 100
#define TARGET_DIST_THRESHOLD 110000
#define MIN_DIST_FRONT 100000
#define CHUNK_SIZE 200000

void set_orientation(enum orient_mode_t mode);

enum driv_states_t
{
    IDLE,
    ROTATING,
    DRIVING
} driv_state = IDLE;

enum driving_dir_t
{
    NS, // North/South
    EW  // East/West
} driving_dir;

uint8_t driver_route[30][2];
uint8_t driver_routeLength;
int32_t driver_speed;
int32_t driver_orientation = 0;
int32_t driver_pos[2] = {0, 6};
uint8_t i_wp = 0;
int32_t target_dir = 0;

void drive_route(uint8_t route[][2], uint8_t routeLength, int32_t speed)
{
    for (uint8_t i = 0; i < routeLength; i++)
    {
        driver_route[i][0] = route[i][0];
        driver_route[i][1] = route[i][1];
    }
    driver_routeLength = routeLength;
    driver_speed = speed;
    i_wp = 0;

    set_orientation(REL);
    driv_state = ROTATING;
    cprintf("Rotate\n\r");
}

void driver_callback(uint32_t dist[])
{
    static int32_t start_pos[] = {0, 0};

    if (driv_state == ROTATING)
    {
        int32_t pos[2], V, Psi;
        get_state(pos, &V, &Psi);
        int32_t dPsi = target_dir - Psi / 1000;
        if (dPsi > deg2rad1000(180))
            dPsi -= deg2rad1000(360);
        else if (dPsi < -deg2rad1000(180))
            dPsi += deg2rad1000(360);

        if (absolute(dPsi) <= DRIVER_PHASE_THRESHOLD)
        {
            driv_state = DRIVING;
            orientation_ctrl_setpoint(target_dir, REL);
            cprintf("Drive with %i%%\n\r", driver_speed);
            forward((int16_t)driver_speed);
            start_pos[0] = pos[0];
            start_pos[1] = pos[1];
        }
    }

    else if (driv_state == DRIVING)
    {
        int8_t wall = 0; // 1 if can follow left wall; -1 if can follow right wall
        int32_t d = 0;

        if ((dist[DIST_LEFT] > FLW_WALL_MIN_THRESHOLD) && (dist[DIST_LEFT] < FLW_WALL_MAX_THRESHOLD))
        {
            wall = 1;
            d = dist[DIST_LEFT];
        }
        else if ((dist[DIST_RIGHT] > FLW_WALL_MIN_THRESHOLD) && (dist[DIST_RIGHT] < FLW_WALL_MAX_THRESHOLD))
        {
            wall = -1;
            d = dist[DIST_RIGHT];
        }

        if (wall != 0)
        {
            int32_t error = FLW_WALL_TARGET - d;
            int32_t orient_cmd = wall * error / INV_K_FLW_WALL + target_dir;
            orientation_ctrl_setpoint(orient_cmd, REL);
        }
        else
            orientation_ctrl_setpoint(target_dir, REL);

        // check if target reached
        static uint8_t min_dist_front_cnt = 0;
        if (dist[DIST_FRONT] < MIN_DIST_FRONT)
            min_dist_front_cnt++;
        else
            min_dist_front_cnt = 0;
        static uint32_t dfront_filtered = 20000;
        if (dist[DIST_FRONT] < 800000)
            dfront_filtered = dfront_filtered*3/4 + dist[DIST_FRONT]/4;

        int32_t pos[2], V, Psi;
        get_state(pos, &V, &Psi);
        uint32_t dist_to_target = 0;

        uint8_t ind = driving_dir == NS ? 0 : 1;
        dist_to_target = absolute(
            pos[ind] - start_pos[ind] // distance travelled
            - (driver_route[i_wp][ind] - driver_pos[ind])*CHUNK_SIZE // distance that needs to be travelled
            );

        // cprintf("pos %i->%i, dist: %i\n\r", pos[0], start_pos[0], dist_to_target);

        if (
            ((dist_to_target <= TARGET_DIST_THRESHOLD) && (dfront_filtered > 200000))
            || min_dist_front_cnt >= 3)
        {
            // target reached
            forward(0);
            i_wp++;
            cprintf("Target %i/%i reached\n\r", i_wp, driver_routeLength);
            if (i_wp == driver_routeLength)
                driv_state = IDLE;
            else
            {
                cprintf("Rotate\n\r");
                driver_pos[0] = driver_route[i_wp - 1][0];
                driver_pos[1] = driver_route[i_wp - 1][1];
                set_orientation(FWD);
                driv_state = ROTATING;
            }
        }
    }
}

void set_orientation(enum orient_mode_t mode)
{
    if (driver_route[i_wp][0] == driver_pos[0])
    {
        // movement in y direction
        driving_dir = EW;
        if (driver_route[i_wp][1] > driver_pos[1])
            target_dir = deg2rad1000(90);
        else
            target_dir = -deg2rad1000(90);
    }
    else
    {
        // movement in x direction
        driving_dir = NS;
        if (driver_route[i_wp][0] > driver_pos[0])
            target_dir = 0;
        else
            target_dir = deg2rad1000(180);
    }

    orientation_ctrl_setpoint(target_dir, mode);
}