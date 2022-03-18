#include <Ecl/driver.h>
#include <Ecl/orientation_ctrl.h>
#include <Ecl/state_estimator.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#include <Lib/cmath.h>
#include <Lib/printf.h>

#define DRIVER_PHASE_THRESHOLD deg2rad1000(5) // rad*1000; minimum heading offset at which driving starts
#define FLW_WALL_TARGET 50000                 // um; target distance for wall that is currently followed
#define FLW_WALL_MIN_THRESHOLD 40000          // um, minimum distant, that wall is used to be followed
#define FLW_WALL_MAX_THRESHOLD 90000          // um, maximum distant, that wall is used to be followed
#define INV_K_FLW_WALL 100                    // ctrl gain for: wall distance error -> heading offset
#define TARGET_DIST_THRESHOLD 110000          // um, minimum distance to target waypoint to switch to next one
#define MIN_DIST_FRONT 100000                 // um, switch to next waypoint if wall is this distance in front of rover
#define CHUNK_SIZE 200000

#define PRINT_STATE(state) cprintf(state) // this can be used to turn off printing of current driver state

int32_t get_orientation_to_wp(uint8_t wp_chunk[], uint8_t current_chunk[]);
int32_t get_rel_angle_to(int32_t target);
uint8_t orientation_reached();
void follow_wall_callback(uint32_t dist[], int32_t dir);

enum driv_states_t
{
    IDLE,
    ROTATING90,
    ROTATING180,
    DRIVING,
    MAPPING
} driv_state = IDLE;

enum driving_dir_t
{
    N,
    S,
    E,
    W
} driving_dir = N;

uint8_t driver_route[30][2];
uint8_t driver_routeLength;
int32_t driver_speed;
uint8_t i_wp = 0;
uint8_t mapping_active = 0;
uint8_t current_chunk[] = {0, 0};

void drive_route(uint8_t route[][2], uint8_t routeLength, int32_t speed)
{
    // first waypoint needs to be current chunk
    driver_route[0][0] = current_chunk[0];
    driver_route[0][1] = current_chunk[1];
    for (uint8_t i = 0; i < routeLength; i++)
    {
        driver_route[i + 1][0] = route[i][0];
        driver_route[i + 1][1] = route[i][1];
    }

    driver_routeLength = routeLength + 1; // first chunk was added
    driver_speed = speed;
    i_wp = 1; // first wp is current chunk, so start with next one

    driv_state = MAPPING;
    PRINT_STATE("Mapping\n\r");
}

void driver_callback(uint32_t dist[])
{
    static int32_t segment_start[] = {0, 0}; // location where the current line segment starts

    int32_t cur_pos[2], cur_V, cur_Psi;
    get_state(cur_pos, &cur_V, &cur_Psi);

    /*
        Driver State Machine
    */

    if (driv_state == ROTATING90)
    {
        /** State Transitions **/
        if (orientation_reached())
        {
            segment_start[0] = cur_pos[0];
            segment_start[1] = cur_pos[1];

            // refresh direction in which the rover will drive next
            if (driver_route[i_wp][0] == driver_route[i_wp - 1][0])
            {
                // movement in y direction
                if (driver_route[i_wp][1] > driver_route[i_wp - 1][1])
                    driving_dir = E;
                else
                    driving_dir = W;
            }
            else
            {
                // movement in x direction
                if (driver_route[i_wp][0] > driver_route[i_wp - 1][0])
                    driving_dir = N;
                else
                    driving_dir = S;
            }

            driv_state = DRIVING;
            PRINT_STATE("Driving\n\r");
        }
    }

    else if (driv_state == ROTATING180)
    {
        /** State Transitions **/
        if (orientation_reached())
        {
            // rover has rotated first 90°; no turn to target
            int32_t orient = get_orientation_to_wp(driver_route[i_wp], current_chunk);
            orientation_ctrl_setpoint(orient, FWD);

            driv_state = ROTATING90;
            PRINT_STATE("Rotating 90°\n\r");
        }
    }

    else if (driv_state == DRIVING)
    {
        // int32_t nextchunk32bit[] = {(int32_t) driver_route[i_wp-1][0], (int32_t) driver_route[i_wp-1][1]};
        int32_t orient = get_orientation_to_wp(driver_route[i_wp], driver_route[i_wp - 1]);
        follow_wall_callback(dist, orient);
        forward((int16_t)driver_speed);

        // check if target reached
        static uint32_t dfront_filtered = 200000;
        if (dist[DIST_FRONT] < 800000) // broken measurements are usually very large; these are discarded
            dfront_filtered = dfront_filtered * 3 / 4 + dist[DIST_FRONT] / 4;

        uint8_t ind = 0; // index for direction vectors; N/S: 0, E/W: 1
        if (driving_dir == E || driving_dir == W)
            ind = 1;

        int32_t dist_to_travel = absolute((driver_route[i_wp][ind] - driver_route[i_wp - 1][ind]) * CHUNK_SIZE);
        int32_t dist_travelled = absolute(cur_pos[ind] - segment_start[ind]);
        uint32_t dist_remaining = absolute(dist_to_travel - dist_travelled);
        // cprintf("Direction: %i, %i->%i, remaining: %i\n\r", driving_dir, dist_travelled, dist_to_travel, dist_remaining);

        // refresh current chunk
        switch (driving_dir)
        {
        case N:
            current_chunk[0] += dist_travelled / CHUNK_SIZE;
            break;
        case S:
            current_chunk[0] -= dist_travelled / CHUNK_SIZE;
            break;
        case E:
            current_chunk[1] += dist_travelled / CHUNK_SIZE;
            break;
        default:
            current_chunk[1] -= dist_travelled / CHUNK_SIZE;
            break;
        }

        /** State Transitions **/
        if (
            ((dist_remaining <= TARGET_DIST_THRESHOLD) && (dfront_filtered > 200000)) || // required distance travelled
            (dfront_filtered < MIN_DIST_FRONT))                                          // wall is in the way
        {
            // target reached
            forward(0);
            current_chunk[0] = driver_route[i_wp][0];
            current_chunk[1] = driver_route[i_wp][1];
            i_wp++;
            cprintf("\tTarget %i/%i reached (dfront: %i)\n\r", i_wp - 1, driver_routeLength - 1, dfront_filtered);

            driv_state = MAPPING;
            PRINT_STATE("Mapping\n\r");
        }
    }

    else if (driv_state == MAPPING)
    {
        uint8_t mapping_finished = 0;
        // TODO: implement measuring and adding to map

        /** State Transitions **/
        if (!mapping_active || mapping_finished)
        {
            if (i_wp == driver_routeLength)
            {
                driv_state = IDLE; // finished route
                PRINT_STATE("Idle\n\r");
            }

            else
            {
                int32_t target_dir = get_orientation_to_wp(driver_route[i_wp], current_chunk);
                cprintf("\tTarget dir:%i\n\r", target_dir);
                int32_t dPsi = get_rel_angle_to(target_dir);
                cprintf("\tRel angle:%i\n\r", dPsi);

                if (absolute(dPsi) > deg2rad1000(135))
                {
                    // 180° turn needed; therefore, first turn half the needed angle backwards
                    int32_t current_Psi = orientation_ctrl_get_setpoint(); // we assume to have the orientation that was last commanded to orient. ctrl
                    int32_t intermediate_target = current_Psi + dPsi / 2;
                    orientation_ctrl_setpoint(intermediate_target, BWD);

                    driv_state = ROTATING180;
                    PRINT_STATE("Rotating 180°\n\r");
                }
                else
                {
                    // 90° turn needed
                    orientation_ctrl_setpoint(target_dir, FWD);

                    driv_state = ROTATING90;
                    PRINT_STATE("Rotating 90°\n\r");
                }
            }
        }
    }
}

void get_current_chunk(int32_t chunk_res)
{
}

int32_t get_orientation_to_wp(uint8_t wp_chunk[], uint8_t current_chunk[])
{
    int32_t dir = 0;
    if (wp_chunk[0] == current_chunk[0])
    {
        // movement in y direction
        if (wp_chunk[1] > current_chunk[1])
            dir = deg2rad1000(90);
        else
            dir = -deg2rad1000(90);
    }
    else
    {
        // movement in x direction
        if (wp_chunk[0] > current_chunk[0])
            dir = 0;
        else
            dir = deg2rad1000(180);
    }

    return dir;
}

int32_t get_rel_angle_to(int32_t target)
{
    int32_t pos[2], V, Psi;
    get_state(pos, &V, &Psi);
    int32_t dPsi = target - Psi / 1000;

    if (dPsi > deg2rad1000(180))
        dPsi -= deg2rad1000(360);
    else if (dPsi < -deg2rad1000(180))
        dPsi += deg2rad1000(360);

    return dPsi;
}

uint8_t orientation_reached()
{
    int32_t targetDir = orientation_ctrl_get_setpoint();
    int32_t dPsi = get_rel_angle_to(targetDir);
    return (absolute(dPsi) <= DRIVER_PHASE_THRESHOLD);
}

void follow_wall_callback(uint32_t dist[], int32_t dir)
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
        int32_t orient_cmd = wall * error / INV_K_FLW_WALL + dir;
        orientation_ctrl_setpoint(orient_cmd, REL);
    }
    else
        orientation_ctrl_setpoint(dir, REL);
}