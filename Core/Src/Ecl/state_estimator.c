#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>
#include <Drivers/I3G4250D_gyro.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Lib/cmath.h>

// Micro meters per engine step
#define UM_PER_ENGINE_STEP 106
// Inverse of distance of wheels in 1/m
#define D_WHEELS_INV 107 / 10 // decrease if Wall-E thinks that it has turned more than he really did
#define EST_FREQ SUB_CTRL_FREQ

#define GRID_SIZE 7                             // chunks
#define SLAM_N_MEAS 10                          // number of measurements needed to determine if a wall exists or not
#define SLAM_MAX_ANGLE 262                      // 1000 rad, 262->15°, minimum accept angular deviation from grid alignment
#define SLAM_WALL_REC_THRESHOLD (60 * 1000)     // um, threshold to recognize a wall
#define SLAM_LOC_K_POS 1 / 2                    // Gain which is used to correct the position with measurements
#define SLAM_LOC_K_PSI 1 / 16                   // Gain which is used to correct the orientation with measurements
#define SLAM_MAX_RANGE (500 * 1000)             // um, maximum range where the range measurement is trusted
#define SLAM_MAX_U_POS (5 * 1000)               // um, maximum position correction based on one measurement
#define SLAM_MAX_U_PSI 262                      // 1000 rad, maximum angle correction based on one measurement
#define SLAM_BROKEN_MEAS_THRESHOLD (800 * 1000) // um, measurements above this value are ignored

// offset of sensor in local frame
int32_t sens_off_loc[4][2] = { // FRONT, BACK, LEFT, RIGHT
    {78000, 0},
    {-10000, 0},
    {60000, -45000},
    {60000, 45000}};

int32_t est_pos[] = {100 * 1000, 100 * 1000}; // um
int32_t est_V = 0;                            // um/s
int32_t est_Psi = 0;                          // 1,000,000 rad, -pi ... pi
/* variable get -1 if distance sensor went through
   variable get +1 if distance sensor hit wall there
   max +/- SLAM_N_MEAS */
int8_t maze[2 * GRID_SIZE + 1][GRID_SIZE + 1];

void get_state(int32_t pos[], int32_t *V, int32_t *Psi)
{
    pos[0] = est_pos[0];
    pos[1] = est_pos[1];
    *V = est_V;
    *Psi = est_Psi;
}

/**
 * @brief Uses the distance measurements to build up map and estimate position in maze
 *
 * @param dist Distance Measurements: FRONT, BACK, RIGHT, LEFT
 */
void slam(int32_t dist[])
{
    // only apply slam if aligned to grid (that is, not while turning)
    uint8_t pos_x_aligned = ((est_Psi / 1000 > (-SLAM_MAX_ANGLE)) && (est_Psi / 1000 < (SLAM_MAX_ANGLE)));
    uint8_t neg_x_aligned = (est_Psi / 1000 < (-PI1000 + SLAM_MAX_ANGLE)) || (est_Psi / 1000 > (PI1000 - SLAM_MAX_ANGLE));
    uint8_t pos_y_aligned = ((est_Psi / 1000 > (PI1000 / 2 - SLAM_MAX_ANGLE)) && (est_Psi / 1000 < (PI1000 / 2 + SLAM_MAX_ANGLE)));
    uint8_t neg_y_aligned = ((est_Psi / 1000 > (-PI1000 / 2 - SLAM_MAX_ANGLE)) && (est_Psi / 1000 < (-PI1000 / 2 + SLAM_MAX_ANGLE)));

    if (!(pos_x_aligned || neg_x_aligned || pos_y_aligned || neg_y_aligned))
        return;

    int32_t xfe[] = {
        cos1000(est_Psi / 1000),
        sin1000(est_Psi / 1000),
    }; // x-body axis in global cs

    // direction of laser in body frame
    int32_t rse[2][4];
    rse[0][DIST_FRONT] = xfe[0];
    rse[1][DIST_FRONT] = xfe[1];
    rse[0][DIST_BACK] = -xfe[0];
    rse[1][DIST_BACK] = -xfe[1];
    rse[0][DIST_LEFT] = xfe[1];
    rse[1][DIST_LEFT] = -xfe[0];
    rse[0][DIST_RIGHT] = -xfe[1];
    rse[1][DIST_RIGHT] = xfe[0];

    // transformation matrix f(rover) -> e(earth)
    uint32_t Mef[2][2] = {
        {xfe[0], -xfe[1]},
        {xfe[1], xfe[0]}};

    // cprintf("pos: (%i, %i)\n", est_pos[0], est_pos[1]);

    // process individual measurements
    for (uint8_t laser = 0; laser < 4; laser++)
    {
        // invalid or non existing measurements are marked as
        // -1 by the distance sensor driver
        if ((dist[laser] <= 0) || (dist[laser] >= SLAM_BROKEN_MEAS_THRESHOLD))
            continue;

        /* follow the laser direction for DIST_MAX
        if a wall is hit:
            -> can it be used for positioning? abs(maze value) >= SLAM_THRESHOLD
                -> if yes, does the wall exist (maze value == 10) or is empty (maze value == -1)?
                    -> if yes, use for positioning with existing measurement
                        -> break loop
                    -> if not, go on following the laser
                -> if not, it is used for building the map
                    -> if measurement says that here is a wall
                        -> increase maze value and break loop
                    -> if not
                        -> decrease maze value
        */
        enum direction
        {
            N,
            E,
            S,
            W
        } dir;
        if (laser == DIST_FRONT)
        {
            if (pos_x_aligned)
                dir = N;
            else if (neg_x_aligned)
                dir = S;
            else if (pos_y_aligned)
                dir = E;
            else
                dir = W;
        }
        else if (laser == DIST_BACK)
        {
            if (pos_x_aligned)
                dir = S;
            else if (neg_x_aligned)
                dir = N;
            else if (pos_y_aligned)
                dir = W;
            else
                dir = E;
        }
        else if (laser == DIST_LEFT)
        {
            if (pos_x_aligned)
                dir = W;
            else if (neg_x_aligned)
                dir = E;
            else if (pos_y_aligned)
                dir = N;
            else
                dir = S;
        }
        else
        { // laser == DIST_BACK
            if (pos_x_aligned)
                dir = E;
            else if (neg_x_aligned)
                dir = W;
            else if (pos_y_aligned)
                dir = S;
            else
                dir = N;
        }

        /*
         * Calculate distance to next wall
         */
        int32_t sens_off_glob[2];
        mat_vec_mult(sens_off_glob, (int32_t *)Mef, (sens_off_loc[laser]), 2, 2); // convert sensor offset from local to global frame
        for (uint8_t i = 0; i < 2; i++)
            sens_off_glob[i] /= 1000; // Mef contains sin/cos * 1000; this line normalizes this

        int32_t pos_sens_glob[2];
        vec_add(pos_sens_glob, est_pos, sens_off_glob, 2);
        // cprintf("Offset: (%i, %i) -> (%i, %i)\n", sens_offset[laser][0], sens_offset[laser][1] ,pse[0], pse[1]);

        int32_t row;    // row=0->problem in x direction; row=1->y direction
        uint16_t m = 0; // index of wall that is hit next by laser in this direction
        if (dir == N || dir == S)
        {
            m = pos_sens_glob[0] / CELL_SIZE + (dir == N ? 1 : 0);
            row = 0;
        }
        else
        { // dir == E || W
            m = pos_sens_glob[1] / CELL_SIZE + (dir == E ? 1 : 0);
            row = 1;
        }

        uint8_t continue_slam = 1;
        uint8_t is_first_loop = 1;
        while (continue_slam)
        {
            if (!is_first_loop)
            {
                int8_t delta_m = (dir == N || dir == E) ? 1 : -1;
                m += delta_m;
            }
            is_first_loop = 0;

            // k: distance to next wall
            // multiplication with 1000 because rse contains sin/cos * 1000
            // cprintf("Pos: (%i,%i), Sensor: (%i,%i)\n", est_pos[0], est_pos[1], pos_sens_glob[0], pos_sens_glob[1]);
            int32_t k = (m * CELL_SIZE - pos_sens_glob[row]) * 1000 / (rse[row][laser]);
            // cprintf("laser: %i, dir: %i, wall: %i, k: %i\n", laser, dir, m, k);
            if (k >= SLAM_MAX_RANGE)
                break;

            /*
             * Use measurement for mapping or localisation
             */
            int8_t *maze_loc;
            if (dir == N || dir == S)
                maze_loc = &(maze[2 * m][pos_sens_glob[1] / CELL_SIZE]);
            else // dir == E || W
                maze_loc = &(maze[2 * (pos_sens_glob[0] / CELL_SIZE) + 1][m]);

            /*cprintf("Maze location: (%i, %i)\n",
                ((uint32_t)maze_loc - (uint32_t)maze) / (GRID_SIZE+1),
                ((uint32_t)maze_loc - (uint32_t)maze) % (GRID_SIZE+1)
                );*/

            // difference between calculated and true distance to next wall
            int32_t ddist = dist[laser] - k;

            if (absolute((uint32_t)*maze_loc) >= SLAM_N_MEAS)
            {
                /* LOCALISATION */
                // cprintf("LOCALISATION\n\r");
                if (dist[laser] >= SLAM_MAX_RANGE)
                    break;

                // cprintf("Dir: %i, Loc error: %i, Maze entry: %i \n\r", dir, ddist, *maze_loc);
                if (*maze_loc >= SLAM_N_MEAS)
                { // there is a wall
                    continue_slam = 0;
                    // correct position with given error
                    int32_t u = ddist * xfe[1 - row] / 1000 * SLAM_LOC_K_POS; // xfe[1-row] considers the cosine
                    // limit correction input such that bad input values cannot disturb position too much
                    u = u > SLAM_MAX_U_POS ? SLAM_MAX_U_POS : (u < -SLAM_MAX_U_POS ? -SLAM_MAX_U_POS : u);

                    // cprintf("Dir: %i, Loc error: %i \n\r", dir, ddist);
                    switch (dir)
                    {
                    case N:
                        est_pos[0] -= u;
                        break;
                    case S:
                        est_pos[0] += u;
                        break;
                    case E:
                        est_pos[1] -= u;
                        break;
                    case W:
                        est_pos[1] += u;
                        break;
                    default:
                        break;
                    }
                }
                else
                { // there is no wall
                    continue_slam = 1;
                }
            }
            else
            {
                /* MAPPING */
                // cprintf("MAPPING\n\r");
                if (ddist > SLAM_WALL_REC_THRESHOLD)
                { // there is no wall
                    (*maze_loc)--;
                    continue_slam = 1;
                }
                else if (absolute(ddist) < SLAM_WALL_REC_THRESHOLD)
                { // there is a wall
                    (*maze_loc)++;
                    continue_slam = 0;
                    if (*maze_loc == SLAM_WALL_REC_THRESHOLD)
                        cprintf("Found wall (Maze loc: %i), delta dist: %i, dir: %i\n\r", *maze_loc, ddist, dir);
                }
                else
                { // measurement is smaller than expected; this case does not make sense so it is not treated
                    continue_slam = 0;
                    // cprintf("\tDiscard\n\r");
                }
                //cprintf("Mapping (Maze loc: %i), delta dist: %i, dir: %i\n\r", *maze_loc, ddist, dir);
            }
        }
    }

    /* correcting heading */
    static int32_t pos_old[2] = {0xDEADBEEF, 0xDEADBEEF};
    if ((pos_old[0] == 0xDEADBEEF) && (pos_old[1] == 0xDEADBEEF))
    {
        // init pos_old
        pos_old[0] = est_pos[0];
        pos_old[1] = est_pos[1];
    }
    else
    {
        int32_t scaler = 128; // 2^9 to improve compiler optimisation options; this is used when the absolute value doesnt matter
        int32_t dPos[2] = {
            (est_pos[0] - pos_old[0]),
            (est_pos[1] - pos_old[1]),
        };
        int32_t V_Psi = atan21000(dPos[1] / scaler, dPos[0] / scaler) * 1000;
        int32_t dPsi = V_Psi - est_Psi;
        int32_t PIe6 = PI1000 * 1000;
        dPsi = dPsi > PIe6 ? dPsi - (2 * PIe6) : (dPsi < (-PIe6) ? dPsi + (2 * PIe6) : dPsi);

        // make gain dependant on current velocity command
        uint32_t gain = est_V/100;
        if (gain > (1000*SLAM_LOC_K_PSI))
            gain = 1000*SLAM_LOC_K_PSI;
        //est_Psi += (dPsi / 1000 * gain);

        //cprintf("posShift: (%i,%i), Psi: %i, V_Psi: %i, dPsi: %i, gain: %i \n\r", dPos[0], dPos[1], est_Psi * 180 / PI1000 / 1000, V_Psi * 180 / PI1000 / 1000, dPsi * 180 / PI1000 / 1000, gain);

        pos_old[0] = est_pos[0];
        pos_old[1] = est_pos[1];

    }
}

void estimator_callback()
{
    /**
     * Data aquisition
     */
    int16_t engine_odometry[2];
    get_engine_odometry(engine_odometry);
    // dr: distance that the wheel travelled since last callback
    int32_t v_wheels[] = {
        ((int32_t)engine_odometry[0]) * UM_PER_ENGINE_STEP * EST_FREQ,
        ((int32_t)engine_odometry[1]) * UM_PER_ENGINE_STEP * EST_FREQ}; // um/s

    /**
     *  Orientation Estimation
     **/
    /* yaw rate */
    int32_t w_eng_odom = (v_wheels[0] - v_wheels[1]) * D_WHEELS_INV;
    int32_t w = w_eng_odom; // 1000000 / s

    /* fuse gyro */
    static int32_t gyr_w_int_last = 0;
    int32_t gyr_w, gyr_w_int;
    I3G4250D_gyro_GetGyrIntZ(&gyr_w, &gyr_w_int);
    int32_t gyr_delta_Psi = gyr_w_int - gyr_w_int_last;
    gyr_w_int_last = gyr_w_int;

    /* calculate yaw increment and adjust heading */
    est_Psi += gyr_delta_Psi; //w / EST_FREQ; // gyro is very good, so currently odometry is not used  // 1000000
    est_Psi = ((est_Psi + PI1000 * 1000) % (2 * PI1000 * 1000)) - PI1000 * 1000; // limit heading to -pi...pi

    /**
     * Position Estimation
     */
    int32_t xfe[] = {
        cos1000(est_Psi / 1000),
        sin1000(est_Psi / 1000),
    };

    est_V = (v_wheels[0] + v_wheels[1]) / 2;
    est_pos[0] += est_V * xfe[0] / EST_FREQ / 1000;
    est_pos[1] += est_V * xfe[1] / EST_FREQ / 1000;
}

void init_maze()
{
    for (uint8_t x = 0; x <= 2 * GRID_SIZE; x++)
    {
        for (uint8_t y = 0; y <= GRID_SIZE; y++)
        {
            maze[x][y] =
                (x == 0 || x == 2 * GRID_SIZE || ((x % 2 == 1) && (y == 0)) || y == GRID_SIZE) ? 10 : 0;
        }
    }
}

void print_maze()
{
    uint32_t chunk[] = {
        est_pos[0] / CELL_SIZE,
        est_pos[1] / CELL_SIZE,
    };

    for (int8_t x = 2 * GRID_SIZE; x >= 0; x--)
    {
        for (uint8_t y = 0; y <= GRID_SIZE; y++)
        {
            int8_t maze_val = maze[x][y];

            if (x % 2 == 0)
            {
                cprintf("+");
                if (y == GRID_SIZE)
                    continue; // only uneven lines use whole space
                if (maze_val >= 10)
                    cprintf("---");
                else if (maze_val <= -10)
                    cprintf("   ");
                else
                    cprintf(" ? ");
            }
            else
            {
                if (maze_val >= 10)
                    cprintf("|");
                else if (maze_val <= -10)
                    cprintf(" ");
                else
                    cprintf("?");

                if ((2 * chunk[0] + 1 == x) && (chunk[1] == y))
                    cprintf(" x ");
                else
                    cprintf("   ");
            }
        }
        cprintf("\n\r");
    }

    for (int8_t x = 2 * GRID_SIZE; x >= 0; x--)
    {
        for (uint8_t y = 0; y <= GRID_SIZE; y++)
        {
            int8_t maze_val = maze[x][y];
            cprintf("%i ", maze_val);
        }
        cprintf("\n\r");
    }
}