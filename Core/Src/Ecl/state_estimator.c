#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Lib/cmath.h>

// Micro meters per engine step
#define UM_PER_ENGINE_STEP    106
// Inverse of distance of wheels in 1/m
#define D_WHEELS_INV    107/10 // decrease if Wall-E thinks that it has turned more than he really did
#define EST_FREQ   SUB_CTRL_FREQ

#define GRID_SIZE   7   // chunks
#define CELL_SIZE   200*1000 // um
#define SLAM_N_MEAS 10  // number of measurements needed to determine if a wall exists or not
#define SLAM_MAX_ANGLE 262 // 1000 rad, 262->15°, minimum accept angular deviation from grid alignment

int32_t sens_offset[2][4] = { // FRONT, BACK, LEFT, RIGHT
    { 60000, 0},
    {-10000, 0},
    { 50000, -10000},
    { 50000,  10000}
};

int32_t est_pos[] = {0, 0}; // um
int32_t est_V = 0;          // um/s
int32_t est_Psi = 0; // 1,000,000 rad, -pi ... pi
/* variable get -1 if distance sensor went through
   variable get +1 if distance sensor hit wall there 
   max +/- SLAM_N_MEAS */
int8_t maze[GRID_SIZE+1][GRID_SIZE+1];

void get_state(int32_t pos[], int32_t* V, int32_t* Psi)
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
 * @param maze_pos Estimated position in maze
 */
void slam(int32_t dist[], uint32_t maze_pos[])
{
    // only apply slam if aligned to grid (that is, not while turning)
    uint8_t pos_x_aligned = ((est_Psi > (- SLAM_MAX_ANGLE)) && (est_Psi < (SLAM_MAX_ANGLE)));
    uint8_t neg_x_aligned = (est_Psi < (-PI1000 + SLAM_MAX_ANGLE)) || (est_Psi > (PI1000 - SLAM_MAX_ANGLE));
    uint8_t pos_y_aligned = ((est_Psi > (PI1000/2 - SLAM_MAX_ANGLE)) && (est_Psi < (PI1000/2 + SLAM_MAX_ANGLE)));
    uint8_t neg_y_aligned = ((est_Psi > (-PI1000/2 - SLAM_MAX_ANGLE)) && (est_Psi < (-PI1000/2 + SLAM_MAX_ANGLE)));        

    if (!(pos_x_aligned || neg_x_aligned || pos_y_aligned || neg_y_aligned))
        return;

    int32_t xfe[] = {
        cos1000(est_Psi),
        sin1000(est_Psi),
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
        {xfe[1], xfe[0]}
    };

    // process individual measurements
    for (uint8_t laser=0; laser<4; laser++) {
        // invalid or non existing measurements are marked as
        // -1 by the distance sensor driver
        if (dist[laser] < 0)
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
        enum direction {N,E,S,W} dir;
        if (laser == DIST_FRONT) {
            if (pos_x_aligned) dir = N;
            else if (neg_x_aligned) dir = S;
            else if (pos_y_aligned) dir = E;
            else if (neg_y_aligned) dir = W;
        }
        else if (laser == DIST_BACK) {
            if (pos_x_aligned) dir = S;
            else if (neg_x_aligned) dir = N;
            else if (pos_y_aligned) dir = W;
            else if (neg_y_aligned) dir = E;
        }
        else if (laser == DIST_LEFT) {
            if (pos_x_aligned) dir = W;
            else if (neg_x_aligned) dir = E;
            else if (pos_y_aligned) dir = N;
            else if (neg_y_aligned) dir = S;
        }
        else { // laser == DIST_BACK
            if (pos_x_aligned) dir = E;
            else if (neg_x_aligned) dir = W;
            else if (pos_y_aligned) dir = S;
            else if (neg_y_aligned) dir = N;
        }

        /*
         * Calculate distance to next wall
         */
        int32_t pse[2];
        mat_vec_mult(pse, Mef, &(sens_offset[0][laser]), 2, 2); // convert sensor offset from local to global frame

        int32_t row; // row=0->problem in x direction; row=1->y direction
        uint16_t m = 0; // index of wall that is hit next by laser in this direction
        if (dir == N || dir == S) {
            m = est_pos[0] / CELL_SIZE + (dir == N ? 1 : 0);
            row = 0;
        }
        else {// dir == E || W
            m = est_pos[1] / CELL_SIZE + (dir == E ? 1 : 0);
            row = 1;
        }

        int32_t k = (m*CELL_SIZE - est_pos[row] - pse[row]) / (rse[row][laser]);
        cprintf("laser: %i, dir: %i, k: %i\n", laser, dir, k);
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
        ((int32_t) engine_odometry[0]) * UM_PER_ENGINE_STEP * EST_FREQ,
        ((int32_t) engine_odometry[1]) * UM_PER_ENGINE_STEP * EST_FREQ
    }; // um/s


    /**
     *  Orientation Estimation
    **/
    /* yaw rate */
    int32_t w_eng_odom = (v_wheels[0] - v_wheels[1]) * D_WHEELS_INV;
    // TODO: Fuse this with gyro data
    int32_t w = w_eng_odom; // 1000000 / s
    
    /* calculate yaw increment and adjust heading */
    est_Psi += w / EST_FREQ; // 1000000
    est_Psi = ((est_Psi+PI1000*1000) % (2*PI1000*1000)) - PI1000*1000;

    /**
     * Position Estimation
     */
    int32_t xfe[] = {
        cos1000(est_Psi/1000),
        sin1000(est_Psi/1000),
    };

    est_V = (v_wheels[0] + v_wheels[1]) / 2;
    est_pos[0] += est_V * xfe[0] / EST_FREQ / 1000;
    est_pos[1] += est_V * xfe[1] / EST_FREQ / 1000;
}

void init_maze()
{
    for (uint8_t x; x<= GRID_SIZE; x++) {
        for (uint8_t y; y<= GRID_SIZE; y++) {
            maze[x][y] = 0;
        }
    }
}