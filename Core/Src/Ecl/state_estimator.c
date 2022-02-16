#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Lib/cmath.h>

// Micro meters per engine step
#define UM_PER_ENGINE_STEP    106
// Inverse of distance of wheels in 1/m
#define D_WHEELS_INV    25/2
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

int32_t est_pos[] = {0, 0};
int32_t est_V = 0;
int32_t est_Psi = 0; // 1000 rad, -pi ... pi
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
void slam(uint32_t dist[], uint32_t maze_pos[])
{
    // only apply slam if aligned to grid (that is, not while turning)
    uint8_t psi_valid =
        est_Psi < (-PI1000/2 + SLAM_MAX_ANGLE) ||
        ((est_Psi > (-PI1000/4 - SLAM_MAX_ANGLE)) && (est_Psi < (-PI1000/4 + SLAM_MAX_ANGLE))) ||
        ((est_Psi > (- SLAM_MAX_ANGLE)) && (est_Psi < (SLAM_MAX_ANGLE))) ||
        ((est_Psi > (PI1000/4 - SLAM_MAX_ANGLE)) && (est_Psi < (PI1000/4 + SLAM_MAX_ANGLE))) ||
        est_Psi > (PI1000/2 + SLAM_MAX_ANGLE);
    if (!psi_valid)
        return;

    int32_t xfe[] = {
        cos1000(est_Psi),
        sin1000(est_Psi),
    }; // x-body axis in global cs

    // current direction of sensors
    int32_t laser_directions[2][4];
    laser_directions[0][DIST_FRONT] = xfe[0];
    laser_directions[1][DIST_FRONT] = xfe[1];
    laser_directions[0][DIST_BACK] = -xfe[0];
    laser_directions[1][DIST_BACK] = -xfe[1];
    laser_directions[0][DIST_LEFT] = xfe[1];
    laser_directions[1][DIST_LEFT] = -xfe[0];
    laser_directions[0][DIST_RIGHT] = -xfe[1];
    laser_directions[1][DIST_RIGHT] = xfe[0];

    // process individual measurements
    for (uint8_t dir=0; dir<4; dir++) {
        // invalid or non existing measurements are marked as
        // -1 by the distance sensor driver
        if (dist[dir] < 0)
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
    int32_t w_eng_odom = (v_wheels[1] - v_wheels[0]) * D_WHEELS_INV;
    // TODO: Fuse this with gyro data
    int32_t w = w_eng_odom; // 1000000 / s
    
    /* calculate yaw increment and adjust heading (calculation
    is linearized to avoid trigonometrical functions) */
    est_Psi += w / EST_FREQ; // 1000
    est_Psi %= PI1000;

    /**
     * Position Estimation
     */
    int32_t xfe[] = {
        cos1000(est_Psi),
        sin1000(est_Psi),
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