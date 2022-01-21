#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>

// Micro meters per engine step
#define UM_PER_ENGINE_STEP    106
// Inverse of distance of wheels in 1/m
#define D_WHEELS_INV    25/2
#define EST_FREQ   SUB_CTRL_FREQ

// heading vector is scaled by 1000, so to norm squard is 1000^2
#define HEADING_NORM_SQUARED 1000000 
#define NORM_DESCENT_GAIN   -1/1000

int32_t est_pos[] = {0, 0};
int32_t est_V = 0;
int32_t est_heading[] = {1000, 0};

void get_state(int32_t pos[], int32_t* V, int32_t heading[])
{
    pos[0] = est_pos[0];
    pos[1] = est_pos[1];
    *V = est_V;
    heading[0] = est_heading[0];
    heading[1] = est_heading[1];
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
    int32_t dPsi = w / EST_FREQ; // 1000
    int32_t heading_0_prev = est_heading[0]; // (buffer)
    est_heading[0] += dPsi*est_heading[1]  / 1000000;
    est_heading[1] += -dPsi*heading_0_prev / 1000000;

    /* normalize heading vector using gradient descent */
    static int32_t norm = 1000;
    int32_t norm_squared = (est_heading[0]*est_heading[0]) + (est_heading[1]*est_heading[1]);
    int32_t norm_squared_error = HEADING_NORM_SQUARED - norm_squared;
    norm += norm_squared_error*NORM_DESCENT_GAIN;
    est_heading[0] = est_heading[0] * 1000 / norm;
    est_heading[1] = est_heading[1] * 1000 / norm;


    /**
     * Position Estimation
     */
    est_V = (v_wheels[0] + v_wheels[1]) / 2;
    est_pos[0] += ((int32_t)est_V) * est_heading[0] / EST_FREQ / 1000;
    est_pos[1] += ((int32_t)est_V) * est_heading[1] / EST_FREQ / 1000;
}