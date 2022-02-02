#include <Ecl/state_estimator.h>
#include <Drivers/lre_stepper.h>
#include <ctrl_stack.h>
#include <Lib/printf.h>
#include <Lib/cmath.h>

// Micro meters per engine step
#define UM_PER_ENGINE_STEP    106
// Inverse of distance of wheels in 1/m
#define D_WHEELS_INV    25/2
#define EST_FREQ   SUB_CTRL_FREQ

int32_t est_pos[] = {0, 0};
int32_t est_V = 0;
int32_t est_Psi = 0; // 1000 rad

void get_state(int32_t pos[], int32_t* V, int32_t* Psi)
{
    pos[0] = est_pos[0];
    pos[1] = est_pos[1];
    *V = est_V;
    *Psi = est_Psi;
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