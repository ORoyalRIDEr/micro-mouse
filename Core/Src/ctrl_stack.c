#include <ctrl_stack.h>

#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>
#include <Drivers/I3G4250D_gyro.h>

#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>
#include <Ecl/position_ctrl.h>
#include <Ecl/driver.h>

#define GYRO_PRESCALER (MAIN_CTRL_FREQ / GYRO_FREQ)
#define SUB_CTRL_PRESCALER (MAIN_CTRL_FREQ / SUB_CTRL_FREQ)
#define MODE_ACTIVE(mode) (ctrl_mode & (1 << mode))

uint32_t ctrl_mode = 0;
uint32_t cpu_usage = 0; // percent * 1000

/* this function is called at MAIN_CTRL_FREQ */
void ctrl_callback(TIM_HandleTypeDef *timer)
{
    static uint32_t main_ctrl_counter = 0;
    static uint32_t gyro_counter = 0;

    main_ctrl_counter++;
    gyro_counter++;

    if (gyro_counter == GYRO_PRESCALER) {
        I3G4250D_gyro_Measure();
        gyro_counter = 0;
    }
        

    if (main_ctrl_counter == SUB_CTRL_PRESCALER)
    {
        __HAL_TIM_SET_COUNTER(timer, 0);
        uint32_t loopstart = __HAL_TIM_GET_COUNTER(timer);

        main_ctrl_counter = 0;
        /* this function is called at SUB_CTRL_FREQ */
        estimator_callback();

        /*if (MODE_ACTIVE(EST_SLAM))
        {
            int32_t dist[4];
            HCSR04_Read(dist);
            slam(dist);
        }

        if (MODE_ACTIVE(CTRL_POS))
            pos_ctrl_callback();*/

        if (MODE_ACTIVE(CTRL_DRIVE))
        {
            int32_t dist[4];
            HCSR04_Read(dist);
            driver_callback(dist);
        }

        if (MODE_ACTIVE(CTRL_ORIENTATION))
            orientation_ctrl_callback();

        HCSR04_Measure(); // measure at the end of loop such that data is available at next loop

        uint32_t looptime = __HAL_TIM_GET_COUNTER(timer) - loopstart;
        cpu_usage = looptime * SUB_CTRL_FREQ * 100 /*%*/ * 1000 /*scaler*/ / 100000 /*s->us*/ ;        
    }
}

void ctrl_set_mode(enum ctrl_modes_t mode)
{
    ctrl_mode |= 1 << mode;
}
void ctrl_unset_mode(enum ctrl_modes_t mode)
{
    ctrl_mode &= ~(1 << mode);
}
uint32_t get_cpu_usage()
{
    return cpu_usage;
}