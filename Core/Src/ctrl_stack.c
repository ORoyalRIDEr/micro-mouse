#include <ctrl_stack.h>

#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>

#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>
#include <Ecl/position_ctrl.h>

#define SUB_CTRL_PRESCALER (MAIN_CTRL_FREQ / SUB_CTRL_FREQ)
#define MODE_ACTIVE(mode) (ctrl_mode & (1 << mode))

uint32_t ctrl_mode = 0;

/* this function is called at MAIN_CTRL_FREQ */
void ctrl_callback()
{
    static uint16_t main_ctrl_counter = 0;
    main_ctrl_counter++;
    if (main_ctrl_counter == SUB_CTRL_PRESCALER)
    {
        main_ctrl_counter = 0;
        /* this function is called at SUB_CTRL_FREQ */
        estimator_callback();

        if (MODE_ACTIVE(EST_SLAM))
        {
            int32_t dist[4];
            HCSR04_Read(dist);
            slam(dist);
        }

        if (MODE_ACTIVE(CTRL_POS))
            pos_ctrl_callback();

        if (MODE_ACTIVE(CTRL_ORIENTATION))
            orientation_ctrl_callback();

        HCSR04_Measure(); // measure at the end of loop such that data is available at next loop
    }

    engine_timer_callback();
}

void ctrl_set_mode(enum ctrl_modes_t mode)
{
    ctrl_mode |= 1 << mode;
}
void ctrl_unset_mode(enum ctrl_modes_t mode)
{
    ctrl_mode &= ~(1 << mode);
}