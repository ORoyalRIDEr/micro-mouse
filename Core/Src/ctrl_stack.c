#include <ctrl_stack.h>
#include <Drivers/lre_stepper.h>
#include <Drivers/HCSR04.h>
#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>

#define SUB_CTRL_PRESCALER (MAIN_CTRL_FREQ/SUB_CTRL_FREQ)

enum ctrl_modes_t ctrl_mode = CTRL_BASE;

/* this function is called at MAIN_CTRL_FREQ */
void ctrl_callback ()
{
    static uint16_t main_ctrl_counter = 0;
    main_ctrl_counter++;
    if (main_ctrl_counter == SUB_CTRL_PRESCALER) {
        main_ctrl_counter = 0;
        /* this function is called at SUB_CTRL_FREQ */

        HCSR04_Measure();
        estimator_callback();

        if (ctrl_mode >= CTRL_ORIENTATION)
            orientation_ctrl_callback();
    }

    engine_timer_callback();
}

void ctrl_set_mode (enum ctrl_modes_t mode)
{
    ctrl_mode = mode;
}