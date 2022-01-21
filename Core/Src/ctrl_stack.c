#include <ctrl_stack.h>
#include <Drivers/lre_stepper.h>
#include <Ecl/state_estimator.h>

#define SUB_CTRL_PRESCALER (MAIN_CTRL_FREQ/SUB_CTRL_FREQ)

/* this function is called at MAIN_CTRL_FREQ */
void ctrl_callback ()
{
    static uint16_t main_ctrl_counter = 0;
    main_ctrl_counter++;
    if (main_ctrl_counter == SUB_CTRL_PRESCALER) {
        main_ctrl_counter = 0;
        /* this function is called at SUB_CTRL_FREQ */
        estimator_callback();
    }

    engine_timer_callback();
}