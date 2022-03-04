#ifndef CTRL_STACK_H
#define CTRL_STACK_H

#define MAIN_CTRL_FREQ  1000  // Hz
#define SUB_CTRL_FREQ   10    // Hz

/* These controllers can be added or removed individually */
enum ctrl_modes_t {CTRL_ORIENTATION, EST_SLAM, CTRL_EST_ALL=0xFFFFFFFF};

void ctrl_callback (void);
void ctrl_set_mode (enum ctrl_modes_t mode);
void ctrl_unset_mode (enum ctrl_modes_t mode);

#endif // CTRL_STACK_H