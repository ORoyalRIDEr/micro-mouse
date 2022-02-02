#ifndef CTRL_STACK_H
#define CTRL_STACK_H

#define MAIN_CTRL_FREQ  1000  // Hz
#define SUB_CTRL_FREQ   10    // Hz

enum ctrl_modes_t {CTRL_BASE, CTRL_ORIENTATION};

void ctrl_callback (void);
void ctrl_set_mode (enum ctrl_modes_t mode);

#endif // CTRL_STACK_H