#ifndef __TASK1_H
#define __TASK1_H

#include <stdint.h> // Allows use of standard integer types
#include <stdio.h>  // Allows the use of printf()

// A datatype for pointers to task state functions. Each task_fcn_t
// function must have no input arguments and have no return
typedef void (*state_fcn_t)(void);

// A datatype for a structure to hold task configuration and state.
// Additional fields can be added as desired.
typedef struct task_1_cfg
{
    int32_t     state;
    int32_t     runs;
    int32_t     num_states;
    state_fcn_t state_list[];
} task_1_cfg_t;

// A prototype for each function implemented in task_1.c
void task_1_initialize(void);
void task_1_run(void);
void task_1_state_0(void);
void task_1_state_1(void);
void task_1_state_2(void);

#endif // __TASK1_H
