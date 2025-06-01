/*
 * task_1.c
 *
 *  Created on: May 14, 2025
 *      Author: aaron
 */

#include "task_1.h"

// A struct used to hold the configuration/state of the task
task_1_cfg_t task_1 = {.state = 0,
                      .runs = 0,
                      .num_states = 3,
                      .state_list = {&task_1_state_0,
                                     &task_1_state_1,
                                     &task_1_state_2}

};

void task_1_initialize(void)
{
    printf("Initializing Task 1\r\n");

}

void task_1_run(void)
{    // Check for a valid state
    if (task_1.state >= 0 && task_1.state < task_1.num_states)
    {
        // Index the list of state functions and then call the appropriate
        // method while passing in (this) task_1 object
        task_1.state_list[task_1.state]();

        // Count the run in the run counter
        task_1.runs++;
    }
    // Big problems if the state is invalid
    else
    {
        printf("Error - Undefined state %d requested in Task 1\r\n"
               "        Task 1 only has %d states.\r\n",
               task_1.state, task_1.num_states);
        while(1){}
    }


}

void task_1_state_0(void)
{
    printf("Inside Task 1 State 0\r\n");
    task_1.state = 1;
}

// A function to implement state 1 of the task
void task_1_state_1(void)
{
    printf("Inside Task 1 State 1\r\n");
    task_1.state = 2;
}

// A function to implement state 2 of the task
void task_1_state_2(void)
{
    printf("Inside Task 1 State 2\r\n");
    task_1.state = 0;
}
