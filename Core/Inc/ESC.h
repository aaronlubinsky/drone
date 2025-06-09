/**
 * @file ESC.h
 * @brief Motor ESC (Electronic Speed Controller) interface functions.
 *
 * This file declares functions used to arm and update ESCs
 * for quadcopter motor control.
 *
 * @author Aaron
 * @date May 20, 2025
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#include <stdint.h>

/**
 * @brief Updates the motor speeds based on control inputs.
 *
 * This function should be called in the main control loop
 * to continuously update motor PWM signals.
 */
void update_Motors(void);

/**
 * @brief Arms the ESCs with a minimum throttle signal.
 *
 * This function initializes the ESCs so they are ready to receive throttle commands.
 * It must be called before any attempt to update motor speeds.
 */
void armESC(void);

#define true 1  ///< Definition for boolean true
#define false 0 ///< Definition for boolean false

#endif /* INC_ESC_H_ */
