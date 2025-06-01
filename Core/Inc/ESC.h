/*
 * ESC.c
 *
 *  Created on: May 20, 2025
 *      Author: aaron
 */

#ifndef INC_ESC_C_
#define INC_ESC_C_

#include <stdint.h>
void update_Motors(int32_t roll_effort, int32_t pitch_effort, int32_t yaw_effort, int32_t effor_set);
void armESC(void);

#endif /* INC_ESC_C_ */
