/*
 * BNO055.h
 *
 *  Created on: May 17, 2025
 *      Author: aaron
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_
#include "main.h"

void BNO_Init(void);
void BNO_Read(int32_t *roll, int32_t *pitch, int32_t *yaw);
#define BNO055_I2C_ADDR       (0x28 << 1)  // shift left for HAL compatibility

#define true 1
#define false 0

// Useful register addresses
#define BNO055_OPR_MODE_ADDR  0x3D
#define BNO055_EULER_LSB    0x1A
#define BNO055_CALIB_STAT 0x35


//defines
extern uint8_t ndof_buf;




#endif /* INC_BNO055_H_ */
