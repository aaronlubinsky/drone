/*
 * HC05.h
 *
 *  Created on: May 14, 2025
 *      Author: aaron
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_



#include <stdint.h> // Allows use of standard integer types
#include <stdio.h>  // Allows the use of printf()




// Prototype for motor object "method"
void InitializeBT(void);
void processInput(char *charBuf, int32_t *roll, int32_t *pitch, int32_t *yaw, int32_t *effort);
//void read(void);
//void write(void);

#endif /* INC_HC05_H_ */
