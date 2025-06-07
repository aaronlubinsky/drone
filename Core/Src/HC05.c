/**
  ******************************************************************************
  * @file    HC05.c
  * @author  Aaron Lubinsky
  * @brief   ESC driver
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================



  */

#include "HC05.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"   // Needed for HAL types
#include "BNO055.h"

int badBTcount = 0;
int effortRate = 10;
extern UART_HandleTypeDef huart2;  // Your UART handle (adjust if it's huart1 etc.)



void InitializeBT(void){
	//send message from HC05. Receive message from PC. Confirm and continue.
}

void processInput(char *charBuf, int32_t *roll, int32_t *pitch, int32_t *yaw, int32_t *effort, int *dumpFlag){
	   int32_t LjoyX, LjoyY, RjoyX, LT, RT, ENTER;

	   // Skip pound sign if present
	       if (charBuf[0] == '#') {
	           charBuf++;  // move pointer to the next character
	       }else{
	    	   badBTcount++;
	    	   printf("Bad BT input (did not process): %d\r\n", badBTcount);
	    	   return;
	       }

	    char *token = strtok(charBuf, ",");
	    if (token) LjoyX = (int32_t)strtol(token, NULL, 10);

	    token = strtok(NULL, ",");
	    if (token) LjoyY = (int32_t)strtol(token, NULL, 10);

	    token = strtok(NULL, ",");
	    if (token) RjoyX = (int32_t)strtol(token, NULL, 10);

	    token = strtok(NULL, ",");
	    if (token) LT = (int32_t)strtol(token, NULL, 10);

	    token = strtok(NULL, ",");
	    if (token) RT = (int32_t)strtol(token, NULL, 10);

	    token = strtok(NULL, ",");
	    if (token) ENTER = (int32_t)strtol(token, NULL, 10);

	    // Assign parsed values to output pointers
	    *roll   = LjoyX*180;
	    *pitch  = LjoyY*180;
	    *yaw    = RjoyX*180;
	    *effort = *effort + (RT - LT)*effortRate/1000;
	    if (*effort < 0){
	    	*effort = 0;
	    }else if(*effort > 1000){
	    	*effort = 1000;
	    }

	    if (ENTER == 1){
	    *dumpFlag = 1;
	    }

	   }

void dumpBlackbox(void)
{
    char msg[64];
    for (uint16_t i = 0; i < sample_index; i++) {
        snprintf(msg, sizeof(msg), "%ld,%ld\r\n", blackbox[i].pitch, blackbox[i].roll);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
    memset(blackbox, 0, sizeof(blackbox));
    sample_index = 0;
}


