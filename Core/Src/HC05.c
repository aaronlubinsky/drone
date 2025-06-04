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

int badBTcount = 0;

void InitializeBT(void){
	//send message from HC05. Receive message from PC. Confirm and continue.
}

void processInput(char *charBuf, int32_t *roll, int32_t *pitch, int32_t *yaw, int32_t *effort){
	   int32_t LjoyX, LjoyY, RjoyX, LT, RT;

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

	    // Assign parsed values to output pointers
	    *roll   = LjoyX*180;
	    *pitch  = LjoyY*180;
	    *yaw    = RjoyX*180;
	    *effort = RT;
	   }

