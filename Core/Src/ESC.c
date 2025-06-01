/*
 * ESC.c
 *
 *  Created on: May 20, 2025
 *      Author: aaron
 */


#include "ESC.h"
#include "stm32f4xx_hal.h"   // Needed for HAL types
#include <stdint.h>
extern TIM_HandleTypeDef htim3;
extern int effort_set;
int A, B, C, D = 0;
int armCompare;

void armESC(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	armCompare =  effort_set + 1000;
	    // Clamp between 3200 and 5000
	    if (armCompare < 960) armCompare = 960;
	    if (armCompare > 2000) armCompare = 20000;


		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, armCompare);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, armCompare);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, armCompare);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, armCompare);
	}



void update_Motors(int32_t roll_effort, int32_t pitch_effort, int32_t yaw_effort, int32_t effort_set){ //Compare 3200 = 1ms (0%)    Compare 6400 = 2ms (100%)
	A =  pitch_effort  -roll_effort  +yaw_effort*0  +effort_set*0.3 + 1200;
	B = pitch_effort  +roll_effort  -yaw_effort*0  +effort_set*0.3 + 1200;
	C =  -pitch_effort  +roll_effort  +yaw_effort*0  +effort_set*0.3 + 1200;
	D =  -pitch_effort  -roll_effort  -yaw_effort*0  +effort_set*0.3 + 1200;


    // Clamp between 3200 and 5000
    if (A < 960) A = 960;
    if (A > 1500) A = 1500;

    if (B < 960) B = 960;
    if (B > 1500) B = 1500;

    if (C < 960) C = 960;
    if (C > 1500) C = 1500;

    if (D < 960) D = 960;
    if (D > 1500) D = 1500;


	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, A);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, B);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, C);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, D);
}
