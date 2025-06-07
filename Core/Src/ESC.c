/**
  ******************************************************************************
  * @file    ESC.c
  * @author  Aaron Lubinsky
  * @brief   ESC driver
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================



  */

#include "ESC.h"
#include "stm32f4xx_hal.h"   // Needed for HAL types
#include <stdint.h>
#include <stdio.h>
extern TIM_HandleTypeDef htim3;
extern int effort_set, K_effort;
extern int effortRate;
// PID Constants
extern int32_t Kp_roll, Ki_roll, Kd_roll;
extern int32_t Kp_pitch, Ki_pitch, Kd_pitch;
extern int32_t Kp_yaw, Ki_yaw, Kd_yaw;

// PID States
extern int32_t roll_set, roll_true, roll_error, roll_integral, roll_derivative, last_roll_error;
extern int32_t pitch_set, pitch_true, pitch_error, pitch_integral, pitch_derivative, last_pitch_error;
extern int32_t yaw_set, yaw_true, yaw_error, yaw_integral, yaw_derivative, last_yaw_error;
extern int32_t roll_effort, pitch_effort, yaw_effort;       // output of control loop

// PID Scaling
#define PID_SCALE 100000

int motA_offset = 960;
int motB_offset = 1230;
int motC_offset = 960;
int motD_offset = 980;

int A, B, C, D = 0;
int armCompare = 0;
int  max_integral = 100000;

void armESC()
{
 effort_set = 1000;
	while(roll_set < 150000){
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	    // Step 1: Send 960 (approx. 1000us) to arm
	    armCompare = effort_set*4 - 2000;
	    if (armCompare < 960) armCompare = 960;
	    if (armCompare > 2000) armCompare = 2000;
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, armCompare);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, armCompare);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, armCompare);
	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, armCompare);

	    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);  // Change GPIOA and PIN as needed
	    HAL_Delay(125);
    }
   effort_set = 0;
   armCompare = 0;
}




void update_Motors(){ //Compare 960 = 1ms (0%)    Compare 2000 = 2ms (100%)
	//  Roll
	int roll_error = -roll_set + roll_true;
	roll_integral += roll_error;
	roll_derivative = roll_error - last_roll_error;

	///TESTING ONLY
	roll_effort = -(Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative) / PID_SCALE;
	roll_effort = 0;
	/////END TESTING
	last_roll_error = roll_error;

	// Pitch

	///TESTING ONLY
	pitch_set = 0;
	/////END TESTING
	int pitch_error = -pitch_set + pitch_true;

	pitch_integral += pitch_error/1000;
	if (pitch_integral > max_integral) {
	    pitch_integral = max_integral;
	} else if (pitch_integral < -max_integral) {
	    pitch_integral = -max_integral;
	}
	pitch_derivative = pitch_error - last_pitch_error;
	pitch_effort = -(Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative) / PID_SCALE;
	last_pitch_error = pitch_error;

	//  Yaw
	int yaw_error = -yaw_set + yaw_true;
	yaw_integral += yaw_error;
	yaw_derivative = yaw_error - last_yaw_error;
	yaw_effort = -(Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_derivative) / PID_SCALE;
	last_yaw_error = yaw_error;
	//printf("Errors -> Roll, Pitch, Yaw,  %d,%d,%d \r\n", roll_error, pitch_error, yaw_error);


	// Start with base throttle
	A = effort_set * K_effort/PID_SCALE + motA_offset;
	B = effort_set * K_effort/PID_SCALE + motB_offset;
	C = effort_set * K_effort/PID_SCALE + motC_offset;
	D = effort_set * K_effort/PID_SCALE + motD_offset;

	// Pitch: affects A, B, C
	if (pitch_effort > 0) {
	    A += pitch_effort;
	    D += pitch_effort;
	}
	else if (pitch_effort < 0) {
	    B -= pitch_effort;
	    C -= pitch_effort;
	}

	// Roll: affects B, C
	if (roll_effort > 0) {
	    D += roll_effort;
	    C += roll_effort;
	}
	else if (roll_effort < 0) {
	    B -= roll_effort;
	    A -= roll_effort;
	}

	// Yaw: affects A, C
	if (yaw_effort > 0) {
	    B += yaw_effort;
	    D += yaw_effort;
	}
	else if (yaw_effort < 0) {
	    D -= yaw_effort;
	    C -= yaw_effort;
	}

    // Clamp between 960 (0%) and 1500 (appx 50%)
    if (A < 960) A = 960;
    if (A > 1500) A = 1500;

    if (B < 960) B = 960;
    if (B > 1500) B = 1500;

    if (C < 960) C = 960;
    if (C > 1500) C = 1500;

    if (D < 960) D = 960;
    if (D > 1500) D = 1500;


     //printf("%d,%d,%d,%d \r\n", pitch_true, 0, 0, 0);




	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, A);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, B);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, C);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, D);

}
