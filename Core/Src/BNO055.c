/**
  ******************************************************************************
  * @file    BNO055.c
  * @author  Aaron Lubinsky
  * @brief   BNO055 IMU driver
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================



  */
#include "BNO055.h"
#include "stm32f4xx_hal.h"   // Needed for HAL types

extern I2C_HandleTypeDef hi2c1;
static uint8_t calibData;


void BNO_Init(){
	uint8_t ndof_mode = 0x0C;
	uint8_t config_mode = 0x00;
	uint8_t successfulRead = false;
	uint8_t sampleData = 0x00;
	int calibrated = false;


	while(successfulRead == false){ //WAIT to verify IMU connection
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	HAL_Delay(1000);
	HAL_I2C_DeInit(&hi2c1);
	HAL_I2C_Init(&hi2c1);
	if (hi2c1.State == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, 0x00, 1, &sampleData, 1, 100); // 0xaA0 should be returned if receiving from IMU
	}
	if (sampleData == 0xa0){
		successfulRead = true;
	}


	}
	// Set to CONFIG mode
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &config_mode, 1, 100);
    HAL_Delay(25);

    //Set to NDOF mode
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &ndof_mode, 1, 100);
    HAL_Delay(25);


	//WAIT until IMU is callibrated

	while(calibrated == false){ //wait for imu to calibrate
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // Set PA0 High
		HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, BNO055_CALIB_STAT, 1, &calibData, 1, 100);
		if (((calibData >> 6) & 0x03) == 0x03){
			calibrated = true;

	}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   // Set PA0 High
}
}


void BNO_Read(int32_t *roll, int32_t *pitch, int32_t *yaw){
	uint8_t eulerData[6];
	int32_t rawYaw16;
	int32_t rawPitch16;
	int32_t rawRoll16;
	HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, BNO055_EULER_LSB, I2C_MEMADD_SIZE_8BIT, eulerData, 6, 100);

    rawYaw16   = (int16_t)((eulerData[1] << 8) | eulerData[0]);
    rawRoll16  = (int16_t)((eulerData[3] << 8) | eulerData[2]);
    rawPitch16 = (int16_t)((eulerData[5] << 8) | eulerData[4]);


    *yaw   = ((int32_t)rawYaw16 * 1000) / 16;
    *roll  = ((int32_t)rawRoll16 * 1000) / 16;
    *pitch = ((int32_t)rawPitch16 * 1000) / 16;
}

