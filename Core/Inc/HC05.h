/**
 * @file HC05.h
 * @brief Bluetooth HC-05 module interface for receiving control input.
 *
 * This file contains functions to process incoming UART data
 * and handle blackbox data dumping for debugging purposes.
 *
 * @author Aaron
 * @date May 14, 2025
 */

#ifndef INC_HC05_H_
#define INC_HC05_H_

#include <stdint.h> ///< Standard integer types
#include <stdio.h>  ///< For printf() debugging

/**
 * @brief Parses input from the HC-05 Bluetooth module.
 *
 * @param charBuf Pointer to the received character buffer.
 * @param roll Pointer to variable storing parsed roll value.
 * @param pitch Pointer to variable storing parsed pitch value.
 * @param yaw Pointer to variable storing parsed yaw value.
 * @param effort Pointer to variable storing parsed throttle/effort value.
 * @param dumpFlag Pointer to a flag indicating whether to dump blackbox data.
 */
void processInput(char *charBuf, int32_t *roll, int32_t *pitch, int32_t *yaw, int32_t *effort, int *dumpFlag);

/**
 * @brief Outputs the blackbox flight data via UART.
 *
 * This function is used for offline analysis or debugging.
 */
void dumpBlackbox(void);

#endif /* INC_HC05_H_ */
