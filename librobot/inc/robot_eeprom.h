/*
 * robot_eeprom.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_EEPROM_H_
#define ROBOT_EEPROM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

//#define PROGRAM_TABLE_TO_EEPROM

/* Block 1 */
#define EEPROM_ROBOT_ID_WORD_IDX		16		// 0x0040
#define EEPROM_INTERCEPT_WORD_IDX		17		// 0x0044
#define EEPROM_SLOPE_WORD_IDX			18		// 0x0048
#define EEPROM_RANDOM_W_WORD_IDX		19		// 0x004C

#define EEPROM_ROBOT_ID_ADDRESS			(EEPROM_ROBOT_ID_WORD_IDX << 2)
#define EEPROM_INTERCEPT_WORD_ADDRESS	(EEPROM_INTERCEPT_WORD_IDX << 2)
#define EEPROM_SLOPE_WORD_ADDRESS		(EEPROM_SLOPE_WORD_IDX << 2)
#define EEPROM_RANDOM_W_WORD_ADDRESS	(EEPROM_RANDOM_WORD_IDX << 2)

/* Block 2 - 3 - 4 - 5 - 6 - 7 */
#define EPPROM_SINE_TABLE_ADDRESS       0x0080

/* Block 8 - 9 - 10 - 11 - 12 - 13 */
#define EPPROM_ARC_SINE_TABLE_ADDRESS   0x0200

//#define EEPROM_ADDR_MOTOR_OFFSET	0x004C	// EEPROM_ADDR_ROBOT_ID + 4

void initEEPROM(void);
uint32_t getRobotIDInEEPROM(void);
bool getTDOAParameterInEEPROM(float* pfIntercept, float* pfSlope);

bool writeWordToEEPROM(uint32_t ui32WordIndex, uint32_t ui32Data);
uint32_t readWordFormEEPROM(uint32_t ui32WordIndex);

float EEPROM_calSin(float x);
float EEPROM_calCos(float x);
float EEPROM_calASin(float x);
float EEPROM_calACos(float x);

void programEEPROM(bool bIsUpdate);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_EEPROM_H_ */
