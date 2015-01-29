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
#include "inc\hw_memmap.h"
#include "driverlib\rom.h"
#include "driverlib\sysctl.h"
#include "driverlib\eeprom.h"

#define EEPROM_ADDR_ROBOT_ID			0x0040

//#define EEPROM_ADDR_MOTOR_OFFSET		0x0044	// EEPROM_ADDR_ROBOT_ID + 4
//
//#define EEPROM_INTERCEPT				0x0048
//#define EEPROM_SLOPE					0x004C

void initEEPROM();
void writeToEEPROM();
void readFormEEPROM();
void setAddressEEPROM();

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_EEPROM_H_ */
