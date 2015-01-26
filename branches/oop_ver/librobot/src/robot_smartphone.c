/*
 * robot_smartphone.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_smartphone.h"

bool tryToGetMotorsParameterInEEPROM(int8_t *pi8Motor1Speed, int8_t *pi8Motor2Speed)
{
	uint32_t pui32Read[1];

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	*pi8Motor1Speed = (*pui32Read & 0xFF);
	*pi8Motor2Speed = (((*pui32Read) >> 8) & 0xFF);

	if(*pi8Motor1Speed < 0 || *pi8Motor1Speed > 99 || *pi8Motor2Speed < 0 || *pi8Motor2Speed > 99)
		return false;

	return true;
}

void goStraight()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(FORWARD, i8Motor1Speed, FORWARD, i8Motor2Speed);
	}
}

void goBackward()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(REVERSE, i8Motor1Speed, REVERSE, i8Motor2Speed);
	}
}

void spinClockwise()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(REVERSE, i8Motor1Speed, FORWARD, i8Motor2Speed);
	}
}

void spinCounterclockwise()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(FORWARD, i8Motor1Speed, REVERSE, i8Motor2Speed);
	}
}
