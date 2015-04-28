/*
 * custom_ir.c
 *
 *  Created on: Apr 27, 2015
 *      Author: VyLong
 */

#include "libcustom/inc/custom_ir.h"

void initProximitySensor(void)
{
	// init IR transmiter
	ROM_SysCtlPeripheralEnable(IRLED_PORT_CLOCK);
	ROM_SysCtlDelay(2);

	ROM_GPIOPinTypeGPIOOutput(IRLED_PORT_BASE, IRLED_TRANSMITER);

	// init IR receiver
}

void turnOnIRLED(void)
{
	ASSERT(_GPIOBaseValid(IRLED_PORT_BASE));
	HWREG(IRLED_PORT_BASE + (GPIO_O_DATA + (IRLED_TRANSMITER << 2))) = IRLED_TRANSMITER;
}

void turnOffIRLED(void)
{
	ASSERT(_GPIOBaseValid(IRLED_PORT_BASE));
	HWREG(IRLED_PORT_BASE + (GPIO_O_DATA + (IRLED_TRANSMITER << 2))) = 0x00;
}

void toggleIRLED(void)
{
	ASSERT(_GPIOBaseValid(IRLED_PORT_BASE));
	HWREG(IRLED_PORT_BASE + (GPIO_O_DATA + (IRLED_TRANSMITER << 2))) ^= 0xFF;
}



