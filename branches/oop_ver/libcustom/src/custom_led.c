/*
 * custom_led.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_led.h"

void initLeds(void)
{
	// Init status led port
	SysCtlPeripheralEnable(LED_PORT_CLOCK);
	SysCtlDelay(2);

	GPIOPinTypeGPIOOutput(LED_PORT_BASE, LED_ALL);
}

void turnOnLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = LEDpin;
}

void turnOffLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = 0x00;
}

void toggleLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) ^= 0xFF;
}
