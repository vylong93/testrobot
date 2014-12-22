/*
 * custom_StickTimer.c
 *
 *  Created on: Dec 22, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_stickTimer.h"
#include "libcustom\inc\custom_led.h"

volatile uint32_t g_ui32SysTickCount = 0;

void initSysTick(void)
{
	// Set the system tick to fire 1000 times per second.
	SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);
	SysTickIntEnable();
	SysTickEnable();
}

void SysTickHandler(void)
{
	g_ui32SysTickCount++;

	if (g_ui32SysTickCount >= SYSTICKS_PER_SECOND)
	{
		g_ui32SysTickCount = 0;
		toggleLED(LED_GREEN);
	}
}



