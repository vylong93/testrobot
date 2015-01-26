/*
 * custom_StickTimer.c
 *
 *  Created on: Dec 22, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_stickTimer.h"

uint32_t g_ui32SysTickCount = 0;

void initSysTick(void)
{
	// Set the system tick to fire 1000 times per second.
	ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
	SysTickIntRegister(SysTickHandler);
	ROM_SysTickIntEnable();
	ROM_SysTickEnable();
}

void SysTickHandler(void)
{
	g_ui32SysTickCount++;
}



