/*
 * custom_clock.c
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_clock.h"

void initSysClock(void)
{
	FPUEnable();
	FPULazyStackingEnable();

	// External Crystal 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

}
