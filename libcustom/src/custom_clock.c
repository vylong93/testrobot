/*
 * custom_clock.c
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_clock.h"

void initSysClock(void)
{
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();

	// WARING!!!: if clock source > 50MHz, dmp_load_motion_driver_firmware() will fault
	// Because i2c_write() & i2c_read() have slow-rate responsiveness

	//
	// Set the system clock to run at 50Mhz off PLL with external crystal as
	// reference.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//
	// Set the system clock to run at 50Mhz off PLL with internal POSC
	// reference.
	//
	// SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);  // 50MHz
}
