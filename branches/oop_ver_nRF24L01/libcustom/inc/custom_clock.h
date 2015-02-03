/*
 * custom_clock.h
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_CLOCK_H_
#define CUSTOM_CLOCK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"

//*****************************************************************************
//
// External Crystal 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
//
//*****************************************************************************
void initSysClock(void);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_CLOCK_H_ */
