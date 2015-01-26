/*
 * custom_StickTimer.h
 *
 *  Created on: Dec 22, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_STICKTIMER_H_
#define CUSTOM_STICKTIMER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

#define SYSTICKS_PER_SECOND 1000

void initSysTick(void);
void SysTickHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_STICKTIMER_H_ */
