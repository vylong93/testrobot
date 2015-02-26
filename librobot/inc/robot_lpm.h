/*
 * robot_lpm.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_LPM_H_
#define ROBOT_LPM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc\hw_memmap.h"
#include "inc\hw_types.h"
#include "inc\hw_nvic.h"
#include "driverlib\rom.h"
#include "driverlib\sysctl.h"
#include "driverlib\interrupt.h"
#include "interrupt_definition.h"

typedef enum
{
  CPU_MODE_RUN,
  CPU_MODE_SLEEP,
  CPU_MODE_DEEP_SLEEP
} e_CpuMode;

void setCpuMode(e_CpuMode eMode);
e_CpuMode getCpuMode(void);

void softReset(void);
void returnToSleep(void);

void LowPowerModeIntHandler(void);

//-----------------------------------------------------------------------------
//  void initLowPowerMode()
//
//  DESCRIPTION:	This function configure clock system when enter Deep Sleep mode
//					and enable some necessary peripherals keep operate in Sleep/Deep Sleep mode.
//					Flash and SRAM power consumption on each state also modified.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
void initLowPowerMode();

//-----------------------------------------------------------------------------
//  void gotoSleepMode()
//
//  DESCRIPTION: 	Switch clock to PIOSC and power down the MOSC before going into Sleep.
//					This will be the Run mode's clock configuration after wake up form Sleep mode.
// 					So that reconfigure system clock should be considered if required.
//					After all, enter Sleep Mode.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
void gotoSleepMode();

//-----------------------------------------------------------------------------
//  void gotoDeepSleepMode()
//
//  DESCRIPTION:	This function forces CPU enter Deep Sleep Mode.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
void gotoDeepSleepMode();

//-----------------------------------------------------------------------------
//  void wakeUpFormLPM()
//
//  DESCRIPTION:	Reconfigure System clock before return normal operation.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
void wakeUpFormLPM();


#ifdef __cplusplus
}
#endif

#endif /* ROBOT_LPM_H_ */
