/*
 * robot_task_timer.h
 *
 *  Created on: Mar 10, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_TASK_TIMER_H_
#define ROBOT_TASK_TIMER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc\hw_memmap.h"
#include "inc\hw_types.h"
#include "driverlib\sysctl.h"
#include "driverlib\rom.h"
#include "driverlib\timer.h"
#include "timers_definition.h"

void initRobotTaskTimer(void);

void resetRobotTaskTimer(void);
void activeRobotTask(uint32_t ui32PeriodInMs, bool (*pfnTask)(va_list argp), ...);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TASK_TIMER_H_ */
