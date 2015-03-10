/*
 * robot_task_timer.c
 *
 *  Created on: Mar 10, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_task_timer.h"

uint32_t g_ui32LastDelayCycles = 0;

void initRobotTaskTimer(void)
{
	ROM_SysCtlPeripheralEnable(TASK_TIMER_CLOCK);

	TimerClockSourceSet(TASK_TIMER_BASE, TIMER_CLOCK_SYSTEM);

	ROM_TimerConfigure(TASK_TIMER_BASE, TIMER_CFG_ONE_SHOT);

	ROM_TimerIntEnable(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
}

void resetRobotTaskTimer(void)
{
	ROM_TimerDisable(TASK_TIMER_BASE, TIMER_A);

	ROM_TimerLoadSet(TASK_TIMER_BASE, TIMER_A, g_ui32LastDelayCycles);

	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerEnable(TASK_TIMER_BASE, TIMER_A);
}

void activeRobotTask(uint32_t ui32PeriodInMs, bool (*pfnTask)(va_list argp), ...)
{
	if (ui32PeriodInMs == 0)
		return;

	va_list argp;

	va_start(argp, pfnTask);

	uint32_t ui32Status;
	g_ui32LastDelayCycles = (ROM_SysCtlClockGet() / 1000) * ui32PeriodInMs;

	ROM_TimerLoadSet(TASK_TIMER_BASE, TIMER_A, g_ui32LastDelayCycles);

	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerEnable(TASK_TIMER_BASE, TIMER_A);

	while(true)
	{
		ui32Status = ROM_TimerIntStatus(TASK_TIMER_BASE, false);
		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;

	    if((*pfnTask)(argp))
	    	break;
	}

	va_end(argp);

	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
}




