/*
 * robot_timer_delay.c
 *
 *  Created on: Jan 28, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_timer_delay.h"

void initRobotTimerDelay(void)
{
	ROM_SysCtlPeripheralEnable(TIMER_DELAY_CLOCK);
	TimerClockSourceSet(TIMER_DELAY_BASE, TIMER_CLOCK_SYSTEM);

	ROM_TimerConfigure(TIMER_DELAY_BASE, TIMER_CFG_ONE_SHOT);

	ROM_TimerIntEnable(TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerIntClear(TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);
}

void delay_timer_us(uint32_t period)
{
	delay_timer_unit(period, TIMER_MICROSECOND_DIV);
}

void delay_timer_ms(uint32_t period)
{
	delay_timer_unit(period, TIMER_MILISECOND_DIV);
}

void delay_timer_unit(uint32_t period, timerdelayunit_t unit)
{
	//
	// Skip for zero period
	//
	if (period == 0)
		return;

	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / (uint32_t)unit) * period;

	//
	// Reset timer counter
	//
	ROM_TimerLoadSet(TIMER_DELAY_BASE, TIMER_A, delayPeriod);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Start timer
	//
	ROM_TimerEnable(TIMER_DELAY_BASE, TIMER_A);

	while(1)
	{
		//
		// Get delay status
		//
		ui32Status = ROM_TimerIntStatus(TIMER_DELAY_BASE, false);

		//
		// Check for delay timeout
		//
		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;
	}

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);
}
