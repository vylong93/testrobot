/*
 * custom_delay.c
 *
 *  Created on: Dec 31, 2014
 *      Author: VyLong
 */

#include "libcustom\inc\custom_led.h"
#include "libcustom\inc\custom_delay.h"

static bool g_bIsTimer1Busy = true;
static bool g_bIsTimer2Busy = true;

//-----------------------------------------------------------------------------
//  void initDelay(void)
//
//  DESCRIPTION:
//	initialize timer for delay purpose
//-----------------------------------------------------------------------------
void initDelay(void)
{
	// Non-interrupt timer delay and delay2
	ROM_SysCtlPeripheralEnable(DELAY_TIMER_CLOCK_NON_INT);
	TimerClockSourceSet(DELAY_TIMER_BASE_NON_INT, TIMER_CLOCK_SYSTEM);
	ROM_TimerConfigure(DELAY_TIMER_BASE_NON_INT, TIMER_CFG_ONE_SHOT);

	ROM_TimerConfigure(DELAY_TIMER_BASE_NON_INT,
	TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);

	ROM_TimerIntEnable(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntEnable(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	g_bIsTimer1Busy = false;
	g_bIsTimer2Busy = false;
}

//-----------------------------------------------------------------------------
//  void delay_unit(uint32_t period, delayunit_t unit)
//
//  DESCRIPTION:
//  delay with period [in unit]
//
//  ARGUMENTS:
//      uint32_t period
//          Number of delay unit
//      delayunit_t unit
//          Unit delay could be MILISECOND_DIV for ms_delay
//			or MICROSECOND_DIV for us_delay
//-----------------------------------------------------------------------------
void delay_unit(uint32_t period, delayunit_t unit)
{
	if (!g_bIsTimer1Busy)
	{
		//turnOnLED(LED_BLUE);
		delay1_unit(period, unit);
		//turnOffLED(LED_BLUE);
	}
	else if (!g_bIsTimer2Busy)
	{
		//turnOnLED(LED_GREEN);
		delay2_unit(period, unit);
		//turnOffLED(LED_GREEN);
	}
	else
	{
		//turnOnLED(LED_RED);
		ROM_SysCtlDelay(ROM_SysCtlClockGet() / (3 * (uint32_t)unit) * period);
		//turnOffLED(LED_RED);
	}
}
void delay1_unit(uint32_t period, delayunit_t unit)
{
	if (period == 0)
		return;
	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / (uint32_t)unit) * period;

	//
	// Reset timer counter
	//
	ROM_TimerLoadSet(DELAY_TIMER_BASE_NON_INT, TIMER_A, delayPeriod);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);

	//
	// Start timer
	//
	ROM_TimerEnable(DELAY_TIMER_BASE_NON_INT, TIMER_A);

	g_bIsTimer1Busy = true;

	while(1)
	{
		//
		// Get delay status
		//
		ui32Status = ROM_TimerIntStatus(DELAY_TIMER_BASE_NON_INT, false);

		//
		// Check for delay timeout
		//
		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;
	}

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);

	g_bIsTimer1Busy = false;
}
void delay2_unit(uint32_t period, delayunit_t unit)
{
	if (period == 0)
		return;
	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / (uint32_t)unit) * period;

	ROM_TimerLoadSet(DELAY_TIMER_BASE_NON_INT, TIMER_B, delayPeriod);

	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	ROM_TimerEnable(DELAY_TIMER_BASE_NON_INT, TIMER_B);

	g_bIsTimer2Busy = true;

	while(1)
	{
		ui32Status = ROM_TimerIntStatus(DELAY_TIMER_BASE_NON_INT, false);

		if (ui32Status & TIMER_TIMB_TIMEOUT)
			break;
	}

	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	g_bIsTimer2Busy = false;
}

//-----------------------------------------------------------------------------
//  void delay_ms(uint32_t period)
//
//  DESCRIPTION:
//  delay with period [in ms]
//
//  ARGUMENTS:
//      uint32_t period
//          Number of milisecond delay
//-----------------------------------------------------------------------------
void delay_ms(uint32_t period)
{
	delay_unit(period, MILISECOND_DIV);
}

//-----------------------------------------------------------------------------
//  void delay_us(uint32_t period)
//
//  DESCRIPTION:
//  delay with period [in us]
//
//  ARGUMENTS:
//      uint32_t period
//          Number of microsecond delay
//-----------------------------------------------------------------------------
void delay_us(uint32_t period)
{
	delay_unit(period, MICROSECOND_DIV);
}

//-----------------------------------------------------------------------------
//  void delay_ms(uint32_t period, bool (*pfnTask)(uint32_t lifeTime, , va_list argp), ...)
//
//  DESCRIPTION:
//  delay with period [in ms]
//
//  ARGUMENTS:
//      uint32_t period
//          Number of milisecond delay
//      bool (*pfnTask)(uint32_t lifeTime, va_list argp)
//			Task function being call in delay period.
//			WARRING!: This task is continuity called in delay period,
//		    so that it need to return as fast as possible!
//		...
//			Are the optional arguments, which depend on the pfnTask arguments
//
//  EXAMPLE:
// 		Gobal variables for DEBUG only, this counter should equal callTime
//		after task1 is kill:
//      	uint32_t counterTask1 = 0;
//
//		Task definition:
//			void task1(uint32_t lifeTime, va_list argp) // This va_list in order is <int><bool><int*>
//			{
//				int callTime = 4; // this task is run ~4 time, maybe 3
//				SysCtlDelay(SysCtlClockGet()/(3 * 1000) * lifeTime / callTime);
//
//				int a;
//				bool b;
//				int* c;
//
//				a = va_arg(argp, int);
//				b = va_arg(argp, bool);
//				c = va_arg(argp, int*);
//
//				a = 15;
//				b = false;
//				*c = 300;
//
//				return false;
//			}
//
//		In main function:
//			int l_a = 31; int l_c = 10; bool l_b = false;
//			uint32_t task1LifeTimeInMilisecond = 5000; // 5 sec for task 1
//			//--Before task call: l_a = 31, l_b = false; l_c = 10
//			delay_ms_with_task(task1LifeTimeInMilisecond, task1, l_a, l_b, &l_c);
//			//--After task call: l_a = 31, l_b = false; l_c = 300. Only l_c is changed
//-----------------------------------------------------------------------------
void delay_ms_with_task(uint32_t period, bool (*pfnTask)(uint32_t lifeTime, va_list argp), ...)
{
	if (period == 0)
		return;

	va_list argp;

    //
    // Start the varargs processing.
    //
	va_start(argp, pfnTask);

	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / 1000) * period;

	//
	// Reset timer counter
	//
	ROM_TimerLoadSet(DELAY_TIMER_BASE_NON_INT, TIMER_A, delayPeriod);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);

	//
	// Start timer
	//
	ROM_TimerEnable(DELAY_TIMER_BASE_NON_INT, TIMER_A);

	g_bIsTimer1Busy = true;

	while(1)
	{
		//
		// Get delay status
		//
		ui32Status = ROM_TimerIntStatus(DELAY_TIMER_BASE_NON_INT, false);

		//
		// Check for delay timeout
		//
		if (ui32Status & TIMER_TIMA_TIMEOUT)
		{
			break; // Delay expried
		}
		else
		{
			//
			// Task call, if task return true then terminate this delay
			//
		    if((*pfnTask)(period, argp))
		    	break;	// Delay terminal
		}
	}

    //
    // We're finished with the varargs now.
    //
	va_end(argp);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);

	g_bIsTimer1Busy = false;
}
void delay2_ms_with_task(uint32_t period, bool (*pfnTask)(va_list argp), ...)
{
	if (period == 0)
		return;

	va_list argp;

	va_start(argp, pfnTask);

	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / 1000) * period;

	ROM_TimerLoadSet(DELAY_TIMER_BASE_NON_INT, TIMER_B, delayPeriod);

	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	ROM_TimerEnable(DELAY_TIMER_BASE_NON_INT, TIMER_B);

	g_bIsTimer2Busy = true;

	while(1)
	{
		ui32Status = ROM_TimerIntStatus(DELAY_TIMER_BASE_NON_INT, false);

		if (ui32Status & TIMER_TIMB_TIMEOUT)
			break;

	    if((*pfnTask)(argp))
	    	break;
	}

	va_end(argp);

	ROM_TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMB_TIMEOUT);

	g_bIsTimer2Busy = false;
}
