/*
 * robot_speaker.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_speaker.h"

#include "pwm_definition.h"

void initSpeaker(void)
{
	ROM_SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	ROM_SysCtlDelay(2);

	ROM_SysCtlPeripheralEnable(SPEAKER_PWM_CLOCK_BASE);
	ROM_SysCtlDelay(2);

	ROM_SysCtlPeripheralEnable(SPEAKER_PORT_CLOCK);
	ROM_SysCtlDelay(2);

	if ((SPEAKER_PORT_BASE == GPIO_PORTF_BASE) && (SPEAKER_PIN == GPIO_PIN_0))
	{
		// unlock the GPIO commit control register to modify PF0 configuration because it may be configured to be a NMI input.
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	}

	uint32_t pwmClock = ROM_SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	uint32_t pwmPeriod = (pwmClock / SPEAKER_PWM_FREQUENCY);

	ROM_GPIOPinConfigure(SPEAKER_PWM_CONFIG);
	ROM_GPIODirModeSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_DIR_MODE_HW);
	ROM_GPIOPadConfigSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_STRENGTH_8MA,
	GPIO_PIN_TYPE_STD);

	ROM_PWMGenConfigure(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, pwmPeriod);
	ROM_PWMPulseWidthSet(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT, pwmPeriod / 2);

	//
	// For persision delay purpose
	//
	initSpeakerTimerDelay();
}

void triggerSpeakerWithPreDelay(uint32_t ui32DelayUs)
{
	delay_us_speaker_timer(ui32DelayUs);

	ROM_PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, true);
	ROM_PWMGenEnable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);

	delay_us_speaker_timer(SPEAKER_GO_OFF_PERIOD_US);

	ROM_PWMGenDisable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	ROM_PWMSyncTimeBase(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN_BIT);
	ROM_PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, false);
}

void initSpeakerTimerDelay(void)
{
	ROM_SysCtlPeripheralEnable(SPEAKER_TIMER_DELAY_CLOCK);
	TimerClockSourceSet(SPEAKER_TIMER_DELAY_BASE, TIMER_CLOCK_SYSTEM);

	ROM_TimerConfigure(SPEAKER_TIMER_DELAY_BASE, TIMER_CFG_ONE_SHOT);

	ROM_TimerIntEnable(SPEAKER_TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);

	ROM_TimerIntClear(SPEAKER_TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);
}

void delay_us_speaker_timer(uint32_t ui32PeriodInUs)
{
	// NOTE: This function should only be used in robot_analog and this library.

	//
	// Skip for zero period
	//
	if (ui32PeriodInUs == 0)
		return;

	uint32_t ui32Status;
	uint32_t ui32DelayCycles = (ROM_SysCtlClockGet() / 1000000) * ui32PeriodInUs;

	//
	// Reset timer counter
	//
	ROM_TimerLoadSet(SPEAKER_TIMER_DELAY_BASE, TIMER_A, ui32DelayCycles);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(SPEAKER_TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Start timer
	//
	ROM_TimerEnable(SPEAKER_TIMER_DELAY_BASE, TIMER_A);

	while(true)
	{
		//
		// Get delay status
		//
		ui32Status = ROM_TimerIntStatus(SPEAKER_TIMER_DELAY_BASE, false);

		//
		// Check for delay timeout
		//
		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;
	}

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(SPEAKER_TIMER_DELAY_BASE, TIMER_TIMA_TIMEOUT);
}

