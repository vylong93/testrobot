/*
 * robot_speaker.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_speaker.h"

inline void initSpeaker()
{
	SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(SPEAKER_PWM_CLOCK_BASE);

//  SysCtlPeripheralEnable(SPEAKER_PORT_CLOCK);
	SysCtlDelay(2);

	if ((SPEAKER_PORT_BASE == GPIO_PORTF_BASE) && (SPEAKER_PIN == GPIO_PIN_0))
	{
		// unlock the GPIO commit control register to modify PF0 configuration because it may be configured to be a NMI input.
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	}

	uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	uint32_t pwmPeriod = (pwmClock / SPEAKER_PWM_FREQUENCY);

	GPIOPinConfigure(SPEAKER_PWM_CONFIG);
	GPIODirModeSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_DIR_MODE_HW);
	GPIOPadConfigSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_STRENGTH_8MA,
	GPIO_PIN_TYPE_STD);

	PWMGenConfigure(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, pwmPeriod);
	PWMPulseWidthSet(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT, pwmPeriod / 2);

	SysCtlPeripheralEnable(SPEAKER_TIMER_CLOCK);
	TimerDisable(SPEAKER_TIMER_BASE, TIMER_A);
	TimerConfigure(SPEAKER_TIMER_BASE, TIMER_CFG_ONE_SHOT);
	TimerLoadSet(SPEAKER_TIMER_BASE, TIMER_A,
			(SysCtlClockGet() / SPEAKER_TIMER_FREQUENCY));
	IntMasterEnable();
	TimerIntEnable(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(SPEAKER_INT);
}

void startSpeaker()
{
	PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, true);
	PWMGenEnable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	TimerEnable(SPEAKER_TIMER_BASE, TIMER_A);
}

void SpeakerTimerIntHandler(void)
{
	TimerIntClear(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	PWMGenDisable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	PWMSyncTimeBase(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN_BIT);
	PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, false);
}
