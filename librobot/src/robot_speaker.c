/*
 * robot_speaker.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_speaker.h"
#include "libcustom\inc\custom_delay.h"

#include "librobot\inc\robot_timer_delay.h"

#include "pwm_definition.h"

void initSpeaker()
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
}

void triggerSpeakerWithPreDelay(uint32_t ui32DelayUs)
{
	delay_timer_us(ui32DelayUs);	// DELAY_BEFORE_START_SPEAKER_US

	ROM_PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, true);
	ROM_PWMGenEnable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);

	delay_timer_us(SPEAKER_GO_OFF_PERIOD_US);

	ROM_PWMGenDisable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	ROM_PWMSyncTimeBase(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN_BIT);
	ROM_PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, false);
}
