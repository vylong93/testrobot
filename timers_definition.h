/*
 * timers_definition.h
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#ifndef TIMERS_DEFINITION_H_
#define TIMERS_DEFINITION_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* custom_delay library */
#define DELAY_TIMER_CLOCK_NON_INT	SYSCTL_PERIPH_WTIMER1
#define DELAY_TIMER_BASE_NON_INT	WTIMER1_BASE

/* robot_analog library */
#define ADC_TIMER_CLOCK         	SYSCTL_PERIPH_TIMER0
#define ADC_TIMER               	TIMER0_BASE

/* robot_task_timer library */
#define TASK_TIMER_CLOCK           SYSCTL_PERIPH_TIMER1
#define TASK_TIMER_BASE            TIMER1_BASE

/* tm4c123_nrf24l01 library */
#define RF_TIMER_CLOCK         		SYSCTL_PERIPH_TIMER2
#define RF_TIMER               		TIMER2_BASE

/* speaker library */
#define SPEAKER_TIMER_DELAY_CLOCK	SYSCTL_PERIPH_TIMER3
#define SPEAKER_TIMER_DELAY_BASE	TIMER3_BASE

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_DEFINITION_H_ */
