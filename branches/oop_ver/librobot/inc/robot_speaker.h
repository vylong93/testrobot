/*
 * robot_speaker.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_SPEAKER_H_
#define ROBOT_SPEAKER_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"

#define SPEAKER_PORT_BASE               GPIO_PORTF_BASE
#define SPEAKER_PORT_CLOCK              SYSCTL_PERIPH_GPIOF
#define SPEAKER_PIN                     GPIO_PIN_0
#define SPEAKER_PWM_CLOCK_BASE          SYSCTL_PERIPH_PWM1
#define SPEAKER_PWM_FREQUENCY           8000                    // tan so phat
#define SPEAKER_PWM_BASE                PWM1_BASE
#define SPEAKER_PWM_CONFIG              GPIO_PF0_M1PWM4
#define SPEAKER_PWM_GEN                 PWM_GEN_2
#define SPEAKER_PWM_GEN_BIT				PWM_GEN_2_BIT
#define SPEAKER_PWM_OUT                 PWM_OUT_4
#define SPEAKER_PWM_OUT_BIT             PWM_OUT_4_BIT
#define SPEAKER_TIMER_CLOCK             SYSCTL_PERIPH_TIMER1
#define SPEAKER_TIMER_BASE              TIMER1_BASE
#define SPEAKER_TIMER_FREQUENCY         4000                   // thoi gian phat
#define SPEAKER_INT                     INT_TIMER1A

inline void initSpeaker();
inline void startSpeaker();



#endif /* ROBOT_SPEAKER_H_ */
