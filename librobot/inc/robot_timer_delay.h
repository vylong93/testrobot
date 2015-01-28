/*
 * robot_timer_delay.h
 *
 *  Created on: Jan 28, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_TIMER_DELAY_H_
#define ROBOT_TIMER_DELAY_H_

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

#define TIMER_DELAY_CLOCK             SYSCTL_PERIPH_TIMER0
#define TIMER_DELAY_BASE              TIMER0_BASE

typedef enum {
	TIMER_MILISECOND_DIV = 1000,
	TIMER_MICROSECOND_DIV = 1000000
} timerdelayunit_t;

void initRobotTimerDelay(void);
void delay_timer_us(uint32_t period);
void delay_timer_ms(uint32_t period);
void delay_timer_unit(uint32_t period, timerdelayunit_t unit);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TIMER_DELAY_H_ */
