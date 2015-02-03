/*
 * custom_delay.h
 *
 *  Created on: Dec 31, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_DELAY_H_
#define CUSTOM_DELAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

typedef enum {
	MILISECOND_DIV = 1000,
	MICROSECOND_DIV = 1000000
} delayunit_t;

// Non-Interrtup Timer
#define DELAY_TIMER_CLOCK_NON_INT	SYSCTL_PERIPH_WTIMER0
#define DELAY_TIMER_BASE_NON_INT	WTIMER0_BASE

void initDelay(void);

void delay_unit(uint32_t period, delayunit_t unit);
void delay1_unit(uint32_t period, delayunit_t unit);
void delay2_unit(uint32_t period, delayunit_t unit);

void delay_ms(uint32_t period);
void delay_us(uint32_t period);

void delay_ms_with_task(uint32_t period, bool (*pfnTask)(uint32_t lifeTime, va_list argp), ...);
void delay_ms_with_task(uint32_t period, bool (*pfnTask)(uint32_t lifeTime, va_list argp), ...);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_DELAY_H_ */
