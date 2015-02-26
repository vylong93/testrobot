/*
 * pwm_definition.h
 *
 *  Created on: Jan 28, 2015
 *      Author: VyLong
 */

#ifndef PWM_DEFINITION_H_
#define PWM_DEFINITION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "driverlib/sysctl.h"

#define PWM_CLOCK_SELECT        SYSCTL_PWMDIV_16
#define PWM_CLOCK_PRESCALE      16 // Must match with PWM_CLOCK_SELECT

#ifdef __cplusplus
}
#endif

#endif /* PWM_DEFINITION_H_ */
