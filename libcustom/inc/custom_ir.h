/*
 * custom_ir.h
 *
 *  Created on: Apr 27, 2015
 *      Author: VyLong
 */

#ifndef LIBCUSTOM_SRC_CUSTOM_IR_H_
#define LIBCUSTOM_SRC_CUSTOM_IR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#define IRLED_PORT_CLOCK 		SYSCTL_PERIPH_GPIOD
#define IRLED_PORT_BASE 		GPIO_PORTD_BASE
#define IRLED_TRANSMITER   		GPIO_PIN_2

void initProximitySensor(void);

void turnOnIRLED(void);
void turnOffIRLED(void);
void toggleIRLED(void);

#ifdef __cplusplus
}
#endif


#endif /* LIBCUSTOM_SRC_CUSTOM_IR_H_ */
