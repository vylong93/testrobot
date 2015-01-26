/*
 * custom_led.h
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_LED_H_
#define CUSTOM_LED_H_

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

#define LED_PORT_CLOCK 		SYSCTL_PERIPH_GPIOF
#define LED_PORT_BASE 		GPIO_PORTF_BASE
#define LED_RED   			GPIO_PIN_1
#define LED_BLUE  			GPIO_PIN_2
#define LED_GREEN 			GPIO_PIN_3
#define LED_ALL				(LED_RED | LED_GREEN | LED_BLUE)

//*****************************************************************************
//
// Configure 3 LEDs output at PF1, PF2 and PF3
//
//*****************************************************************************
void initLeds(void);

void turnOnLED(uint8_t LEDpin);
void turnOffLED(uint8_t LEDpin);
void toggleLED(uint8_t LEDpin);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_LED_H_ */
