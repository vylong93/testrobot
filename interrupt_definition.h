/*
 * interrupt_definition.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef INTERRUPT_DEFINITION_H_
#define INTERRUPT_DEFINITION_H_

#include "inc/hw_ints.h"

// Priority level definition, High to Low:
#define LEVEL_0		0x00
#define LEVEL_1		0x20

#define LEVEL_2		0x40
#define LEVEL_3		0x60
#define LEVEL_4		0x80
#define LEVEL_5		0xA0
#define LEVEL_6		0xC0
#define LEVEL_7		0xE0

//#define PRIORITY_CC2500_IRQ		LEVEL_0 // CANNOT change this priority in here!
#define PRIORITY_DMA_BATT			LEVEL_0

#define PRIORITY_DMA_RANDOM_GEN		LEVEL_2
#define PRIORITY_DMA_MIC1			LEVEL_2
#define PRIORITY_DMA_MIC2			LEVEL_2

#define PRIORITY_MOTOR_TIMERA       LEVEL_3
#define PRIORITY_MOTOR_TIMERB       LEVEL_3

#define PRIORITY_ROBOT_RESPONSE		LEVEL_5

#define PRIORITY_LOW_POWER_MODE		LEVEL_7

#define INT_SW_TRIGGER_LPM				INT_I2C1
#define INT_SW_TRIGGER_ROBOT_RESPONSE	INT_I2C2


#endif /* INTERRUPT_DEFINITION_H_ */
