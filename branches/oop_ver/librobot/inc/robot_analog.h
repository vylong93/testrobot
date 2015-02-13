/*
 * robot_analog.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_ANALOG_H_
#define ROBOT_ANALOG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc\hw_memmap.h"
#include "inc\hw_types.h"
#include "inc\hw_ints.h"
#include "inc\hw_nvic.h"
#include "inc\hw_gpio.h"
#include "inc\hw_adc.h"
#include "inc\hw_udma.h"
#include "driverlib\pin_map.h"
#include "driverlib\interrupt.h"
#include "driverlib\sysctl.h"
#include "driverlib\gpio.h"
#include "driverlib\adc.h"
#include "driverlib\udma.h"
#include "driverlib\rom.h"

#define DELAY_SAMPING_MICS_US	1000	// unit in microsecond

// For mics signal
#define NUMBER_OF_SAMPLE    300
#define SAMPLE_FREQUENCY    100000	// unit in Hz

#define ADC_PORT_CLOCK          	SYSCTL_PERIPH_GPIOE
#define ADC_PORT                	GPIO_PORTE_BASE
#define ADC0_IN                 	GPIO_PIN_1
#define ADC1_IN                 	GPIO_PIN_3
#define ADC_ADJACENT_PINS       	GPIO_PIN_2 | GPIO_PIN_0
#define ADC0_CHANNEL            	ADC_CTL_CH2
#define ADC1_CHANNEL            	ADC_CTL_CH0
#define ADC0_INT                	INT_ADC0SS3
#define ADC1_INT                	INT_ADC1SS3

#define ADC_TIMER_CLOCK         	SYSCTL_PERIPH_TIMER0
#define ADC_TIMER               	TIMER0_BASE

#define ADC_AVERAGING_FACTOR    	8
#define ADC_SEQUENCE_TYPE       	3
#define ADC_SEQUENCE_ADDRESS    	ADC_O_SSFIFO3
#define ADC0_DMA_CHANNEL       		UDMA_CHANNEL_ADC3
#define ADC1_DMA_CHANNEL        	UDMA_SEC_CHANNEL_ADC13
#define DMA_ADC0_CHANNEL        	UDMA_CH17_ADC0_3
#define DMA_ADC1_CHANNEL       		UDMA_CH27_ADC1_3
#define DISTANCE_SENSING_PRIORITY	0

// For battery monitoring
#define BATTERY_PORT_CLOCK				SYSCTL_PERIPH_GPIOD
#define BATTERY_PORT					GPIO_PORTD_BASE
#define BATTERY_IN						GPIO_PIN_1
#define BATTERY_CHANNEL 				ADC_CTL_CH6
#define BATTERY_MEASURENMENT_PRIORITY	1
#define ADC_BATT_BASE					ADC0_BASE
#define ADC_BATT_SEQUENCE_TYPE     	 	2	// IMPORTANCE note: make sure this sequence type NOT equal ADC_SEQUENCE_TYPE
#define ADC_BATT_SEQUENCE_ADDRESS    	ADC_O_SSFIFO2
#define	ADC_BATT_INT	 				INT_ADC0SS2
#define BATT_DMA_CHANNEL       			UDMA_CHANNEL_ADC2
#define DMA_BATT_CHANNEL        		UDMA_CH16_ADC0_2

// For random generator
#define ADC_RANDOM_GEN_BASE			    ADC1_BASE
#define ADC_RANDOM_GEN_SEQUENCE_TYPE	0
#define RANDOM_GEN_PRIORITY				1
#define RANDOM_GEN_CHANNEL				ADC1_CHANNEL
#define RANDOM_GEN_SEQUENCE_ADDRESS		ADC_O_SSFIFO0
#define RANDOM_GEN_DMA_CHANNEL			UDMA_SEC_CHANNEL_ADC10
#define DMA_RANDOM_GEN_CHANNEL			UDMA_CH24_ADC1_0
#define RANDOM_GEN_INT                	INT_ADC1SS0

uint8_t* getMicrophone0BufferPointer(void);
uint8_t* getMicrophone1BufferPointer(void);

void initPeripheralsForAnalogFunction(void);
void triggerSamplingMicSignalsWithPreDelay(uint32_t ui32DelayUs);

void triggerSamplingBatteryVoltage(bool bIsSendToHost);

float generateRandomFloatInRange(float min, float max);
void triggerGenerateRandomByte(void);

void ADC0IntHandler(void);
void ADC1IntHandler(void);
void BatterySequenceIntHandler(void);
void RandomGeneratorIntHandler(void);
void uDMAErrorHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_ANALOG_H_ */
