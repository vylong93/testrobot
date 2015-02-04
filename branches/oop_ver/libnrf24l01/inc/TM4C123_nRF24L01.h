#ifdef RF_USE_nRF24L01

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"
#include "libnrf24l01/inc/nRF24L01.h"

#ifndef _TM4C123_NRF24L01_H
#define _TM4C123_NRF24L01_H

#define RF24_SPI_PORT_CLOCK     SYSCTL_PERIPH_GPIOA
#define RF24_SPI_PORT           GPIO_PORTA_BASE
#define RF24_SCK                GPIO_PIN_2
#define RF24_CSN                GPIO_PIN_3
#define RF24_MISO               GPIO_PIN_4
#define RF24_MOSI               GPIO_PIN_5

#define RF24_CSN_CONFIGURE      GPIO_PA3_SSI0FSS
#define RF24_SCK_CONFIGURE      GPIO_PA2_SSI0CLK
#define RF24_MISO_CONFIGURE     GPIO_PA4_SSI0RX
#define RF24_MOSI_CONFIGURE     GPIO_PA5_SSI0TX

#define RF24_INT_PORT_CLOCK     SYSCTL_PERIPH_GPIOA
#define RF24_INT_PORT           GPIO_PORTA_BASE
#define RF24_CE                 GPIO_PIN_6
#define RF24_INT_Pin            GPIO_PIN_7
#define RF24_INT                INT_GPIOA
#define RF24_INT_Channel        GPIO_INT_PIN_7

#define RF24_SPI_CLOCK          SYSCTL_PERIPH_SSI0
#define RF24_SPI                SSI0_BASE
#define RF24_SPI_BAUDRATE       2000000

void MCU_RF_InitSpiForRf(void);
void MCU_RF_ConfigIRQPin(bool enable);
void MCU_RF_WaitUs(unsigned int cycles);

bool MCU_RF_GetInterruptState(void);
void MCU_RF_EnableInterrupt(void);
void MCU_RF_DisableInterrupt(void);
bool MCU_RF_IsInterruptPinAsserted(void);
void MCU_RF_ClearIntFlag(void);
void MCU_RF_ClearPending(void);

void MCU_RF_SetCSN();	// Pull the RF24_CSN pin to logic high
void MCU_RF_ClearCSN();	// Pull the RF24_CSN pin to logic low
void MCU_RF_SetCE();	// Pull the RF24_CE pin to logic high
void MCU_RF_ClearCE();	// Pull the RF24_CE pin to logic high
char MCU_RF_SendAndGetData (uint32_t inData);	// Send and get data through the RF24_SPI

// Return the current state of the CE pin.
// Should use a private variable to store CE state then return it
// through this function instead of reading pin value.
char MCU_RF_GetCEState();

#endif //_TM4C123_NRF24L01_H

#ifdef __cplusplus
}
#endif

#endif
