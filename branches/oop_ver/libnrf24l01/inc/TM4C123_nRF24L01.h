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

#define RF24_GLOBAL_BOARDCAST_BYTE2	0xBE
#define RF24_GLOBAL_BOARDCAST_BYTE1	0xAD
#define RF24_GLOBAL_BOARDCAST_BYTE0	0xFF

#define RF24_LOCAL_BOARDCAST_BYTE2	0xBE
#define RF24_LOCAL_BOARDCAST_BYTE1	0xAD
#define RF24_LOCAL_BOARDCAST_BYTE0	0x00

// Settings for 50MHz System Clock
#define DELAY_CYCLES_5MS       250000
#define DELAY_CYCLES_1MS5      75000
#define DELAY_CYCLES_130US     6500
#define DELAY_CYCLES_15US      750

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
// There are two ways to implement RF24 interrupt handler:
// 1) (Recommend!) the interupt handler make a call to this function.
// 2) (Not recommend) put this directly to the NVIC table in the startup file.
extern void RF24_IntHandler(void);

// Init the spi peripheral used by the RF board
inline void initSpiForRF (bool interruptEnable);

// Pull the RF24_CSN pin to logic high
inline void setRfCSN();

// Pull the RF24_CSN pin to logic low
inline void clearRfCSN();

// Pull the RF24_CE pin to logic high
inline void setRfCE();

// Pull the RF24_CE pin to logic high
inline void clearRfCE();

// Send and get data through the RF24_SPI
char SPI_sendAndGetData (uint32_t inData);

// A delay loop used by RF24 driver (do not need to be accurate)
void rfDelayLoop (uint32_t delay);

// Return the current state of the CE pin.
// Should use a private variable to store CE state then return it
// through this function instead of reading pin value.
char getCEState();

void disableRF24Interrupt();

void enableRF24Interrupt();

#endif //_TM4C123_NRF24L01_H
