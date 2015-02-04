/*
 * TM4C123_CC2500.h
 *
 *  Created on: Dec 31, 2014
 *      Author: VyLong
 */
#ifdef RF_USE_CC2500

#ifndef TM4C123_CC2500_H_
#define TM4C123_CC2500_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define CC2500_SPI_PORT_CLOCK     SYSCTL_PERIPH_GPIOA
#define CC2500_SPI_PORT           GPIO_PORTA_BASE
#define CC2500_SCK                GPIO_PIN_2
#define CC2500_CSN                GPIO_PIN_3
#define CC2500_MISO               GPIO_PIN_4
#define CC2500_MOSI               GPIO_PIN_5

#define CC2500_CSN_CONFIGURE      GPIO_PA3_SSI0FSS
#define CC2500_SCK_CONFIGURE      GPIO_PA2_SSI0CLK
#define CC2500_MISO_CONFIGURE     GPIO_PA4_SSI0RX
#define CC2500_MOSI_CONFIGURE     GPIO_PA5_SSI0TX

#define CC2500_INT_PORT_CLOCK     SYSCTL_PERIPH_GPIOA
#define CC2500_INT_PORT           GPIO_PORTA_BASE
#define CC2500_CCA_Pin            GPIO_PIN_6
#define CC2500_INT_Pin            GPIO_PIN_7
#define CC2500_INT                INT_GPIOA
#define CC2500_INT_Channel        GPIO_INT_PIN_7

#define CC2500_SPI_CLOCK          SYSCTL_PERIPH_SSI0
#define CC2500_SPI                SSI0_BASE
#define CC2500_SPI_BAUDRATE       8000000

void MCU_RF_InitSpiForRf();
void MCU_RF_ConfigIRQPin(bool enable);
void MCU_RF_WaitUs(unsigned int cycles);

bool MCU_RF_GetInterruptState(void);
void MCU_RF_EnableInterrupt(void);
void MCU_RF_DisableInterrupt(void);
bool MCU_RF_IsInterruptPinAsserted(void);
void MCU_RF_ClearIntFlag(void);
void MCU_RF_ClearPending(void);

void MCU_RF_SetCSN();
void MCU_RF_ClearCSN();
char MCU_RF_SendAndGetData(uint32_t inData);

void MCU_RF_WaitForCCxxxxReady(void);
void MCU_RF_WaitForIntGoHigh(void);
void MCU_RF_WaitForIntGoLow(void);
bool MCU_RF_IsCRCOK(void);
char MCU_RF_Strobe(char strobe);

#ifdef __cplusplus
}
#endif

#endif /* TM4C123_CC2500_H_ */

#endif
