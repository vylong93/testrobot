/*
 * TM4C123_CC2500.h
 *
 *  Created on: Dec 31, 2014
 *      Author: VyLong
 */

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

inline void TI_CC_SetCSN();
inline void TI_CC_ClearCSN();

void TI_CC_Setup();
void TI_CC_ConfigIRQPin(bool isEnable);
char TI_CC_SendAndGetData(char inData);

void TI_CC_Wait(unsigned int);
void TI_CC_WaitForCCxxxxReady(void);
void TI_CC_WaitForIntGoHigh(void);
void TI_CC_WaitForIntGoLow(void);

void TI_CC_EnableInterrupt(void);
void TI_CC_DisableInterrupt(void);
void TI_CC_ClearIntFlag(void);
void TI_CC_ClearPending(void);
bool TI_CC_GetInterruptState(void);
bool TI_CC_IsInterruptPinAsserted(void);

bool TI_CC_IsCRCOK(void);

char TI_CC_Strobe(char strobe);

#ifdef __cplusplus
}
#endif

#endif /* TM4C123_CC2500_H_ */
