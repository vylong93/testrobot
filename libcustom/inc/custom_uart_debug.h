/*
 * custom_uart.h
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */

#ifndef CUSTOM_UART_DEBUG_H_
#define CUSTOM_UART_DEBUG_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#define DEBUG_UTILS

#ifdef DEBUG_UTILS
extern void UARTprintf(const char *pcString, ...);
#define DEBUG_PRINT(x)		UARTprintf(x)
#define DEBUG_PRINTS(x, y)	UARTprintf(x, y)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTS(x, y)
#endif

#define UART_DEBUG_PERIPH			SYSCTL_PERIPH_UART0
#define UART_DEBUG_BASE				UART0_BASE
#define UART_DEBUG_PORT_PERIPH		SYSCTL_PERIPH_GPIOA
#define UART_DEBUG_PORT_BASE		GPIO_PORTA_BASE
#define UART_DEBUG_RX				GPIO_PA0_U0RX
#define UART_DEBUG_TX				GPIO_PA1_U0TX
#define UART_DEBUG_RX_PIN			GPIO_PIN_0
#define UART_DEBUG_TX_PIN			GPIO_PIN_1
#define UART_DEBUG_BAUDRATE			115200

void initUartDebug(void);

#ifdef DEBUG_UTILS

#else
void UART_DEBUG_send_char(const char Byte);
void UART_DEBUG_send_string(const char *sendbuf);
void UART_DEBUG_send_line(const char *sendbuf);

//char UART_DEBUG_recive_char(void);
//void UART_DEBUG_recive_string(char *reciveBuf,int num);

void UART_DEBUG_send_int64_t(int64_t temp,char on);
void UART_DEBUG_send_int32_t(int32_t temp,char on);
void UART_DEBUG_send_int16_t(int16_t temp,char on);
void UART_DEBUG_send_float(float temp,int64_t length);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_UART_DEBUG_H_ */
