/*
 * custom_uart_debug.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */
#include <stdint.h>
#include <stdbool.h>
#include "libcustom\inc\custom_uart_debug.h"

extern void UARTStdioConfig(uint32_t ui32PortNum, uint32_t ui32Baud, uint32_t ui32SrcClock);

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void initUartDebug(void)
{
#ifdef DEBUG_UTILS
	//
	// Enable UART peripheral
	//
	SysCtlPeripheralEnable(UART_DEBUG_PERIPH);

	//
	// Enable the GPIO Peripheral used by the UART.
	//
	SysCtlPeripheralEnable(UART_DEBUG_PORT_PERIPH);

	//
	// Configure GPIO Pins for UART mode.
	//
	GPIOPinConfigure(UART_DEBUG_RX);
	GPIOPinConfigure(UART_DEBUG_TX);
	GPIOPinTypeUART(UART_DEBUG_PORT_BASE, UART_DEBUG_RX_PIN | UART_DEBUG_TX_PIN);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART_DEBUG_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART0 for console I/O.
	//
	UARTStdioConfig(0, UART_DEBUG_BAUDRATE, 16000000);

	SysCtlDelay(SysCtlClockGet() / (3 * 1000)); // delay 1ms

	DEBUG_PRINT("init UART DEBUG: OK\n");
#endif
}

#ifdef DEBUG_UTILS

#else

void UART_DEBUG_send_char(const char Byte)
{
    UARTCharPut(UART_DEBUG_BASE, Byte);
}

void UART_DEBUG_send_string(const char *sendbuf)
{
	while(*sendbuf != '\0')
	{
		UART_DEBUG_send_char(*sendbuf);
		sendbuf++;
	}
}

void UART_DEBUG_send_line(const char *sendbuf)
{
	UART_DEBUG_send_string(sendbuf);
	UART_DEBUG_send_char('\r');
	UART_DEBUG_send_char('\n');
}

//char UART_DEBUG_recive_char(void)
//{
//	char Byte;
//	Byte = UARTCharGet(UART_DEBUG_BASE);
//	return Byte;
//}

//void UART1_recive_string(char *reciveBuf, int num)
//{
//	int i;
//	for(i=0; i<num;i++)
//		reciveBuf[i] = UART_DEBUG_recive_char();
//}

void UART_DEBUG_send_int64_t(int64_t temp, char on)
{
	int64_t temp_p = temp;

	if(temp_p < 0)
	{
		UART_DEBUG_send_char('-');
		temp_p = 0 - temp_p;
	}
	else if(temp_p > 0)
	{
		temp = temp_p / 10;
		UART_DEBUG_send_int64_t(temp, 0);
		UART_DEBUG_send_char(temp_p % 10 + '0');
	}
	else if(temp_p == 0)
	{
		if(on)
			UART_DEBUG_send_char('0');
	}
}

void UART_DEBUG_send_int32_t( int32_t temp, char on)
{
	UART_DEBUG_send_int64_t((int64_t)temp, on);
}

void UART_DEBUG_send_int16_t(int16_t temp, char on)
{
	UART_DEBUG_send_int64_t((int64_t)temp, on);
}

void UART_DEBUG_send_float(float temp, int64_t length)
{
	int64_t i;
	float num = temp;
	int64_t temp_p;

	if(num < 0)
	{
		UART_DEBUG_send_char('-');
		num = 0 - num;
	}

	temp_p = (int64_t)num;
	UART_DEBUG_send_int64_t(temp_p, 1);
	UART_DEBUG_send_char('.');

	for(i=0; i<length; i++)
	{
		UART_DEBUG_send_char(((int)(10*(num - temp_p)))%10 + '0');
		num = num*10;
		temp_p = (int64_t)num;
	}
}

#endif
