/*
 * robot_eeprom.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_eeprom.h"

//static uint32_t g_ui32EEPROMAdderss;

void initEEPROM() {
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while (ROM_EEPROMInit() != EEPROM_INIT_OK)
		;
	ROM_EEPROMIntDisable(EEPROM_INT_PROGRAM);
}

void writeToEEPROM() {
//	uint32_t ui32WriteAddress;
//	uint32_t ui32WriteWord;
//
//	ui32WriteAddress = RF24_RX_buffer[1] << 24;
//	ui32WriteAddress = RF24_RX_buffer[2] << 16;
//	ui32WriteAddress = RF24_RX_buffer[3] << 8;
//	ui32WriteAddress |= RF24_RX_buffer[4];
//
//	ui32WriteAddress <<= 2;
//
//	ui32WriteWord = RF24_RX_buffer[5] << 24;
//	ui32WriteWord |= RF24_RX_buffer[6] << 16;
//	ui32WriteWord |= RF24_RX_buffer[7] << 8;
//	ui32WriteWord |= RF24_RX_buffer[8];
//
//	EEPROMProgramNonBlocking(ui32WriteWord, ui32WriteAddress);
//
//	if (EEPROMStatusGet() == 0)
//	{
//		EEPROMIntClear(EEPROM_INT_PROGRAM);
//	}
//	else
//	{
//		// EEPROM operation error occur...
//	}
}

void readWordFormEEPROM() {
//	uint8_t pui8ReadBuffer[4];
//	uint32_t pui32Read[1];
//
//	EEPROMRead(pui32Read, g_ui32EEPROMAdderss, sizeof(pui32Read));
//
//	pui8ReadBuffer[0] = *pui32Read;
//	pui8ReadBuffer[1] = (*pui32Read) >> 8;
//	pui8ReadBuffer[2] = (*pui32Read) >> 16;
//	pui8ReadBuffer[3] = (*pui32Read) >> 24;
//
//	sendDataToControlBoard(pui8ReadBuffer);
}

void setAddressEEPROM() {
//	turnOnLED(LED_GREEN);
//
//	g_ui32EEPROMAdderss = RF24_RX_buffer[1] << 24;
//	g_ui32EEPROMAdderss = RF24_RX_buffer[2] << 16;
//	g_ui32EEPROMAdderss = RF24_RX_buffer[3] << 8;
//	g_ui32EEPROMAdderss |= RF24_RX_buffer[4];
//	g_ui32EEPROMAdderss <<= 2;
//
//	turnOffLED(LED_GREEN);
}
