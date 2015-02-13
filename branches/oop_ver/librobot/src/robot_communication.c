/*
 * robot_communication.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_communication.h"
#include "librobot\inc\robot_lpm.h"
#include "librobot\inc\robot_speaker.h"
#include "librobot\inc\robot_analog.h"
#include "librobot\inc\robot_motor.h"
#include "librobot\inc\robot_eeprom.h"

#include "libcustom\inc\custom_led.h"
#include "libcustom\inc\custom_uart_debug.h"

#define Robot_TX
//#define Robot_RX

bool decodeBasicHostCommand(uint8_t ui8Cmd)
{
	switch (ui8Cmd)
	{
	case HOST_COMMAND_SLEEP:
		setCpuMode(CPU_MODE_SLEEP);
		break;

	case HOST_COMMAND_DEEP_SLEEP:
		setCpuMode(CPU_MODE_DEEP_SLEEP);
		break;

	case HOST_COMMAND_WAKE_UP:
		wakeUpFormLPM();
		break;

	case HOST_COMMAND_RESET:
		softReset();
		break;

	default:
		return false;
	}
	return true;
}

void decodeAdvanceHostCommand(uint8_t ui8Cmd, uint8_t* pui8MessageBuffer)
{
	switch (ui8Cmd)
	{
	case HOST_COMMAND_TEST_RF_TRANSMISTER:
		testRfTransmister(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_TEST_RF_RECEIVER:
		testRfReceiver(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_TOGGLE_ALL_STATUS_LEDS:
		DEBUG_PRINT("Test: Toggle all status leds\n");
		toggleLED(LED_ALL);
		break;

	case HOST_COMMAND_START_SAMPLING_MIC:
		DEBUG_PRINT("Start sampling two microphones !!!\n");
		triggerSamplingMicSignalsWithPreDelay(DELAY_SAMPING_MICS_US);
		break;

	case HOST_COMMAND_REQUEST_BATT_VOLT:
		DEBUG_PRINT("Sampling battery voltage\n");
		sendBatteryVoltageToHost();
		break;

	case HOST_COMMAND_START_SPEAKER:
#ifdef Robot_TX
		triggerSamplingMicSignalsWithPreDelay(0);	// DELAY_SAMPING_MICS_US
		triggerSpeakerWithPreDelay(60);				// DELAY_BEFORE_START_SPEAKER_US
#endif
#ifdef Robot_RX
		triggerSamplingMicSignalsWithPreDelay(0);
#endif
		break;

	case HOST_COMMAND_CHANGE_MOTORS_SPEED:
		modifyMotorsConfiguration(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_STOP_MOTOR_LEFT:
		MotorLeft_stop();
		break;

	case HOST_COMMAND_STOP_MOTOR_RIGHT:
		MotorRight_stop();
		break;

	case HOST_COMMAND_DATA_ADC0_TO_HOST:
		turnOnLED(LED_BLUE);
		if (sendDataToHost(getMicrophone0BufferPointer(), NUMBER_OF_SAMPLE * 2))
		{
			turnOffLED(LED_BLUE);
			DEBUG_PRINT("ADC0 data Transmitted!\n");
		}
		else
			DEBUG_PRINT("Failed to sent ADC0 data...\n");
		break;

	case HOST_COMMAND_DATA_ADC1_TO_HOST:
		turnOnLED(LED_BLUE);
		if (sendDataToHost(getMicrophone1BufferPointer(), NUMBER_OF_SAMPLE * 2))
		{
			turnOffLED(LED_BLUE);
			DEBUG_PRINT("ADC1 data Transmitted!\n");
		}
		else
			DEBUG_PRINT("Failed to sent ADC1 data...\n");
		break;

	case HOST_COMMAND_EEPROM_DATA_READ:
		transmitRequestDataInEeprom(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_EEPROM_DATA_WRITE:
		synchronousEepromData(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_EEPROM_DATA_READ_BULK:
		transmitRequestBulkDataInEeprom(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	case HOST_COMMAND_EEPROM_DATA_WRITE_BULK:
		writeBulkToEeprom(&pui8MessageBuffer[MESSAGE_DATA_START_IDX]);
		break;

	default:
		decodeBasicHostCommand(ui8Cmd);
		break;
	}
}

void testRfReceiver(uint8_t* pui8Data)
{
	DEBUG_PRINT("Test: Rf Receiver\n");

	uint32_t ui32TestDataSize = construct4Byte(pui8Data);

	turnOnLED(LED_GREEN);

	if (RfTryToCaptureRfSignal(1000000, checkForCorrectRxDataStream,
			ui32TestDataSize))
	{
		MessageHeader responseReader;
		responseReader.eMessageType = MESSAGE_TYPE_ROBOT_RESPONSE;
		responseReader.ui8Cmd = ROBOT_RESPONSE_OK;

		sendDataToHost((uint8_t *) (&responseReader), 2);

		turnOffLED(LED_GREEN);

		DEBUG_PRINT("Test RF Transmission RX: OK\n");
	}
	else
	{
		DEBUG_PRINT("Test RF Transmission RX: Connection failed...\n");
	}
}

bool checkForCorrectRxDataStream(va_list argp)
{
	//  ARGUMENTS:
	//		va_list argp
	//			This list containt one argument in order:
	//				1/ uint32_t ui32TestDataSize

	// Get the input arguments
	uint32_t ui32TestDataSize;
	ui32TestDataSize = va_arg(argp, uint32_t);

	uint16_t i;
	uint8_t ui8TestValue = 0;
	uint32_t ui32ReceivedDataSize = 0;
	uint8_t* pui8DataHolder = 0;

	if (Network_receivedMessage(&pui8DataHolder, &ui32ReceivedDataSize))
	{
		if (ui32ReceivedDataSize == ui32TestDataSize)
		{
			for (i = 0; i < ui32TestDataSize; i++)
			{
				if (ui8TestValue != pui8DataHolder[i])
				{
					DEBUG_PRINT(
							"checkForCorrectRxDataStream: Invalid data...\n");

					if (pui8DataHolder != 0)
						Network_deleteBuffer(pui8DataHolder);

					return false;
				}
				else
				{
					ui8TestValue++;
				}
			}

			return true;
		}
		else
		{
			DEBUG_PRINT("checkForCorrectRxDataStream: Invalid size...\n");
		}
	}
	else
	{
		DEBUG_PRINT("Network_receivedMessage: Timeout...\n");
	}

	if (pui8DataHolder != 0)
		Network_deleteBuffer(pui8DataHolder);

	return false;
}

void testRfTransmister(uint8_t* pui8Data)
{
	DEBUG_PRINT("Test: Rf Transmitter\n");

	uint32_t ui32TestDataSize = construct4Byte(pui8Data);

	uint16_t* pui16TestData = malloc(sizeof(uint16_t) * (ui32TestDataSize >> 1));

	uint16_t i;
	for (i = 0; i < (ui32TestDataSize >> 1); i++)
	{
		pui16TestData[i] = i;
	}

	turnOnLED(LED_GREEN);

	if (sendDataToHost((uint8_t *) pui16TestData, ui32TestDataSize))
	{
		turnOffLED(LED_GREEN);
		DEBUG_PRINT("Test RF Transmission TX: OK\n");
	}
	else
	{
		DEBUG_PRINT("Test RF Transmission TX: Connection failed...\n");
	}

	free(pui16TestData);
}

void sendBatteryVoltageToHost(void)
{
	turnOnLED(LED_GREEN);

	triggerSamplingBatteryVoltage(true);
}

void modifyMotorsConfiguration(uint8_t* pui8Data)
{
	DEBUG_PRINT("Configure motors\n");

	Motor_t mLeftMotor;
	Motor_t mRightMotor;

	mLeftMotor.eDirection = (e_MotorDirection)pui8Data[0];
	mLeftMotor.ui8Speed = pui8Data[1];

	mRightMotor.eDirection = (e_MotorDirection)pui8Data[2];
	mRightMotor.ui8Speed = pui8Data[3];

	configureMotors(mLeftMotor, mRightMotor);
}

void transmitRequestDataInEeprom(uint8_t* pui8Data)
{
	uint32_t ui32DataCount = pui8Data[0];

	DEBUG_PRINTS("Host request read %d word(s) in EEPROM\n", ui32DataCount);

	uint8_t ui8ResponseSize = ui32DataCount * 6 + 1;
	uint8_t* pui8ResponseBuffer = malloc(sizeof(uint8_t) * ui8ResponseSize);

	pui8ResponseBuffer[0] = ui32DataCount;

	uint16_t ui16WordIndex;
	uint32_t ui32Data;
	uint32_t ui32Pointer = 1;
	uint16_t i;
	for (i = 1; i <= ui32DataCount * 2; i += 2)
	{
		ui16WordIndex = (uint16_t)((pui8Data[i] << 8) | pui8Data[i + 1]);
		ui32Data = readWordFormEEPROM(ui16WordIndex);

		pui8ResponseBuffer[ui32Pointer++] = pui8Data[i];
		pui8ResponseBuffer[ui32Pointer++] = pui8Data[i + 1];
		parse32bitTo4Bytes(&pui8ResponseBuffer[ui32Pointer], ui32Data);
		ui32Pointer += 4;
	}

	turnOnLED(LED_GREEN);

	if (sendMessageToHost(MESSAGE_TYPE_ROBOT_RESPONSE, ROBOT_RESPONSE_OK, (uint8_t *) pui8ResponseBuffer, ui8ResponseSize))
	{
		turnOffLED(LED_GREEN);
		DEBUG_PRINT("Send eeprom data to host: OK!\n");
	}
	else
	{
		DEBUG_PRINT("Send eeprom data to host: Failed...\n");
	}

	free(pui8ResponseBuffer);
}

void synchronousEepromData(uint8_t* pui8Data)
{
	DEBUG_PRINT("Synchronous EEPROM Data\n");
	uint8_t ui8DataNum = pui8Data[0];
	uint32_t ui32WordIndex;
	uint32_t ui32Data;
	uint8_t i;
	for(i = 1; i <= ui8DataNum * 6; i += 6)
	{
		ui32WordIndex = (pui8Data[i] << 8) | pui8Data[i + 1];
		ui32Data = construct4Byte(&pui8Data[i + 2]);
		if(!writeWordToEEPROM(ui32WordIndex, ui32Data))
		{
			DEBUG_PRINT("Failed to write to EEPROM...\n");
			return;
		}
	}
	DEBUG_PRINT("EEPROM Data Synchronized!\n");
}

void writeBulkToEeprom(uint8_t* pui8Data)
{
	DEBUG_PRINT("Write Bulk to EEPROM\n");
	uint32_t ui32NumberOfBytes = pui8Data[0] * 4; // pui8Data[0] is number of Word
	uint32_t EEPROMStartAddress = construct4Byte(&pui8Data[1]);

	EEPROMProgram((uint32_t*)(&pui8Data[5]), EEPROMStartAddress, ui32NumberOfBytes);
	//!!! Returns only after all data has been written or an error occurs.
}

void transmitRequestBulkDataInEeprom(uint8_t* pui8Data)
{
	uint32_t ui32NumberOfBytes = pui8Data[0] * 4; // pui8Data[0] is number of Word
	uint32_t EEPROMStartAddress = construct4Byte(&pui8Data[1]);

	DEBUG_PRINTS("Host request read bulk %d word(s) in EEPROM\n", pui8Data[0);

	uint32_t ui32ResponseSize = 1 + 4 + ui32NumberOfBytes;
	uint8_t* pui8ResponseBuffer = malloc(sizeof(uint8_t) * ui32ResponseSize);

	// Number of words
	pui8ResponseBuffer[0] = pui8Data[0];

	// Start address
	parse32bitTo4Bytes(&pui8ResponseBuffer[1], EEPROMStartAddress);

	// Read bulk
	EEPROMRead((uint32_t*)(&pui8ResponseBuffer[5]), EEPROMStartAddress, ui32NumberOfBytes);

	turnOnLED(LED_GREEN);

	if (sendMessageToHost(MESSAGE_TYPE_ROBOT_RESPONSE, ROBOT_RESPONSE_OK, (uint8_t *) pui8ResponseBuffer, ui32ResponseSize))
	{
		turnOffLED(LED_GREEN);
		DEBUG_PRINT("Send eeprom bulk data to host: OK!\n");
	}
	else
	{
		DEBUG_PRINT("Send eeprom bulk data to host: Failed...\n");
	}

	free(pui8ResponseBuffer);
}

bool sendMessageToHost(e_MessageType eMessType, uint8_t ui8Command,
		uint8_t* pui8Data, uint32_t ui32Size)
{
	uint32_t i;
	bool bReturn;

	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32Size;

	uint8_t* puiMessageBuffer = malloc(ui32TotalSize);

	//TODO: construct MessageHeader instead
	puiMessageBuffer[MESSAGE_TYPE_IDX] = eMessType;
	puiMessageBuffer[MESSAGE_COMMAND_IDX] = ui8Command;

	// Fill data
	for (i = 0; i < ui32Size; i++)
	{
		puiMessageBuffer[i + MESSAGE_DATA_START_IDX] = pui8Data[i];
	}

	bReturn = sendDataToHost(puiMessageBuffer, ui32TotalSize);

	free(puiMessageBuffer);

	return bReturn;
}

bool sendDataToHost(uint8_t * pui8Data, uint32_t ui32Length)
{
	if (Network_sendMessage(RF_CONTOLBOARD_ADDR, pui8Data, ui32Length, true))
		return true;
	return false;
}

//=========================================================

void sendMessageToOneNeighbor(uint32_t neighborID, uint8_t * messageBuffer,
		uint32_t length)
{
//	uint8_t addr[3];
//
//	addr[2] = neighborID >> 16;
//	addr[1] = neighborID >> 8;
//	addr[0] = neighborID;
//	RF24_RX_setAddress(RF24_PIPE0, addr);
//	RF24_TX_setAddress(addr);
//
//	uint32_t pointer = 0;
//	uint32_t i;
//
//	RF24_RX_flush();
//	RF24_clearIrqFlag(RF24_IRQ_RX);
//	RF24_TX_activate();
//	RF24_RETRANS_setCount(RF24_RETRANS_COUNT15);
//	RF24_RETRANS_setDelay(RF24_RETRANS_DELAY_2000u);
//
//	disableRF24Interrupt();
//
//	RF24_clearIrqFlag(RF24_IRQ_RX);
//	GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);
//
//	while (1)
//	{
//		rfDelayLoop(DELAY_CYCLES_1MS5);
//
//		if (messageBuffer != RF24_TX_buffer)
//		{
//			for (i = 0; (i < length) && (i < 32); i++)
//			{
//				RF24_TX_buffer[i] = *(messageBuffer + pointer);
//				pointer++;
//			}
//		}
//		else
//		{
//			i = length;
//		}
//
//		RF24_TX_writePayloadAck(i, RF24_TX_buffer);
//
//		RF24_TX_pulseTransmit();
//
//		while (1)
//		{
//			if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
//			{
//				if (RF24_getIrqFlag(RF24_IRQ_TX))
//					break;
//				if (RF24_getIrqFlag(RF24_IRQ_MAX_RETRANS))
//				{
//					RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);
//
//					if (g_ui8ReTransmitCounter != 0) // software trigger reTransmit
//					{
//						g_ui8ReTransmitCounter++;
//					}
//
//					addr[2] = g_ui32RobotID >> 16;
//					addr[1] = g_ui32RobotID >> 8;
//					addr[0] = g_ui32RobotID;
//					RF24_RX_setAddress(RF24_PIPE0, addr);
//					RF24_RX_activate();
//
//					enableRF24Interrupt();
//
//					return false;
//				}
//				else
//				{
//					RF24_clearIrqFlag(RF24_IRQ_MASK);
//				}
//			}
//		}
//		RF24_clearIrqFlag(RF24_IRQ_TX);
//
//		if (length > 32)
//			length -= 32;
//		else
//		{
//			addr[2] = g_ui32RobotID >> 16;
//			addr[1] = g_ui32RobotID >> 8;
//			addr[0] = g_ui32RobotID;
//			RF24_RX_setAddress(RF24_PIPE0, addr);
//			RF24_RX_activate();
//
//			enableRF24Interrupt();
//
//			return true;
//		}
//	}
}

void broadcastLocalNeighbor(uint8_t* pData, uint8_t ui8Length)
{
//	// WARNING!: must call RF24_RX_activate() to switch back RX mode after this function servered
//
//	uint8_t addr[3];
//
//	RF24_TX_activate();
//
//	addr[2] = RF24_LOCAL_BOARDCAST_BYTE2;
//	addr[1] = RF24_LOCAL_BOARDCAST_BYTE1;
//	addr[0] = RF24_LOCAL_BOARDCAST_BYTE0;
//	RF24_TX_setAddress(addr);
//
//	RF24_TX_writePayloadNoAck(ui8Length, pData);
//
//	disableRF24Interrupt();
//
//	RF24_TX_pulseTransmit();
//
//	while (1)
//	{
//		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
//		{
//			if (RF24_getIrqFlag(RF24_IRQ_TX))
//				break;
//		}
//	}
//
//	RF24_clearIrqFlag(RF24_IRQ_TX);
//
//	enableRF24Interrupt();
}

