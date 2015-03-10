/*
 * robot_process.c
 *
 *  Created on: Feb 25, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_process.h"
#include "librobot/inc/robot_communication.h"
#include "librobot/inc/robot_lpm.h"
#include "librobot/inc/robot_speaker.h"
#include "librobot/inc/robot_analog.h"
#include "librobot/inc/robot_motor.h"
#include "librobot/inc/robot_eeprom.h"

#include "libcustom/inc/custom_led.h"
#include "libcustom/inc/custom_delay.h"
#include "libalgorithm/inc/TDOA.h"
#include "libprotocol/inc/network.h"
#include "libstorage/inc/robot_data.h"
#include "librobot/inc/robot_task_timer.h"
#include "interrupt_definition.h"

e_RobotState g_eRobotState = ROBOT_STATE_IDLE;
e_RobotResponseState g_eRobotResponseState = ROBOT_RESPONSE_STATE_NONE;

uint8_t* g_pui8RequestData;

void initRobotProcess(void)
{
	uint32_t ui32ReadEEPROMData;
	float fTDOA_Intercept;
	float fTDOA_Slope;

	//
	// Initilize self ID in eeprom
	//
	ui32ReadEEPROMData = getRobotIDInEEPROM();
	if(ui32ReadEEPROMData != 0xFFFFFFFF)
	{
		Network_setSelfAddress(ui32ReadEEPROMData);
		DEBUG_PRINTS("set Network Self Address to 0x%08x\n", ui32ReadEEPROMData);
	}
	else
	{
		Network_setSelfAddress(RF_DEFAULT_ROBOT_ID);
		DEBUG_PRINTS("set Self Address to Default Address: 0x%08x\n", RF_DEFAULT_ROBOT_ID);
	}

	//
	// Initilize TDOA
	//
	if(getTDOAParameterInEEPROM(&fTDOA_Intercept, &fTDOA_Slope))
	{
		TDOA_setIntercept(fTDOA_Intercept);
		TDOA_setSlope(fTDOA_Slope);
	}
	TDOA_initFilters();

	//
	// Initilize Data Storages
	//
	initLinkedList();

	//==============================================
	// IMPORTANCE: Configure Software Interrupt
	//==============================================
	ROM_IntPrioritySet(INT_SW_TRIGGER_ROBOT_RESPONSE, PRIORITY_ROBOT_RESPONSE);

	ROM_IntEnable(INT_SW_TRIGGER_ROBOT_RESPONSE);

}

void setRobotState(e_RobotState eState)
{
	g_eRobotState = eState;
}
e_RobotState getRobotState(void)
{
	return g_eRobotState;
}

void setRobotResponseState(e_RobotResponseState eState)
{
	g_eRobotResponseState = eState;
}
e_RobotResponseState getRobotResponseState(void)
{
	return g_eRobotResponseState;
}
uint8_t* getRequestMessageDataPointer(void)
{
	return g_pui8RequestData;
}

void triggerResponseState(e_RobotResponseState eResponse, uint8_t* pui8RequestData, uint32_t ui32DataSize)
{
	int32_t i;

	g_eRobotResponseState = eResponse;

	if(ui32DataSize > 0)
	{
		g_pui8RequestData = malloc(sizeof(uint8_t) * ui32DataSize);
		if(g_pui8RequestData == 0)
			return;

		// WARNING!: We must be copied the message content to another spaces. Because the network layer
		// will delete the current dynamic stogare after this function return!
		// Data at which pui8RequestData point to become invalid.
		// This new dynamic will be free at the end of RobotResponseIntHandler.
		for(i = 0; i < ui32DataSize; i++)
		{
			g_pui8RequestData[i] = pui8RequestData[i];
		}
	}
	else
	{
		g_pui8RequestData = 0;
	}

	IntTrigger(INT_SW_TRIGGER_ROBOT_RESPONSE);
}

//========= State 1 - Measure Distances ================================

bool g_bIsSuccessMeasuredDistances;

bool tryToRequestLocalNeighborsForDistanceMeasurement(void)
{
	//NOTE: i16Intercept and i16Slope is Fixed-point <6.10> format
	int16_t i16Intercept = (int16_t)(TDOA_getIntercept() * 1024 + 0.5);	// Fixed-point 6.10
	int16_t i16Slope = (int16_t)(TDOA_getSlope() * 1024 + 0.5);			// Fixed-point 6.10

	triggerSamplingMicSignalsWithPreDelay(0);
	while(!isSamplingCompleted());

	TDOA_filterSignalsMic(getMicrophone0BufferPointer());
	if(TDOA_isFilteredSignalNoisy())
		return false;

	TDOA_filterSignalsMic(getMicrophone1BufferPointer());
	if(TDOA_isFilteredSignalNoisy())
		return false;

	broadcastMeasureDistanceCommandToLocalNeighbors(ROBOT_REQUEST_SAMPLING_MICS, i16Intercept, i16Slope);
	triggerSpeakerWithPreDelay(DELAY_BEFORE_START_SPEAKER_US);

	return true;
}

void broadcastMeasureDistanceCommandToLocalNeighbors(uint8_t ui8Command, int16_t i16Intercept, int16_t i16Slope)
{
	//NOTE: i16Intercept and i16Slope is Fixed-point <6.10> format

	uint32_t ui32SelfId = Network_getSelfAddress();
	uint8_t pui8MessageData[8];	// <4-byte SelfId><2-byte Intercept><2-byte Slope>

	parse32bitTo4Bytes(pui8MessageData, ui32SelfId);

	pui8MessageData[4] = (uint8_t)(i16Intercept >> 8);
	pui8MessageData[5] = (uint8_t)(i16Intercept);

	pui8MessageData[6] = (uint8_t)(i16Slope >> 8);
	pui8MessageData[7] = (uint8_t)(i16Slope);

	broadcastToLocalNeighbors(ui8Command, pui8MessageData, 8);
}

void handleSamplingMicsRequest(uint8_t* pui8RequestData)
{
	bool bIsSkipTheRest = false;
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);
	float fPeakA, fMaxA;
	float fPeakB, fMaxB;

	bool bCurrentInterruptStage = MCU_RF_GetInterruptState();
	MCU_RF_DisableInterrupt();

	triggerSamplingMicSignalsWithPreDelay(0);
	while(!isSamplingCompleted())
	{
		if (MCU_RF_IsInterruptPinAsserted())
		{
			MCU_RF_ClearIntFlag(); //TODO: must be filter the received command

			reponseCommandToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_RESET_STATE_ONE);

			bIsSkipTheRest = true;
		}
	}

	if(!bIsSkipTheRest)
	{
		TDOA_process(getMicrophone0BufferPointer(), &fPeakA, &fMaxA);
		TDOA_process(getMicrophone1BufferPointer(), &fPeakB, &fMaxB);

		float fInterceptOfRequestRobot = (int16_t)((pui8RequestData[4] << 8) | pui8RequestData[5]) / 1024.0f;
		float fSlopeOfRequestRobot = (int16_t)((pui8RequestData[6] << 8) | pui8RequestData[7]) / 1024.0f;
		uint16_t ui16Distance = TDOA_calculateDistanceFromTwoPeaks(fPeakA, fPeakB, fInterceptOfRequestRobot, fSlopeOfRequestRobot);	// Fixed-point <8.8>

		addOverrideToNeighborsTable(ui32RequestRobotID, ui16Distance);

		// random 1->100: delay unit (5ms)
		uint32_t ui32RandomUs = (uint32_t)(generateRandomFloatInRange(1, 100) * 1000);
		delay_us(ui32RandomUs + 2000);

		responseDistanceToNeighbor(ui32RequestRobotID, ui16Distance);
	}

	// Because previous action may have assert interrupt flag
    MCU_RF_ClearIntFlag();
    MCU_RF_ClearPending();
	if (bCurrentInterruptStage)
		MCU_RF_EnableInterrupt();	// recover last interrupt state
}

bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	//NOTE: ui16Distance is Fixed-point <8.8> format

	uint8_t pui8ResponseData[6];	// <4-byte self id><2-byte distance>
	uint32_t ui32SelfId;

	ui32SelfId = Network_getSelfAddress();
	parse32bitTo4Bytes(pui8ResponseData, ui32SelfId);

	pui8ResponseData[4] = (uint8_t)(ui16Distance >> 8);
	pui8ResponseData[5] = (uint8_t)ui16Distance;

	return responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_DISTANCE_RESULT, pui8ResponseData, 6);
}

void StateOne_MeasureDistance(void)
{
	/* Pseudo-Code
	   clear isSuccessMeasureDistance flag
	   call Robot timer delay [DELAY_MEASURE_DISTANCE_STATE_US] with TASK(*isSuccessMeasureDistance)
	   {
	   	   DO
	   	   {
	   	   	   generate random values
			   clear isFlagAssert flag
			   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
				   handlerInDelayRandom()
				   -- {
							call RF handler()
							-- {
								-> call handleSamplingMicsRequest:
								-- trigger sampling
								-- run TDOA
								-- cal distance
								-- delay random(1->100) * 50us
								-- responseDistanceToNeighbor()
							-- }
							return true; // alway return true to reset task timer
				    -- }

			   if (isFlagAssert == true)
			   	   reset Robot timer delay();

		   } WHILE (isFlagAssert == true);

		   // Now, robot timer delay random is expired
		   IF isSuccessMeasureDistance == false THEN
				if (tryToCalibrateLocalNeighborsForDistanceMeasurement() == true)
				{
					RfTryToCaptureRfSignal(15ms, updateTable1Process);
					   updateTable1Process()
					   -- {
					   	    get Response ID
					   	    get Distance
					   	    add to Table1

					   	    return false; // continue task updateTable1Process()
					   -- }
					isSuccessMeasureDistance = true;
				}
		   END

		   return false; // continue the main TASK
	   }
	*/

	StateOne_MeasureDistance_ResetFlag();

	activeRobotTask(MEASURE_DISTANCE_STATE_MAINTASK_LIFE_TIME_IN_MS, StateOne_MeasureDistance_MainTask);

	//TODO: check if Neighbors Table is NULL

	turnOnLED(LED_ALL);

	setRobotState(ROBOT_STATE_IDLE);	//TODO: switch to next state
}

void StateOne_MeasureDistance_ResetFlag(void)
{
	clearNeighborsTable();
	g_bIsSuccessMeasuredDistances = false;
}

bool StateOne_MeasureDistance_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	turnOnLED(LED_BLUE);

	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask1;
	do
	{
		ui32LifeTimeInUsOfSubTask1 = generateRandomFloatInRange(100000, 1000000); // 100ms to 1000ms

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask1, StateOne_MeasureDistance_SubTask_DelayRandom_Handler);

		if (isRfFlagAssert)		// if subtask 1 is forced to terminal then reset delay
			resetRobotTaskTimer();
	}
	while (isRfFlagAssert);

	// In here, robot timer delay is expired
	if(g_bIsSuccessMeasuredDistances == false)
	{
		resetRobotTaskTimer();

	    if (tryToRequestLocalNeighborsForDistanceMeasurement())
	    {
	    	g_bIsSuccessMeasuredDistances = true;

			turnOffLED(LED_BLUE);
		}
	    // else { Do nothing! There are too noisy in sound! }
	}
	// else { Nothing to do! Already broadcast request successed! }

	return false; // continue the main TASK
}

bool StateOne_MeasureDistance_SubTask_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	uint32_t ui32MessageSize;
	uint8_t* pui8RxBuffer = 0;

	turnOnLED(LED_RED);

	if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
	{
		decodeMessage(pui8RxBuffer, ui32MessageSize);
	}

	if (pui8RxBuffer != 0)
		Network_deleteBuffer(pui8RxBuffer);

	turnOffLED(LED_RED);

	return true; // Terminal This subTask whether or not correct rf message is received
}

void StateOne_MeasureDistance_UpdateNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	turnOnLED(LED_GREEN);

	uint32_t ui32ResponseRobotId = construct4Byte(pui8MessageData);
	uint16_t ui16Distance = (pui8MessageData[4] << 8) | pui8MessageData[5];

	addOverrideToNeighborsTable(ui32ResponseRobotId, ui16Distance);

	turnOffLED(LED_GREEN);
}

//========= Calibration Tab ================================
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
		responseReader.ui8Cmd = ROBOT_RESPONSE_TO_HOST_OK;

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
	if(pui16TestData == 0)
		return;

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

void indicateBatteryVoltage(void)
{
	turnOffLED(LED_ALL);

	triggerSamplingBatteryVoltage(false);
	while(!isNewBattVoltAvailable());

	uint16_t ui16BattVolt = getBatteryVoltage();

	if(ui16BattVolt < BATTERY_LEVEL_LOW)
		turnOnLED(LED_RED);
	else if(ui16BattVolt < BATTERY_LEVEL_FULL)
		turnOnLED(LED_BLUE);
	else
		turnOnLED(LED_GREEN);
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

void transmitADCResultsToHost(uint8_t* pui8Buffer)
{
	turnOnLED(LED_BLUE);
	if (sendDataToHost(pui8Buffer, NUMBER_OF_SAMPLE * 2))
	{
		turnOffLED(LED_BLUE);
	}
}

void transmitRequestDataInEeprom(uint8_t* pui8Data)
{
	uint32_t ui32DataCount = pui8Data[0];

	DEBUG_PRINTS("Host request read %d word(s) in EEPROM\n", ui32DataCount);

	uint8_t ui8ResponseSize = ui32DataCount * 6 + 1;
	uint8_t* pui8ResponseBuffer = malloc(sizeof(uint8_t) * ui8ResponseSize);
	if(pui8ResponseBuffer == 0)
		return;

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

	if (sendMessageToHost(MESSAGE_TYPE_ROBOT_RESPONSE, ROBOT_RESPONSE_TO_HOST_OK, (uint8_t *) pui8ResponseBuffer, ui8ResponseSize))
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

	DEBUG_PRINTS("Host request read bulk %d word(s) in EEPROM\n", pui8Data[0]);

	uint32_t ui32ResponseSize = 1 + 4 + ui32NumberOfBytes;
	uint8_t* pui8ResponseBuffer = malloc(sizeof(uint8_t) * ui32ResponseSize);
	if(pui8ResponseBuffer == 0)
		return;

	// Number of words
	pui8ResponseBuffer[0] = pui8Data[0];

	// Start address
	parse32bitTo4Bytes(&pui8ResponseBuffer[1], EEPROMStartAddress);

	// Read bulk
	EEPROMRead((uint32_t*)(&pui8ResponseBuffer[5]), EEPROMStartAddress, ui32NumberOfBytes);

	turnOnLED(LED_GREEN);

	if (sendMessageToHost(MESSAGE_TYPE_ROBOT_RESPONSE, ROBOT_RESPONSE_TO_HOST_OK, (uint8_t *) pui8ResponseBuffer, ui32ResponseSize))
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

void calibrationTx_TDOA(uint8_t* pui8Data)
{
	// 9-byte = <Rx ID 4b><Test Times 1b><Delay 4b>

	int8_t i;

	uint32_t ui32TargetId = construct4Byte(pui8Data);
	uint8_t ui8TestTimes = pui8Data[4];
	uint32_t ui32DelayMs = construct4Byte(&pui8Data[5]);

	uint16_t ui16PeakA, ui16PeakB;
	uint32_t ui32RespectedRxSize = 8;

	uint32_t ui32DataPointer = 0;
	uint32_t ui32ResponseLength = ui8TestTimes * 4; // Each peak is 2-byte in 8.8 fixed-point format
	uint8_t* pui8ResponseToHostBuffer = malloc(sizeof(uint8_t) * ui32ResponseLength);
	if(pui8ResponseToHostBuffer == 0)
		return;

	i = 0;
	while(i < ui8TestTimes)
	{
		if(tryToCalibrateLocalNeighborsForDistanceMeasurement() == false)
			continue;

		if (RfTryToCaptureRfSignal(100000, isCorrectTDOAResponse, ui32RespectedRxSize, ui32TargetId, &ui16PeakA, &ui16PeakB))
		{
			pui8ResponseToHostBuffer[ui32DataPointer++] = (uint8_t)(ui16PeakA >> 8);
			pui8ResponseToHostBuffer[ui32DataPointer++] = (uint8_t)ui16PeakA;
			pui8ResponseToHostBuffer[ui32DataPointer++] = (uint8_t)(ui16PeakB >> 8);
			pui8ResponseToHostBuffer[ui32DataPointer++] = (uint8_t)ui16PeakB;
		}
		else
		{
			pui8ResponseToHostBuffer[ui32DataPointer++] = 0;
			pui8ResponseToHostBuffer[ui32DataPointer++] = 0;
			pui8ResponseToHostBuffer[ui32DataPointer++] = 0;
			pui8ResponseToHostBuffer[ui32DataPointer++] = 0;
		}

		i++;

		delay_ms(ui32DelayMs);
	}

	if(ui32DataPointer == ui32ResponseLength)
		sendDataToHost(pui8ResponseToHostBuffer, ui32ResponseLength);

	free(pui8ResponseToHostBuffer);
}

bool tryToCalibrateLocalNeighborsForDistanceMeasurement(void)
{
	//NOTE: i16Intercept and i16Slope is Fixed-point <6.10> format
	int16_t i16Intercept = (int16_t)(TDOA_getIntercept() * 1024 + 0.5);	// Fixed-point 6.10
	int16_t i16Slope = (int16_t)(TDOA_getSlope() * 1024 + 0.5);			// Fixed-point 6.10

	triggerSamplingMicSignalsWithPreDelay(0);
	while(!isSamplingCompleted());

	TDOA_filterSignalsMic(getMicrophone0BufferPointer());
	if(TDOA_isFilteredSignalNoisy())
		return false;

	TDOA_filterSignalsMic(getMicrophone1BufferPointer());
	if(TDOA_isFilteredSignalNoisy())
		return false;

	broadcastMeasureDistanceCommandToLocalNeighbors(ROBOT_REQUEST_CALIBRATE_SAMPLING_MICS, i16Intercept, i16Slope);
	triggerSpeakerWithPreDelay(DELAY_BEFORE_START_SPEAKER_US);

	return true;
}

bool isCorrectTDOAResponse(va_list argp)
{
	//  ARGUMENTS:
	//		va_list argp
	//			This list containt one argument in order:
	//				1/ uint32_t ui32RespectedRxSize
	//				2/ uint32_t ui32TargetId
	//				3/ uint16_t* pui16PeakA
	//				4/ uint16_t* pui16PeakB

	// Get the input arguments
	uint32_t ui32RespectedRxSize;
	uint32_t ui32TargetId;
	uint16_t* pui16PeakA;
	uint16_t* pui16PeakB;
	ui32RespectedRxSize = va_arg(argp, uint32_t);
	ui32TargetId = va_arg(argp, uint32_t);
	pui16PeakA = va_arg(argp, uint16_t*);
	pui16PeakB = va_arg(argp, uint16_t*);

	uint32_t ui32ReceivedDataSize = 0;
	uint8_t* pui8DataHolder = 0;
	uint32_t ui32RxId = 0;
	uint8_t* pui8MessageData = 0;

	if (Network_receivedMessage(&pui8DataHolder, &ui32ReceivedDataSize))
	{
		if ((ui32ReceivedDataSize - MESSAGE_HEADER_LENGTH) == ui32RespectedRxSize)
		{
			MessageHeader* pMessageHeader = (MessageHeader*) pui8DataHolder;

			if(pMessageHeader->eMessageType == MESSAGE_TYPE_ROBOT_RESPONSE &&
					pMessageHeader->ui8Cmd == ROBOT_RESPONSE_TDOA_RESULT)
			{
				pui8MessageData = &pui8DataHolder[MESSAGE_DATA_START_IDX];

				ui32RxId = construct4Byte(pui8MessageData);
				if(ui32RxId == ui32TargetId)
				{
					*pui16PeakA = (pui8MessageData[4] << 8) | pui8MessageData[5];
					*pui16PeakB = (pui8MessageData[6] << 8) | pui8MessageData[7];
					return true;
				}
			}
		}
	}

	if (pui8DataHolder != 0)
		Network_deleteBuffer(pui8DataHolder);

	return false;
}

void handleCalibrateSamplingMicsRequest(uint8_t* pui8RequestData)
{
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);
	float fPeakA, fMaxA;
	float fPeakB, fMaxB;

	triggerSamplingMicSignalsWithPreDelay(0);
	while(!isSamplingCompleted());

	TDOA_process(getMicrophone0BufferPointer(), &fPeakA, &fMaxA);
	TDOA_process(getMicrophone1BufferPointer(), &fPeakB, &fMaxB);

	responseTDOAResultsToNeighbor(ui32RequestRobotID, fPeakA, fPeakB);
}

bool responseTDOAResultsToNeighbor(uint32_t ui32NeighborId, float fPeakA, float fPeakB)
{
	uint8_t pui8ResponseData[8];	// <self id 4b><peak A 2b><peak B 2b>
	uint32_t ui32SelfId;
	uint16_t ui16TemplatePeak;

	ui32SelfId = Network_getSelfAddress();
	parse32bitTo4Bytes(pui8ResponseData, ui32SelfId);

	ui16TemplatePeak = (uint16_t)(fPeakA * 256 + 0.5);
	pui8ResponseData[4] = (uint8_t)(ui16TemplatePeak >> 8);
	pui8ResponseData[5] = (uint8_t)ui16TemplatePeak;

	ui16TemplatePeak = (uint16_t)(fPeakB * 256 + 0.5);
	pui8ResponseData[6] = (uint8_t)(ui16TemplatePeak >> 8);
	pui8ResponseData[7] = (uint8_t)ui16TemplatePeak;

	return responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_TDOA_RESULT, pui8ResponseData, 8);
}


/* Test PID Controller only */
float kP = 1;
float kI = 0;
float kD = 0;
float r = 0;
bool bIsRunPID = false;
void testPIDController(uint8_t* pui8Data)
{
	int32_t i32Data;

	i32Data = construct4Byte(pui8Data);
	kP = i32Data / 65536.0f;

	i32Data = construct4Byte(&pui8Data[4]);
	kI = i32Data / 65536.0f;

	i32Data = construct4Byte(&pui8Data[8]);
	kD = i32Data / 65536.0f;

	i32Data = construct4Byte(&pui8Data[12]);
	r = i32Data / 65536.0f;

	if(pui8Data[16] == 1)
		bIsRunPID = true;
	else
		bIsRunPID = false;
}
//-------------------------------

// ======================= Debug Tab ===============================
void sendNeighborsTableToHost(void)
{
	uint32_t ui32Length = NEIGHBOR_TABLE_LENGTH * 6;
	uint8_t* pui8DataBuffer = malloc(sizeof(uint8_t) * ui32Length);
	if(pui8DataBuffer == 0)
		return;

	fillNeighborsTableToByteBuffer(pui8DataBuffer, ui32Length);

	sendDataToHost(pui8DataBuffer, ui32Length);

	free(pui8DataBuffer);
}
