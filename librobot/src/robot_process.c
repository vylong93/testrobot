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
#include "data_manipulation.h"

extern bool Tri_tryToCalculateRobotLocationsTable(void);
extern void GradientDescentMulti_correctLocationsTable(uint32_t ui32OriginalID);

static e_RobotState g_eRobotState = ROBOT_STATE_IDLE;
static e_RobotResponseState g_eRobotResponseState = ROBOT_RESPONSE_STATE_NONE;

static uint8_t* g_pui8RequestData;

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

void handleCommonSubTaskDelayRandomState(void)
{
	uint32_t ui32MessageSize;
	uint8_t* pui8RxBuffer = 0;

	turnOnLED(LED_RED);

	if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
	{
		decodeMessage(pui8RxBuffer, ui32MessageSize);
	}

	Network_deleteBuffer(pui8RxBuffer);

	turnOffLED(LED_RED);
}

void handleNeighborResponseSamplingCollision(void)
{
	switch (getRobotState())
	{
		case ROBOT_STATE_MEASURE_DISTANCE:
			StateOne_MeasureDistance_ResetFlag();
		break;

		default:
		break;
	}
}

//========= State 1 - Measure Distances ================================
static bool g_bIsSuccessMeasuredDistances;

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
				if (tryToRequestLocalNeighborsForDistanceMeasurement() == true)
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

	turnOffLED(LED_ALL);

	do
	{
		StateOne_MeasureDistance_ResetFlag();

		activeRobotTask(MEASURE_DISTANCE_STATE_MAINTASK_LIFE_TIME_IN_MS, StateOne_MeasureDistance_MainTask);
	}
	while(NeighborsTable_getSize() == 0);

	setRobotState(ROBOT_STATE_EXCHANGE_TABLE);
}

void StateOne_MeasureDistance_ResetFlag(void)
{
	NeighborsTable_clear();
	OneHopNeighborsTable_clear();
	RobotLocationsTable_clear();
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
	uint32_t ui32LifeTimeInUsOfSubTask;
	do
	{
		 // 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN, MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateOne_MeasureDistance_SubTask_DelayRandom_Handler);

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

	// Valid commands in this state: ROBOT_REQUEST_SAMPLING_MICS, ROBOT_RESPONSE_DISTANCE_RESULT and ROBOT_RESPONSE_SAMPLING_COLLISION
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle

//	uint32_t ui32MessageSize;
//	uint8_t* pui8RxBuffer = 0;
//
//	turnOnLED(LED_RED);
//
//	if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
//	{
//		// Valid commands in this state: ROBOT_REQUEST_SAMPLING_MICS, ROBOT_RESPONSE_DISTANCE_RESULT and ROBOT_RESPONSE_SAMPLING_COLLISION
//		decodeMessage(pui8RxBuffer, ui32MessageSize);
//	}
//
//	Network_deleteBuffer(pui8RxBuffer);
//
//	turnOffLED(LED_RED);
//
//	return true; // Terminal This subTask whether or not correct rf message is received
}

void StateOne_MeasureDistance_UpdateNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	turnOnLED(LED_GREEN);

	// Get neighbor measure results
	uint32_t ui32ResponseRobotId = construct4Byte(pui8MessageData);
	uint16_t ui16Distance = (pui8MessageData[4] << 8) | pui8MessageData[5];

	// Filtered the invalid results measurement
	if (ui16Distance > 0 && ui16Distance < MAXIMUM_DISTANCE)
		NeighborsTable_addOverride(ui32ResponseRobotId, ui16Distance);

	turnOffLED(LED_GREEN);
}

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

	parse16bitTo2Bytes(&pui8MessageData[4], i16Intercept);

	parse16bitTo2Bytes(&pui8MessageData[6], i16Slope);

	broadcastToLocalNeighbors(ui8Command, pui8MessageData, 8);
}

void handleSamplingMicsRequest(uint8_t* pui8RequestData)
{
	bool bIsSkipTheRest = false;

	uint8_t* pui8RxBuffer = 0;
	uint32_t ui32MessageSize;

	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);
	float fPeakA, fMaxA;
	float fPeakB, fMaxB;

	bool bCurrentInterruptStage;
	MCU_RF_PauseInterruptState(&bCurrentInterruptStage);

	triggerSamplingMicSignalsWithPreDelay(0);
	while(!isSamplingCompleted())
	{
		if (MCU_RF_IsInterruptPinAsserted())
		{
			MCU_RF_ClearIntFlag();

			if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
			{
				if(((MessageHeader*)pui8RxBuffer)->eMessageType == MESSAGE_TYPE_ROBOT_REQUEST &&
						((MessageHeader*)pui8RxBuffer)->ui8Cmd == ROBOT_REQUEST_SAMPLING_MICS)
				{
					reponseCommandToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_SAMPLING_COLLISION);
					bIsSkipTheRest = true;
				}
				// else { Do nothing! Because other command is not collision with the sampling process }
			}
			Network_deleteBuffer(pui8RxBuffer);
		}
	}

	if(!bIsSkipTheRest)
	{
		TDOA_process(getMicrophone0BufferPointer(), &fPeakA, &fMaxA);
		TDOA_process(getMicrophone1BufferPointer(), &fPeakB, &fMaxB);

		if(TDOA_isGoodQualityMeasurement(fMaxA, fMaxB))
		{
			float fInterceptOfRequestRobot = (int16_t)((pui8RequestData[4] << 8) | pui8RequestData[5]) / 1024.0f;
			float fSlopeOfRequestRobot = (int16_t)((pui8RequestData[6] << 8) | pui8RequestData[7]) / 1024.0f;
			uint16_t ui16Distance = TDOA_calculateDistanceFromTwoPeaks(fPeakA, fPeakB, fInterceptOfRequestRobot, fSlopeOfRequestRobot);	// Fixed-point <8.8>

			if (ui16Distance > 0)
			{
				// NOTE: Only add override to neighbors table when at the first state
				if(getRobotState() == ROBOT_STATE_MEASURE_DISTANCE)
				{
					NeighborsTable_addOverride(ui32RequestRobotID, ui16Distance);
				}

				// random 1->100: delay unit (1ms)
				uint32_t ui32RandomUs = (uint32_t)(generateRandomFloatInRange(1, 100) * 1000);
				delay_us(ui32RandomUs + 2000);

				responseDistanceToNeighbor(ui32RequestRobotID, ui16Distance);
			}
			// else { Do nothing! Because the neigbor too far }
		}
		// else { Do nothing! Because of the bad results }
	}

	MCU_RF_ContinueInterruptStateBeforePause(bCurrentInterruptStage);
}

bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	//NOTE: ui16Distance is Fixed-point <8.8> format

	uint8_t pui8ResponseData[6];	// <4-byte self id><2-byte distance>
	uint32_t ui32SelfId;

	ui32SelfId = Network_getSelfAddress();
	parse32bitTo4Bytes(pui8ResponseData, ui32SelfId);

	parse16bitTo2Bytes(&pui8ResponseData[4], ui16Distance);

	return responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_DISTANCE_RESULT, pui8ResponseData, 6);
}


//========= State 2 - Exchange Table ===================================
static bool g_bIsNewLocationsTableAvailable;

void StateTwo_ExchangeTable(void)
{
	/* Pseudo-Code New Attempt

	   clear Locs Table flag: g_bIsNewLocationsTableAvailable := false

	   call Robot timer delay [EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS] with TASK()
		   TASK()
		   --{
			   DO
			   {
				   generate random values
				   clear isFlagAssert flag
				   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
					   handlerInDelayRandom()
					   -- {
								call RF handler()
								-- {
									CASE AskCommand:
										IF i need the neighbors table of the request Robot THEN
											delay random(1->100) * 50us
											send request GiveMe command to which robot ask
											RfTryToCaptureRfSignal(STATE_TWO_SubTask_LIFE_TIME_IN_MS, SubTask)
												SubTask()
												--{
													IF recieved it table THEN
														*g_bIsNewLocationsTableAvailable = false;
														terminal this SubTask
													ELSE
														continue loop at this task until expired
												--}
										END

									CASE GiveMeCommand:
										send my neighbors table to request robot
								-- }
								return true; // alway return true to keep loop at the first state of TASK()
						-- }

				   if (isFlagAssert == true)
					   reset Robot timer delay();

				} WHILE (isFlagAssert == true);

				// Now, robot timer delay random is expired
				IF received enough neighbors' neighbors table AND g_bIsNewLocationsTableAvailable == false THEN
					calculate new Locs Table()
					{
						Tri_addLocation(g_ui32RobotID, 0, 0);
						Tri_findLocs(NeighborsTable, OneHopNeighborsTable);
					}
					*g_bIsNewLocationsTableAvailable = true;
					reset Robot timer delay();
					// NOTE: shouldn't break the TASK, wait for task expired to keep synchronous state
				END

				broadcast ASK command
		   --}

		IF Locs table cannot update THEN
			go back to State ONE
		ELSE
			go to next State
	 */

	/* Pseudo-Code Old Attempt
	   reset targetNeighbor
	   clear Locs Table flag: isNeedUpdateLocsTable := false
	   DO
   	   {
	   	   call Robot timer delay [STATE_TWO_LIFE_TIME_IN_MS] with TASK(*targetNeighbor, *isNeedUpdateLocsTable)
	   	   	   TASK()
			   --{
			   	   DO
			   	   {
					   generate random values
					   clear isFlagAssert flag
					   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
						   handlerInDelayRandom()
						   -- {
									call RF handler()
									-- {
										-> call handleTransmitNeighborsTableRequest:
										-- delay random(1->100) * 50us
										-- response self neighbors table to request robot
									-- }
									return true; // alway return true to reset task timer
							-- }

						if (isFlagAssert == true)
							reset Robot timer delay();

					} WHILE (isFlagAssert == true);

		   	   	   	// Now, robot timer delay random is expired
		   	   	   	IF received all require Table THEN		// IDEAS: cal the locs table if received enough neighbors' neighbors table >= 2, and recall locs table if receive more
		   	   	   		IF isNeedUpdateLocsTable == true THEN
		   	   	   			reset Robot timer delay();
		   	   	   			calculate Locs Table()
		   	   	   			{
								Tri_addLocation(g_ui32RobotID, 0, 0);
								Tri_findLocs(NeighborsTable, OneHopNeighborsTable);
							}
		   	   	   			isNeedUpdateLocsTable = false;
		   	   	   		END
		   	   	   		// NOTE: shouldn't break the TASK, wait for task expired to keep synchronous state
		   	   	   	ELSE
		   	   	   		reset Robot timer delay();
		   	   	   		IF send Neighbors Table Request to targetNeighbor is success THEN
		   	   	   			set targetNeighbor to next neighbor
		   	   	   			isNeedUpdateLocsTable = true;
						END
						// NOTE: shouldn't calculate the Locs Table in here due to asynchronous
					END
			   --}

		} WHILE Locs Table isn't updated

		turnOffLED(LED_BLUE);
	 */

	turnOffLED(LED_ALL);

	StateTwo_ExchangeTable_ResetFlag();

	//
	// Synchornous delay for previous state
	//
	delay_us(MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	do
	{
		activeRobotTask(EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS, StateTwo_ExchangeTable_MainTask);
	}
	while(!g_bIsNewLocationsTableAvailable);

	setRobotState(ROBOT_STATE_IDLE);	// TODO: switch to next state: VOTE_ORIGIN
}

void StateTwo_ExchangeTable_ResetFlag(void)
{
	OneHopNeighborsTable_clear();
	g_bIsNewLocationsTableAvailable = false;
}

bool StateTwo_ExchangeTable_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	turnOnLED(LED_GREEN);

	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask;

	do
	{
		// 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN, EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateTwo_ExchangeTable_SubTask_DelayRandom_Handler);

		if (isRfFlagAssert)
			resetRobotTaskTimer();
	}
	while(isRfFlagAssert);

	// Now, robot timer delay random is expired
	if(!g_bIsNewLocationsTableAvailable)
	{
		if((OneHopNeighborsTable_getSize() == NeighborsTable_getSize()))
		{
			RobotLocationsTable_add(Network_getSelfAddress(), 0, 0);

			if(Tri_tryToCalculateRobotLocationsTable())
			{
				g_bIsNewLocationsTableAvailable = true;
			}
			else
			{
				RobotLocationsTable_clear();
			}
			// NOTE: shouldn't break the TASK, wait for task expired to keep synchronous state
		}
		else
		{
			uint32_t ui32TargetId;
			int i = NeighborsTable_getSize() - OneHopNeighborsTable_getSize();
			for(; i > 0 ; i--)
			{
				ui32TargetId = NeighborsTable_getIdAtIndex(i - 1);
				if(!OneHopNeighborsTable_isContainRobot(ui32TargetId))
				{
					sendRequestNeighborsTableCommandToNeighbor(ui32TargetId);
					break;
				}
			}
			// NOTE: shouldn't calculate the Locs Table in here due to asynchronous
		}
	}

	return false; // continue the main TASK
}

bool StateTwo_ExchangeTable_SubTask_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// Valid commands in this state: ROBOT_REQUEST_NEIGHBORS_TABLE and ROBOT_RESPONSE_NEIGHBORS_TABLE
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle

//	uint32_t ui32MessageSize;
//	uint8_t* pui8RxBuffer = 0;
//
//	turnOnLED(LED_RED);
//
//	if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
//	{
//		// Valid commands in this state: ROBOT_REQUEST_NEIGHBORS_TABLE and ROBOT_RESPONSE_NEIGHBORS_TABLE
//		decodeMessage(pui8RxBuffer, ui32MessageSize);
//	}
//
//	Network_deleteBuffer(pui8RxBuffer);
//
//	turnOffLED(LED_RED);
//
//	return true; // alway return true to keep loop at the first state of TASK()
}

void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData)
{
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);
	uint32_t ui32SelfID = Network_getSelfAddress();

	uint32_t ui32TotalLength = SIZE_OF_ROBOT_ID + NeighborsTable_getSize() * SIZE_OF_ROBOT_MEAS;
	uint8_t* pui8DataBuffer = malloc(sizeof(uint8_t) * ui32TotalLength);
	if(pui8DataBuffer == 0)
		return;

	parse32bitTo4Bytes(pui8DataBuffer, ui32SelfID);

	NeighborsTable_fillContentToByteBuffer(&pui8DataBuffer[SIZE_OF_ROBOT_ID], ui32TotalLength - SIZE_OF_ROBOT_ID);

	responseMessageToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_NEIGHBORS_TABLE, pui8DataBuffer, ui32TotalLength);

	free(pui8DataBuffer);
}

void StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	turnOnLED(LED_BLUE);

	uint32_t ui32ResponseRobotId = construct4Byte(pui8MessageData);

	if(!OneHopNeighborsTable_isContainRobot(ui32ResponseRobotId))
		OneHopNeighborsTable_add(ui32ResponseRobotId, &pui8MessageData[SIZE_OF_ROBOT_ID], ui32DataSize - SIZE_OF_ROBOT_ID);

	turnOffLED(LED_BLUE);
}

void sendRequestNeighborsTableCommandToNeighbor(uint32_t ui32NeighborId)
{
	uint32_t ui32SelfId = Network_getSelfAddress();
	uint8_t pui8MessageData[4];	// <4-byte SelfId>

	parse32bitTo4Bytes(pui8MessageData, ui32SelfId);

	sendMessageToNeighbor(ui32NeighborId, ROBOT_REQUEST_NEIGHBORS_TABLE, pui8MessageData, 4);
}


//========= Calibration Tab ============================================
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
	parse16bitTo2Bytes(&pui8ResponseData[4], ui16TemplatePeak);

	ui16TemplatePeak = (uint16_t)(fPeakB * 256 + 0.5);
	parse16bitTo2Bytes(&pui8ResponseData[6], ui16TemplatePeak);

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
	uint8_t pui8DataBuffer[60] = {0};

	NeighborsTable_fillContentToByteBuffer(pui8DataBuffer, 60);

	sendDataToHost(pui8DataBuffer, 60);
}

void sendOneHopNeighborsTableToHost(void)
{
	uint8_t pui8DataBuffer[640] = {0};

	OneHopNeighborsTable_fillContentToByteBuffer(pui8DataBuffer, 640);

	sendDataToHost(pui8DataBuffer, 640);
}

void sendRobotLocationsTableToHost(void)
{
//	uint8_t* pui8DataBuffer = malloc(sizeof(uint8_t) * 120);
	uint8_t pui8DataBuffer[120] = {0};

	turnOnLED(LED_ALL);

	RobotLocationsTable_fillContentToByteBuffer(pui8DataBuffer, 120);

	sendDataToHost(pui8DataBuffer, 120);

	turnOffLED(LED_ALL);

//	free(pui8DataBuffer);
}

void selfCorrectLocationsTable(void)
{
	GradientDescentMulti_correctLocationsTable(Network_getSelfAddress());
}
