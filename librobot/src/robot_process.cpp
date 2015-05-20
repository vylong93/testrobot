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
#include "librobot/inc/robot_task_timer.h"
#include "librobot/inc/robot_imu.h"

#include "libcustom/inc/custom_uart_debug.h"
#include "libcustom/inc/custom_led.h"
#include "libcustom/inc/custom_delay.h"

#include "libprotocol/inc/network.h"

#include "libstorage/inc/robot_data.h"
#include "libstorage/inc/RobotIdentity.h"

#include "libalgorithm/inc/TDOA.h"
#include "libalgorithm/inc/Trilateration.h"
#include "libalgorithm/inc/GradientDescentGlobal.h"
#include "libalgorithm/inc/GradientMap.h"

#include "libcontroller/inc/Controller.h"
#include "interrupt_definition.h"
#include "data_manipulation.h"

#ifdef REGION_ROBOT_VARIABLES_AND_FUNCTIONS

RobotIdentity_t g_RobotIdentity;

static e_RobotState g_eRobotState = ROBOT_STATE_IDLE;
static e_RobotResponseState g_eRobotResponseState = ROBOT_RESPONSE_STATE_NONE;

static uint8_t* g_pui8RequestData;

static GradientMap* g_pGradientMap;

static uint32_t g_ui32RandomW;
static uint32_t g_ui32RandomWIndex;

void test(void) // Test Only
{
//	Robot:0xBEAD08 (31.3617; -0.052002)
//	Self neighbors = 6
//	Origin:0xBEAD01, neighbors = 6, Hopth = 1
//	Rotation Hop:0xBEAD01 (0; 0)
	g_RobotIdentity.Self_ID = 0xBEAD08;
	g_RobotIdentity.Self_NeighborsCount = 6;

	g_RobotIdentity.x = 0.0f;
	g_RobotIdentity.y = 0.0f;
	g_RobotIdentity.theta = 0.0f;

	g_RobotIdentity.Origin_ID = 0xBEAD01;
	g_RobotIdentity.Origin_NeighborsCount = 6;
	g_RobotIdentity.Origin_Hopth = 1;

	g_RobotIdentity.RotationHop_ID = 0xBEAD01;
	g_RobotIdentity.RotationHop_x = 0;
	g_RobotIdentity.RotationHop_y = 0;

	uint8_t* pui8MessageData = new uint8_t[72];
	initData(pui8MessageData);

	Tri_tryToRotateLocationsTable(&g_RobotIdentity, pui8MessageData, 72 / 12);

	delete[] pui8MessageData;
}

int readChipRev(void)
{
//	#define SCRD  	0x400FE000
//	#define	DID0	0x00000000
//	#define	DID1	0x00000004
//
//	int32_t rDID0 = HWREG(SCRD + DID0);
//	int DID0_VER = (rDID0 >> 28) & 0x07;
//	int DID0_CLASSS = (rDID0 >> 16) & 0x0F;
//	int DID0_MAJOR = (rDID0 >> 8) & 0x0F;
//	int DID0_MINOR = rDID0 & 0x0F;
//
//	int32_t rDID1 = HWREG(SCRD + DID1);
//	int DID1_VER = (rDID1 >> 28) & 0x0F;
//	int DID1_FAM = (rDID1 >> 24) & 0x0F;
//	int DID1_PARTNO = (rDID1 >> 16) & 0xFF;
//	int DID1_PINCOUNT = (rDID1 >> 13) & 0x07;
//	int DID1_TEMP = (rDID1 >> 5) & 0x07;
//	int DID1_PKG = (rDID1 >> 3) & 0x03;
//	int DID1_ROHS = (rDID1 >> 2) & 0x01;
//	int DID1_QUAL = rDID1 & 0x03;
//
//	return (DID0_MAJOR + DID0_MINOR);

	return 0;
}

void initRobotProcess(void)
{
	uint32_t ui32ReadEEPROMData;
	float fTDOA_Intercept;
	float fTDOA_Slope;
	uint8_t ui8LeftMotorOffset;
	uint8_t ui8RightMotorOffset;
	uint16_t ui16PeriodMotorOffset;

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

	resetRobotIdentity();

	//
	// Initialize motor parameters
	//
	if(getMotorParametersInEEPROM(&ui8LeftMotorOffset, &ui8RightMotorOffset, &ui16PeriodMotorOffset))
	{
		setLeftMotorOffset(ui8LeftMotorOffset);
		setRightMotorOffset(ui8RightMotorOffset);
		setPeriodMotorOffset(ui16PeriodMotorOffset);
	}

	//
	// Initialize TDOA
	//
	if(getTDOAParameterInEEPROM(&fTDOA_Intercept, &fTDOA_Slope))
	{
		TDOA_setIntercept(fTDOA_Intercept);
		TDOA_setSlope(fTDOA_Slope);
	}
	TDOA_initFilters();

	//
	// Initialize Data Storages
	//
	initLinkedList();

	//
	// Initialize Gradient Map
	//
	g_pGradientMap = new GradientMap();
	DEBUG_PRINTS3("init DASH::Gradient Map [%d][%d] at 0x%08x\n", g_pGradientMap->Height, g_pGradientMap->Width, g_pGradientMap->pGradientMap);

	//
	// Initialize Random Word
	//
	if(!getRandomWordInEEPROM(&g_ui32RandomW))
	{
		//TODO: generate 8 random half-byte and store to g_ui32RandomW
	}
	g_ui32RandomWIndex = 0;
	
	//==============================================
	// IMPORTANCE: Configure Software Interrupt
	//==============================================
	ROM_IntPrioritySet(INT_SW_TRIGGER_ROBOT_RESPONSE, PRIORITY_ROBOT_RESPONSE);

	ROM_IntEnable(INT_SW_TRIGGER_ROBOT_RESPONSE);
}

void resetRobotIdentity(void)
{
	//
	// Initialize RobotIdentity
	//
	g_RobotIdentity.Self_ID = Network_getSelfAddress();
	g_RobotIdentity.Self_NeighborsCount = 0;
	g_RobotIdentity.Origin_ID = 0;
	g_RobotIdentity.Origin_NeighborsCount = 0;
	g_RobotIdentity.Origin_Hopth = 0;
	g_RobotIdentity.RotationHop_ID = 0;
	g_RobotIdentity.x = 0;
	g_RobotIdentity.y = 0;
	g_RobotIdentity.theta = MATH_PI_DIV_2;
	g_RobotIdentity.RotationHop_x = 0;
	g_RobotIdentity.RotationHop_y = 0;
}

void setRobotState(e_RobotState eState)
{
	Motors_stop();
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
//		g_pui8RequestData = malloc(sizeof(*g_pui8RequestData) * ui32DataSize);
		g_pui8RequestData = new uint8_t[ui32DataSize];
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

#endif

#ifdef REGION_STATE_ONE_MEASURE_DISTANCE

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

	turnOnLED(LED_BLUE);

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
	resetRobotIdentity();
	g_bIsSuccessMeasuredDistances = false;
}

bool StateOne_MeasureDistance_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

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
}

void StateOne_MeasureDistance_SamplingMicsHandler(uint8_t* pui8RequestData)
{
	bool bIsSkipTheRest = false;

	uint8_t* pui8RxBuffer = 0;
	uint32_t ui32MessageSize;

	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);
	float fPeakA, fMaxA;
	float fPeakB, fMaxB;

//	bool bCurrentInterruptStage;
//	MCU_RF_PauseInterruptState(&bCurrentInterruptStage);

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

//	MCU_RF_ContinueInterruptStateBeforePause(bCurrentInterruptStage);
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

	uint8_t pui8MessageData[8];	// <4-byte SelfId><2-byte Intercept><2-byte Slope>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	parse16bitTo2Bytes(&pui8MessageData[4], i16Intercept);

	parse16bitTo2Bytes(&pui8MessageData[6], i16Slope);

	broadcastToLocalNeighbors(ui8Command, pui8MessageData, 8);
}

bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	//NOTE: ui16Distance is Fixed-point <8.8> format

	uint8_t pui8ResponseData[6];	// <4-byte self id><2-byte distance>

	parse32bitTo4Bytes(pui8ResponseData, g_RobotIdentity.Self_ID);

	parse16bitTo2Bytes(&pui8ResponseData[4], ui16Distance);

	return responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_DISTANCE_RESULT, pui8ResponseData, 6);
}

#endif

#ifdef REGION_STATE_TWO_EXCHANGE_TABLE

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

	turnOnLED(LED_GREEN);

	do
	{
		activeRobotTask(EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS, StateTwo_ExchangeTable_MainTask);
	}
	while(!g_bIsNewLocationsTableAvailable);

	setRobotState(ROBOT_STATE_VOTE_ORIGIN);
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
			RobotLocationsTable_add(g_RobotIdentity.Self_ID, 0, 0);

			if(Tri_tryToCalculateRobotLocationsTable(g_RobotIdentity.Self_ID))
			{
				g_bIsNewLocationsTableAvailable = true;
				g_RobotIdentity.Self_NeighborsCount = RobotLocationsTable_getSize();

				turnOffLED(LED_GREEN);
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
}

void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData)
{
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);

	uint32_t ui32TotalLength = SIZE_OF_ROBOT_ID + NeighborsTable_getSize() * SIZE_OF_ROBOT_MEAS;
//	uint8_t* pui8DataBuffer = malloc(sizeof(*pui8DataBuffer) * ui32TotalLength);
	uint8_t* pui8DataBuffer = new uint8_t[ui32TotalLength];
	if(pui8DataBuffer == 0)
		return;

	parse32bitTo4Bytes(pui8DataBuffer, g_RobotIdentity.Self_ID);

	NeighborsTable_fillContentToByteBuffer(&pui8DataBuffer[SIZE_OF_ROBOT_ID], ui32TotalLength - SIZE_OF_ROBOT_ID);

	responseMessageToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_NEIGHBORS_TABLE, pui8DataBuffer, ui32TotalLength);

//	free(pui8DataBuffer);
	delete[] pui8DataBuffer;
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
	uint8_t pui8MessageData[4];	// <4-byte SelfId>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	sendMessageToNeighbor(ui32NeighborId, ROBOT_REQUEST_NEIGHBORS_TABLE, pui8MessageData, 4);
}

#endif

#ifdef REGION_STATE_THREE_VOTE_ORIGIN

static uint8_t g_ui8BroadcastVoteCommandCounter;

void StateThree_VoteTheOrigin(void)
{
	/* Pseudo code
	   reset myselt as the origin()
	   {
		   set the number of neighbors of the origin is my locations table count
		   set the hopt count to zero
		   set the rotation hop ID to my ID
		   set the origin id to my ID
	   }
	   reset flag's state 3: broadcastCounter = 0
	   call Robot timer delay [STATE_THREELIFE_TIME_IN_MS] with TASK()
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
									-> call handle Update Network Origin COMMAND:
									-- updateOrRejectNetworkOrigin()
									   {
									   	   IF rxOriginNodeID == my origin ID THEN
									   	   	   return
									   	   END

										   IF my Number of Neighbors Of Origin <= rxNumberOfNeighborsOfOrigin
											   my Hopth = rxHopth + 1
											   my Origin ID = rxOriginNodeID
											   my Number of Neighbors Of Origin = rxNumberOfNeighborsOfOrigin
										   END

									   	   broadcastCounter = 0;
									   }
								-- }
								return true; // alway return true to reset task timer
						-- }

					if (isFlagAssert == true)
						reset Robot timer delay();

				} WHILE (isFlagAssert == true);

				// Now, robot timer delay random is expired

				IF broadcastCounter < BROADCAST_VOTE_TIMES THEN
					broadcast <Update Network Origin COMMAND><thisOriginID><thisOriginNumberOfNeighbors><thisHopthNumber>
					broadcastCounter++
				END

				return false; // continue the main TASK
		   --}
	   goto next state: ROTATE_NETWORK
	 */

	turnOffLED(LED_ALL);

	StateThree_VoteTheOrigin_ResetFlag();

	//
	// Synchornous delay for previous state
	//
	delay_us(EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	//
	// indicates the origin id
	//
	indicatesOriginIdToLEDs(g_RobotIdentity.Origin_ID);

	//
	// Synchornous delay for previous state
	//
	delay_us(EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	activeRobotTask(VOTE_THE_OGIRIN_STATE_MAINTASK_LIFE_TIME_IN_MS, StateThree_VoteTheOrigin_MainTask);

	setRobotState(ROBOT_STATE_ROTATE_COORDINATES);
}

void StateThree_VoteTheOrigin_ResetFlag(void)
{
	g_RobotIdentity.Origin_ID = g_RobotIdentity.Self_ID;
	g_RobotIdentity.Origin_NeighborsCount = g_RobotIdentity.Self_NeighborsCount;
	g_RobotIdentity.Origin_Hopth = 0;
	g_ui8BroadcastVoteCommandCounter = 0;
}

bool StateThree_VoteTheOrigin_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask;
	do
	{
		 // 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MIN, VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateThree_VoteTheOrigin_SubTask_DelayRandom_Handler);

		if (isRfFlagAssert)
			resetRobotTaskTimer();
	}
	while (isRfFlagAssert);

	// In here, robot timer delay is expired

	if (g_ui8BroadcastVoteCommandCounter < BROADCAST_VOTE_TIMES)
	{
		broadcastVoteTheOriginCommandToLocalNeighbors();
		g_ui8BroadcastVoteCommandCounter++;
	}

	return false; // continue the main TASK
}

bool StateThree_VoteTheOrigin_SubTask_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// Valid commands in this state: ROBOT_REQUEST_VOTE_THE_ORIGIN
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle
}

void StateThree_VoteTheOrigin_VoteTheOriginHandler(uint8_t* pui8RequestData)
{
	uint32_t ui32RxOriginNodeID = construct4Byte(pui8RequestData);
	uint8_t ui8RxOriginNeighborsCount = pui8RequestData[4];
	uint8_t ui8RxOriginHopth = pui8RequestData[5];

	bool bIsNeedToUpdateRobotIdentity ;

	if (ui32RxOriginNodeID == g_RobotIdentity.Origin_ID)
		return;

	if (ui8RxOriginNeighborsCount > g_RobotIdentity.Origin_NeighborsCount)
	{
		bIsNeedToUpdateRobotIdentity = true;
	}
	else if (ui8RxOriginNeighborsCount == g_RobotIdentity.Origin_NeighborsCount &&
			ui32RxOriginNodeID < g_RobotIdentity.Origin_ID)
	{
		bIsNeedToUpdateRobotIdentity = true;
	}
	else
	{
		bIsNeedToUpdateRobotIdentity = false;
	}

	if(bIsNeedToUpdateRobotIdentity)
	{
		g_RobotIdentity.Origin_ID = ui32RxOriginNodeID;
		g_RobotIdentity.Origin_NeighborsCount = ui8RxOriginNeighborsCount;
		g_RobotIdentity.Origin_Hopth = ui8RxOriginHopth + 1;

		//
		// indicates the origin id
		//
		indicatesOriginIdToLEDs(g_RobotIdentity.Origin_ID);
	}

	g_ui8BroadcastVoteCommandCounter = 0;
}

void broadcastVoteTheOriginCommandToLocalNeighbors(void)
{
	uint8_t pui8MessageData[6]; // <4-byte origin><1-byte origin's neighbors count><1-byte hopth>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Origin_ID);
	pui8MessageData[4] = g_RobotIdentity.Origin_NeighborsCount;
	pui8MessageData[5] = g_RobotIdentity.Origin_Hopth;

	broadcastToLocalNeighbors(ROBOT_REQUEST_VOTE_THE_ORIGIN, pui8MessageData, 6);
}

void indicatesOriginIdToLEDs(uint32_t ui32Id)
{
	if (ui32Id & 0x01)
		turnOnLED(LED_GREEN);
	else
		turnOffLED(LED_GREEN);

	if (ui32Id & 0x02)
		turnOnLED(LED_BLUE);
	else
		turnOffLED(LED_BLUE);

	if (ui32Id & 0x04)
		turnOnLED(LED_RED);
	else
		turnOffLED(LED_RED);
}

#endif

#ifdef REGION_STATE_FOUR_ROTATE_NETWORK
RobotRotationFlag_t* g_pCoordinatesRotationFlagTable;
//RobotRotationFlag_t g_pCoordinatesRotationFlagTable[NEIGHBORS_TABLE_LENGTH];
int32_t g_i32FlagTableLength;
int32_t g_i32TargetFlagPointer;
bool g_bIsCoordinatesRotated;
uint8_t* g_pui8LocationsTableBuffer;
uint32_t g_ui32LocationsTableBufferLength;

void StateFour_RotateCoordinates(void)
{
	/* Pseudo code
	   allocate rotation flag table()
	   {
	   	   clear global target pointer
	   	   clear all rotation flag to FALSE
		   IF I am the origin THEN
		   	   set my rotation hop id is my id
		   	   set my location is (0; 0)
		   	   set my rotation flag to TRUE
	   	   END
	   }

	   call Robot timer delay [STATE_FOUR_LIFE_TIME_IN_MS] with TASK()
		   TASK()
		   --{
		   	   try to find a neighbor which rotation flag is FALSE

			   DO
			   {
				   generate random values
				   clear isFlagAssert flag
				   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
					   handlerInDelayRandom() ,  and
					   -- {
								call RF handler()
								-- {
									-> call handle COMMAND:
									case ROBOT_REQUEST_ROTATE_COORDINATES:
									-- IF my coordinates rotated THEN
									 	  transmit ROBOT_RESPONSE_COORDINATES_ROTATED
									   ELSE
									   	  transmit ROBOT_REQUEST_READ_LOCATIONS_TABLE
									   END

									case ROBOT_RESPONSE_COORDINATES_ROTATED:
									-- set corresponding rotation flag to TRUE

									case ROBOT_REQUEST_LOCATIONS_TABLE:
									-- transmit my locations table to request neighbor

									case ROBOT_RESPONSE_LOCATIONS_TABLE:
									-- try to rotate locations table
									   IF success THEN
									   	   set my rotation flag to TRUE
									   END
								-- }
								return true; // alway return true to reset task timer
						-- }

					if (isFlagAssert == true)
						reset Robot timer delay();

				} WHILE (isFlagAssert == true);

				// Now, robot timer delay random is expired
	   			IF I have rotated AND target neighbor is not rotated THEN
					request target neighbors to rotate
					increase target pointer
					reset Robot timer delay
	   			END

				return false; // continue the main TASK
		   --}

	   IF I didn't rotate my locations table THEN
	   	   goto previous state: VOTE_THE_ORIGIN
	   ELSE
		   transform locs table to world frame
	       goto next state
	   END
	 */

	DEBUG_PRINTS("Self ID = 0x%06x\n", g_RobotIdentity.Self_ID);
	DEBUG_PRINTS("Origin ID = 0x%06x\n", g_RobotIdentity.Origin_ID);

	turnOffLED(LED_ALL);

	if(StateFour_RotateCoordinates_ResetFlag() == false)
	{
		setRobotState(ROBOT_STATE_IDLE);
		return;
	}

	//
	// Synchornous delay for previous state
	//
	delay_us(VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	activeRobotTask(ROTATE_COORDINATES_STATE_MAINTASK_LIFE_TIME_IN_MS, StateFour_RotateCoordinates_MainTask);

	if(g_pCoordinatesRotationFlagTable != 0)
		delete[] g_pCoordinatesRotationFlagTable;

	if(g_pui8LocationsTableBuffer != 0)
		delete[] g_pui8LocationsTableBuffer;

	if(g_bIsCoordinatesRotated)
		setRobotState(ROBOT_STATE_AVERAGE_VECTOR);
	else
		setRobotState(ROBOT_STATE_VOTE_ORIGIN);
}

bool StateFour_RotateCoordinates_ResetFlag(void)
{
	DEBUG_PRINT("Entering StateFour_RotateCoordinates_ResetFlag........\n");

	g_pui8LocationsTableBuffer = 0;

	g_i32TargetFlagPointer = 0;

	g_i32FlagTableLength = RobotLocationsTable_getSize() - 1;
	g_pCoordinatesRotationFlagTable = new RobotRotationFlag_t[g_i32FlagTableLength];
	if (g_pCoordinatesRotationFlagTable == 0)
	{
		DEBUG_PRINT("........Returning FALSE from StateFour_RotateCoordinates_ResetFlag\n");
		return false;
	}

	uint32_t ui32TargetId;
	int pointer = 0;
	int i;
	for(i = 0; i < g_i32FlagTableLength; i++)
	{
		ui32TargetId = RobotLocationsTable_getIdAtIndex(pointer++);
		if(ui32TargetId == g_RobotIdentity.Self_ID)
		{
			i--;
			continue;
		}

		g_pCoordinatesRotationFlagTable[i].ID = ui32TargetId;
		g_pCoordinatesRotationFlagTable[i].isRotated = false;
		DEBUG_PRINTS2("add item to Rotation Flag Table: 0x%06x : %d\n", g_pCoordinatesRotationFlagTable[i].ID, g_pCoordinatesRotationFlagTable[i].isRotated);
	}

	if(g_RobotIdentity.Origin_ID == g_RobotIdentity.Self_ID)
	{
		g_RobotIdentity.RotationHop_ID = g_RobotIdentity.Self_ID;
		g_RobotIdentity.x = 0;
		g_RobotIdentity.y = 0;
		g_bIsCoordinatesRotated = true;

		prepareLocationsTableBuffer();

		turnOnLED(LED_ALL);

		DEBUG_PRINTS("I am the origin: 0x%06x\n", g_RobotIdentity.RotationHop_ID);
	}
	else
	{
		g_bIsCoordinatesRotated = false;
	}

	DEBUG_PRINT("........Returning TRUE from StateFour_RotateCoordinates_ResetFlag\n");
	return true;
}

bool StateFour_RotateCoordinates_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	DEBUG_PRINT("Entering StateFour_RotateCoordinates_MainTask........\n");

	uint32_t ui32TargetId;
	do
	{
		ui32TargetId = g_pCoordinatesRotationFlagTable[g_i32TargetFlagPointer].ID;

		g_i32TargetFlagPointer++;
		if(g_i32TargetFlagPointer >= g_i32FlagTableLength)
		{
			g_i32TargetFlagPointer = 0;
			break;
		}

		if(g_pCoordinatesRotationFlagTable[g_i32TargetFlagPointer].isRotated)
			continue;
		else
			break;

	} while(true);

	DEBUG_PRINTS3("Select target neighbor: pointer = %d, ID = 0x%06x, flag = %d\n", g_i32TargetFlagPointer, ui32TargetId, g_pCoordinatesRotationFlagTable[g_i32TargetFlagPointer].isRotated);

	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask;
	do
	{
		 // 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MIN, ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateFour_RotateCoordinates_SubTask_DelayRandom_Handler);

		if (isRfFlagAssert)
			resetRobotTaskTimer();
	}
	while (isRfFlagAssert);

	// In here, robot timer delay is expired
	if (g_bIsCoordinatesRotated == true && getRotationFlagOfRobot(ui32TargetId) == false)
	{
		if(sendRequestRotateCoordinatesCommandToNeighbor(ui32TargetId))
			setRotationFlagOfRobotTo(ui32TargetId, true);

		resetRobotTaskTimer();
	}

	DEBUG_PRINT("........Returning from StateFour_RotateCoordinates_MainTask\n");

	return false; // continue the main TASK
}

bool StateFour_RotateCoordinates_SubTask_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// Valid commands in this state: ROBOT_REQUEST_ROTATE_COORDINATES, ROBOT_REQUEST_READ_LOCATIONS_TABLE, ROBOT_RESPONSE_COORDINATES_ROTATED and ROBOT_RESPONSE_LOCATIONS_TABLE
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle
}

void StateFour_RotateCoordinates_RotateCoordinatesHandler(uint8_t* pui8RequestData)
{
	// <4-byte SelfId>
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);

	DEBUG_PRINTS("received ROBOT_REQUEST_ROTATE_COORDINATES from 0x%06x\n", ui32RequestRobotID);

	uint8_t pui8MessageData[4];	// <4-byte SelfId>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	if(g_bIsCoordinatesRotated)
	{
		DEBUG_PRINTS("send ROBOT_RESPONSE_COORDINATES_ROTATED to 0x%06x\n", ui32RequestRobotID);
		responseMessageToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_COORDINATES_ROTATED, pui8MessageData, 4);
	}
	else
	{
		DEBUG_PRINTS("send ROBOT_REQUEST_READ_LOCATIONS_TABLE to 0x%06x\n", ui32RequestRobotID);
		sendMessageToNeighbor(ui32RequestRobotID, ROBOT_REQUEST_READ_LOCATIONS_TABLE, pui8MessageData, 4);
	}
}

void StateFour_RotateCoordinates_UpdateRotationFlagTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	uint32_t ui32ResponseRobotId = construct4Byte(pui8MessageData);

	DEBUG_PRINTS("received ROBOT_RESPONSE_COORDINATES_ROTATED from 0x%06x\n", ui32ResponseRobotId);

	setRotationFlagOfRobotTo(ui32ResponseRobotId, true);
}

void StateFour_RotateCoordinates_ReadLocationsTableHandler(uint8_t* pui8RequestData)
{
	uint32_t ui32RequestRobotID = construct4Byte(pui8RequestData);

//	uint32_t ui32TotalLength = RobotLocationsTable_getSize() * SIZE_OF_ROBOT_LOCATION + 12;
//	uint8_t* pui8DataBuffer = malloc(sizeof(*pui8DataBuffer) * ui32TotalLength);
//	if(pui8DataBuffer == 0)
//		return;
//
//	parse32bitTo4Bytes(pui8DataBuffer, g_RobotIdentity.Self_ID);
//
//	int32_t i32Template;
//	i32Template = (int32_t)(g_RobotIdentity.x * 65536 + 0.5);
//	parse32bitTo4Bytes(&pui8DataBuffer[4], i32Template);
//
//	i32Template = (int32_t)(g_RobotIdentity.y * 65536 + 0.5);
//	parse32bitTo4Bytes(&pui8DataBuffer[8], i32Template);
//
//	RobotLocationsTable_fillContentToByteBufferOffsetLocal(g_RobotIdentity.Self_ID, &pui8DataBuffer[12], ui32TotalLength - 12);
//
//	responseMessageToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_LOCATIONS_TABLE, pui8DataBuffer, ui32TotalLength);

	responseMessageToNeighbor(ui32RequestRobotID, ROBOT_RESPONSE_LOCATIONS_TABLE, g_pui8LocationsTableBuffer, g_ui32LocationsTableBufferLength);

	DEBUG_PRINTS("transmit locations table to 0x%06x\n", ui32RequestRobotID);

//	free(pui8DataBuffer);
}

void StateFour_RotateCoordinates_ReceivedLocationsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	uint32_t ui32RequestRobotID = construct4Byte(pui8MessageData);

	int32_t i32TemplateXY;
	float fRequestRobot_x;
	float fRequestRobot_y;

	i32TemplateXY = construct4Byte(&pui8MessageData[4]);
	fRequestRobot_x = (float)(i32TemplateXY / 65536.0f);

	i32TemplateXY = construct4Byte(&pui8MessageData[8]);
	fRequestRobot_y = (float)(i32TemplateXY / 65536.0f);

	g_RobotIdentity.RotationHop_ID = ui32RequestRobotID;
	g_RobotIdentity.RotationHop_x = fRequestRobot_x;
	g_RobotIdentity.RotationHop_y = fRequestRobot_y;

	if(Tri_tryToRotateLocationsTable(&g_RobotIdentity, &pui8MessageData[12], (int32_t)((ui32DataSize - 12) / SIZE_OF_ROBOT_LOCATION)))
		RobotLocationsTable_transformToWorldFrame(&g_RobotIdentity);

	g_bIsCoordinatesRotated = true;

	prepareLocationsTableBuffer();

	turnOnLED(LED_ALL);
}

void prepareLocationsTableBuffer(void)
{
	g_ui32LocationsTableBufferLength = RobotLocationsTable_getSize() * SIZE_OF_ROBOT_LOCATION + 12;
//	g_pui8LocationsTableBuffer = malloc(sizeof(*g_pui8LocationsTableBuffer) * g_ui32LocationsTableBufferLength);
	g_pui8LocationsTableBuffer = new uint8_t[g_ui32LocationsTableBufferLength];
	if(g_pui8LocationsTableBuffer == 0)
		return;

	parse32bitTo4Bytes(g_pui8LocationsTableBuffer, g_RobotIdentity.Self_ID);

	int32_t i32Template;
	i32Template = (int32_t)(g_RobotIdentity.x * 65536 + 0.5);
	parse32bitTo4Bytes(&g_pui8LocationsTableBuffer[4], i32Template);

	i32Template = (int32_t)(g_RobotIdentity.y * 65536 + 0.5);
	parse32bitTo4Bytes(&g_pui8LocationsTableBuffer[8], i32Template);

	RobotLocationsTable_fillContentToByteBufferOffsetLocal(g_RobotIdentity.Self_ID, &g_pui8LocationsTableBuffer[12], g_ui32LocationsTableBufferLength - 12);
}

bool sendRequestRotateCoordinatesCommandToNeighbor(uint32_t ui32NeighborID)
{
	uint8_t pui8MessageData[4];	// <4-byte SelfId>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	DEBUG_PRINTS("send ROBOT_REQUEST_ROTATE_COORDINATES to 0x%06x\n", ui32NeighborID);

	return sendMessageToNeighbor(ui32NeighborID, ROBOT_REQUEST_ROTATE_COORDINATES, pui8MessageData, 4);
}

void setRotationFlagOfRobotTo(uint32_t ui32RobotID, bool bFlag)
{
	if (g_pCoordinatesRotationFlagTable == 0)
		return;

	int i;
	for(i = 0; i < g_i32FlagTableLength; i++)
	{
		if (g_pCoordinatesRotationFlagTable[i].ID == ui32RobotID)
		{
			g_pCoordinatesRotationFlagTable[i].isRotated = bFlag;
			return;
		}
	}
}

bool getRotationFlagOfRobot(uint32_t ui32RobotID)
{
	if (g_pCoordinatesRotationFlagTable == 0)
		return false;

	int i;
	for(i = 0; i < g_i32FlagTableLength; i++)
	{
		if (g_pCoordinatesRotationFlagTable[i].ID == ui32RobotID)
			return g_pCoordinatesRotationFlagTable[i].isRotated;
	}
	return false;
}

#endif

#ifdef REGION_STATE_FIVE_CORRECT_COORDINATES
uint8_t g_ui8NeighborsTablePointer;
uint8_t g_ui8NeighborsResponseVectorCounter;
bool g_bIsCalculatedAverageVector;
float g_fSumXAxisOfSelf;
float g_fSumYAxisOfSelf;

void StateFive_AverageVector(void)
{
	/* Pseudo-Code
	   reset counter = 0
	   DO
   	   {
	   	   call Robot timer delay [STATE_FIVE_LIFE_TIME_IN_MS] with TASK()
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
										-> call handleTransmitNeighborVectorRequest:
										-- delay random(1->100) * 50us
										-- response neighbor's vector to request robot
									-- }
									return true; // alway return true to reset task timer
							-- }

						if (isFlagAssert == true)
							reset Robot timer delay();

					} WHILE (isFlagAssert == true);

		   	   	   	// Now, robot timer delay random is expired
		   	   	   	IF I had ask all my neighbor THEN
						calculate the average
		   	   	   		// NOTE: shouldn't break the TASK, wait for task expired to keep synchronous state
		   	   	   	ELSE
		   	   	   		reset Robot timer delay();
		   	   	   		IF send Request to targetNeighbor is success THEN
		   	   	   			set targetNeighbor to next neighbor
						END
						// NOTE: shouldn't calculate the Locs Table in here due to asynchronous
					END
			   --}

		} WHILE not ask all neighbors yet

	 */

	turnOffLED(LED_ALL);

	StateFive_AverageVector_ResetFlag();

	//
	// Synchornous delay for previous state
	//
	delay_us(ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	do
	{
		activeRobotTask(AVERAGE_VECTOR_STATE_MAINTASK_LIFE_TIME_IN_MS, StateFive_AverageVector_MainTask);
	}
	while(g_ui8NeighborsTablePointer < NeighborsTable_getSize());

	setRobotState(ROBOT_STATE_IDLE); // TODO: switch to next state
//	setRobotState(ROBOT_STATE_CORRECT_LOCATIONS);
}

void StateFive_AverageVector_ResetFlag(void)
{
	g_ui8NeighborsTablePointer = 0;
	g_ui8NeighborsResponseVectorCounter = 0;
	g_bIsCalculatedAverageVector = false;
	g_fSumXAxisOfSelf = g_RobotIdentity.x;
	g_fSumYAxisOfSelf = g_RobotIdentity.y;
}

bool StateFive_AverageVector_MainTask(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.


	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask;

	do
	{
		// 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MIN, AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateFive_AverageVector_SubTask_DelayRandom_Handler);

		if (isRfFlagAssert)
			resetRobotTaskTimer();
	}
	while(isRfFlagAssert);

	// Now, robot timer delay random is expired
	if(!g_bIsCalculatedAverageVector)
	{
		if(g_ui8NeighborsTablePointer >= NeighborsTable_getSize())
		{
			g_ui8NeighborsResponseVectorCounter = NeighborsTable_getSize() + 1;

			g_RobotIdentity.x = g_fSumXAxisOfSelf / g_ui8NeighborsResponseVectorCounter;
			g_RobotIdentity.y = g_fSumYAxisOfSelf / g_ui8NeighborsResponseVectorCounter;

			g_bIsCalculatedAverageVector = true;

			turnOnLED(LED_BLUE);

			// NOTE: shouldn't break the TASK, wait for task expired to keep synchronous state
		}
		else
		{
			resetRobotTaskTimer();

			sendRequestNeighborVectorCommandToNeighbor(NeighborsTable_getIdAtIndex(g_ui8NeighborsTablePointer));

			// NOTE: shouldn't calculate the Locs Table in here due to asynchronous
		}
	}

	return false; // continue the main TASK
}

bool StateFive_AverageVector_SubTask_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// Valid commands in this state: ROBOT_REQUEST_NEIGHBOR_VECTOR, ROBOT_RESPONSE_NEIGHBOR_VECTOR and ROBOT_RESPONSE_NOT_FOUND_NEIGHBOR_VECTOR
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle
}

void StateFive_AverageVector_ReadNeighborVectorHandler(uint8_t* pui8RequestData)
{
	uint32_t ui32NeighborId = construct4Byte(pui8RequestData);

	int indexOfNeighbor = RobotLocationsTable_getIndexOfRobot(ui32NeighborId);

	if (indexOfNeighbor < 0)
	{
		reponseCommandToNeighbor(ui32NeighborId, ROBOT_RESPONSE_NOT_FOUND_NEIGHBOR_VECTOR);
	}
	else
	{
		responseNeighborVectorToRequestRobot(ui32NeighborId);
	}
}

void StateFive_AverageVector_ReceivedSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	int32_t i32Template;

	i32Template = construct4Byte(pui8MessageData);
	g_fSumXAxisOfSelf += (i32Template / 65536.0f);

	i32Template = construct4Byte(&pui8MessageData[4]);
	g_fSumYAxisOfSelf += (i32Template / 65536.0f);

	g_ui8NeighborsResponseVectorCounter++;
	g_ui8NeighborsTablePointer++;
}

void StateFive_AverageVector_NotFoundSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	g_ui8NeighborsTablePointer++;
}

void sendRequestNeighborVectorCommandToNeighbor(uint32_t ui32NeighborId)
{
	uint8_t pui8MessageData[4];	// <4-byte SelfId>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	DEBUG_PRINTS("send ROBOT_REQUEST_NEIGHBOR_VECTOR to 0x%06x\n", ui32NeighborId);

	sendMessageToNeighbor(ui32NeighborId, ROBOT_REQUEST_NEIGHBOR_VECTOR, pui8MessageData, 4);
}

void responseNeighborVectorToRequestRobot(uint32_t ui32NeighborId)
{
	uint8_t pui8MessageData[8];	// <4-byte vectNeighbor.x><4-byte vectNeighbor.y>

	float fXAxisOfNeighbor;
	float fYAxisOfNeighbor;

	if (RobotLocationsTable_getVectorOfRobot(ui32NeighborId, &fXAxisOfNeighbor, &fYAxisOfNeighbor))
	{
		int32_t i32Template;

		i32Template = (int32_t)(fXAxisOfNeighbor * 65536 + 0.5);
		parse32bitTo4Bytes(pui8MessageData, i32Template);

		i32Template = (int32_t)(fYAxisOfNeighbor * 65536 + 0.5);
		parse32bitTo4Bytes(&pui8MessageData[4], i32Template);

		responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_NEIGHBOR_VECTOR, pui8MessageData, 8);
	}
}

#endif

#ifdef REGION_STATE_SIX_SYNCH_LOCS_TABLE
uint8_t g_ui8TargetNeighborPointer;
bool g_bIsGradientSearchStopFlag;
bool g_bIsActiveCoordinatesFixing;
uint32_t g_ui32LocalLoop;

void StateSix_CorrectLocations(void)
{
	/* Pseudo code

	   IF I'm origin THEN
	   	   g_bIsActiveCoordinatesFixing = false
	   	   g_bIsGradientSearchStop = true;
	   	   force self vector to (0, 0)
		   g_ui32localLoop = 0

		   DO
		   {
			   call Robot Timer Task ()
			   Task () --{
				   DO
				   {
					   generate random values
					   clear isFlagAssert flag
					   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
						   handlerInDelayRandom()
						   -- {
									call RF handler()
									-- {

									-- }
									return true; // alway return true to reset task timer
							-- }

						if (isFlagAssert == true)
						{
							reset Robot timer delay
						}
					} WHILE (isFlagAssert == true);

					// Do nothing
					return false; // continue this task
			   -- }
		   } WHILE not ask all neighbors yet

	   ELSE
	   	   g_bIsActiveCoordinatesFixing = true
	   	   g_bIsGradientSearchStop = false;

	   	   g_ui32LocalLoop = 1

		   updateLocsByOtherRobotCurrentPosition()
		   {
			   clear locations table
			   add to locations table origin vector(0, 0)
			   add to locations table self vector(x, y)
			   g_ui8TargetNeighborPointer = 0
			   DO
			   {
				   call Robot Timer Task ()
				   Task () --{
					   DO
					   {
						   generate random values
						   clear isFlagAssert flag
						   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
							   handlerInDelayRandom()
							   -- {
										call RF handler() --{
											CASE request self vector and flag:
												IF g_bIsActiveCoordinatesFixing == false THEN
													response unactive
												ELSEIF g_ui32LocalLoop < neighborsLocalLoop THEN
													response please wait
												ELSE
													response self vector and flag
												END

											CASE response self vector and flag:
												add neighbor vector to Locations table
												g_bIsGradientSearchStopFlag &&= neighbor gradient search stop flag
												increase target neighbor pointer

											CASE response please wait
												reset task timer

											CASE response Unactive:
												increase target neighbor pointer
										--}
										return true; // alway return true to reset task timer
								-- }

							if (isFlagAssert == true)
								reset Robot timer delay();

						} WHILE (isFlagAssert == true);

						// delay random expired
						IF target id is OriginID THEN
							skip to next neighbor
						ELSE IF g_ui8TargetNeighborPointer == neighbors table count
							return true; // Terminal this task
						ELSE
							reset task timer
							send request self vector and flag with local loop
						END

						return false; // continue the main task
				   --}
				} WHILE not ask all neighbors yet
			}

			vectEstimatePosNew = current self vector
			vectGradienNew = (0, 0)

			g_bIsGradientSearchStopFlag = false;
			g_ui32LocalLoop++;

			WHILE g_bIsGradientSearchStopFlag == false
				// Algorithm 1
				WHILE g_bIsGradientSearchStopFlag == false

					updateGradient(&vectGradienNew, false);

					updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, g_fStepSize);

					synchronousLocsTableAndMyVector

					g_ui32LocalLoop++;

					g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition2);

					updateLocsByOtherRobotCurrentPosition()

					indicate local loop to LEDs
				ENDWHILE

				//Escape Local Minima
				updateGradient(&vectGradienNew, true);

				fRandomeStepSize from 2.0f to 4.0f

				updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, fRandomeStepSize);

				synchronousLocsTableAndMyVector

				g_ui32LocalLoop++;

				g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition2);

				updateLocsByOtherRobotCurrentPosition()
			ENDWHILE

			g_bIsActiveCoordinatesFixing = false;
	   END
	*/

	turnOffLED(LED_ALL);

	//
	// Synchornous delay for previous state
	//
	delay_us(AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);

	if (g_RobotIdentity.Self_ID == g_RobotIdentity.Origin_ID)
	{
		StateSix_CorrectLocations_Task1_ResetFlag();

		activeRobotTask(CORRECT_LOCATIONS_STATE_MAINTASK1_LIFE_TIME_IN_MS, StateSix_CorrectLocations_MainTask1);
	}
	else
	{
		g_bIsActiveCoordinatesFixing = true;
		g_ui32LocalLoop = 1;

		indicatesLocalLoopToLEDs();

		StateSix_CorrectLocations_UpdateLocsByOtherRobotCurrentPosition();

		Vector2<float> vectSelf(g_RobotIdentity.x, g_RobotIdentity.y);
		Vector2<float> vectEstimatePosNew = vectSelf;
		Vector2<float> vectEstimatePosOld;
		Vector2<float> vectGradienNew(0, 0);
		Vector2<float> vectGradienOld;

		g_bIsGradientSearchStopFlag = false;
		g_ui32LocalLoop++;

		while(!g_bIsGradientSearchStopFlag)
		{
			// Algorithm 1
			while(!g_bIsGradientSearchStopFlag)
			{
				GradientDescent_updateGradient(g_RobotIdentity.Self_ID, vectSelf, &vectGradienNew, false);

				GradientDescent_updatePosition(&vectSelf, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, GRADIENT_GLOBAL_STEP_SIZE);

				g_RobotIdentity.x = vectSelf.x;
				g_RobotIdentity.y = vectSelf.y;
//				RobotLocationTable_setVectorOfRobot(g_RobotIdentity.Self_ID, vectSelf.x, vectSelf.y);

				g_ui32LocalLoop++;

				g_bIsGradientSearchStopFlag = GradientDescent_checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, GRADIENT_GLOBAL_STOP_CONDITION);

				StateSix_CorrectLocations_UpdateLocsByOtherRobotCurrentPosition();

				indicatesLocalLoopToLEDs();
			}
//			//TODO: Escape Local Minima
//			updateGradient(&vectGradienNew, true);
//
//			fRandomeStepSize from 2.0f to 4.0f
//
//			updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, fRandomeStepSize);
//
//			g_RobotIdentity.x = vectSelf.x;
//			g_RobotIdentity.y = vectSelf.y;
//			RobotLocationTable_setVectorOfRobot(g_RobotIdentity.Self_ID, vectSelf.x, vectSelf.y);
//
//			g_ui32LocalLoop++;
//
//			g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition2);
//
//			updateLocsByOtherRobotCurrentPosition()
		}
		g_bIsActiveCoordinatesFixing = false;
	}

//	StateSix_CorrectLocations_ResetFlag();
//
//	//
//	// Synchornous delay for previous state
//	//
//	delay_us(AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MAX);
//
//	do
//	{
//		activeRobotTask(SYNCHRONOUS_LOCATIONS_STATE_MAINTASK_LIFE_TIME_IN_MS, StateSix_SynchronousLocations_MainTask);
//	}
//	while(g_ui8TargetNeighborPointer < NeighborsTable_getSize());


	setRobotState(ROBOT_STATE_IDLE); // TODO: switch to next state
}

void StateSix_CorrectLocations_Task1_ResetFlag(void)
{
	g_bIsActiveCoordinatesFixing = false;
	g_RobotIdentity.x = 0;
	g_RobotIdentity.y = 0;
}

bool StateSix_CorrectLocations_MainTask1(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	bool isRfFlagAssert = RfTryToCaptureRfSignal(CORRECT_LOCATIONS_STATE_MAINTASK1_LIFE_TIME_IN_MS, StateSix_CorrectLocations_SubTask1_Delay_Handler);
	if (isRfFlagAssert)
		resetRobotTaskTimer();

	return false; // Continue the main task
}

bool StateSix_CorrectLocations_SubTask1_Delay_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// No valid commands in this state
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle
}

void StateSix_CorrectLocations_UpdateLocsByOtherRobotCurrentPosition(void)
{
	StateSix_CorrectLocations_Task2_ResetFlag();

	do
	{
		activeRobotTask(CORRECT_LOCATIONS_STATE_MAINTASK2_LIFE_TIME_IN_MS, StateSix_CorrectLocations_MainTask2);
	}
	while(g_ui8TargetNeighborPointer < NeighborsTable_getSize());
}

void StateSix_CorrectLocations_Task2_ResetFlag(void)
{
	RobotLocationsTable_clear();
	RobotLocationsTable_add(g_RobotIdentity.Origin_ID, 0, 0);
	RobotLocationsTable_add(g_RobotIdentity.Self_ID, g_RobotIdentity.x, g_RobotIdentity.y);
	g_ui8TargetNeighborPointer = 0;
}

bool StateSix_CorrectLocations_MainTask2(va_list argp)
{
	// NOTE: This task must be call by activeRobotTask() because the content below call to resetRobotTaskTimer()

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument.

	bool isRfFlagAssert;
	uint32_t ui32LifeTimeInUsOfSubTask;

	do
	{
		// 100ms to 1000ms
		ui32LifeTimeInUsOfSubTask = generateRandomFloatInRange(CORRECT_LOCATIONS_STATE_SUBTASK2_LIFE_TIME_IN_US_MIN, CORRECT_LOCATIONS_STATE_SUBTASK2_LIFE_TIME_IN_US_MAX);

		isRfFlagAssert = RfTryToCaptureRfSignal(ui32LifeTimeInUsOfSubTask, StateSix_CorrectLocations_SubTask2_DelayRandom_Handler);

		if (isRfFlagAssert)
			resetRobotTaskTimer();
	}
	while(isRfFlagAssert);

	// Now, robot timer delay random is expired
	uint32_t ui32TargetId = NeighborsTable_getIdAtIndex(g_ui8TargetNeighborPointer);

	if (ui32TargetId == g_RobotIdentity.Origin_ID)
	{
		g_ui8TargetNeighborPointer++;
	}
	else if (g_ui8TargetNeighborPointer >= NeighborsTable_getSize())
	{
		return true; // Terminal this task
	}
	else
	{
		resetRobotTaskTimer();
		sendRequestSelfVectorAndFlagCommandToNeighbor(ui32TargetId);
	}

	return false; // Continue the main task
}

bool StateSix_CorrectLocations_SubTask2_DelayRandom_Handler(va_list argp)
{
	//NOTE: This task will be call every time RF interrupt pin asserted in delay random of The Main Task

	//  ARGUMENTS:
	//		va_list argp
	//			This list containt no argument

	// Valid commands in this state: ROBOT_REQUEST_SELF_VECTOR_AND_FLAG, ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG, ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_PLEASE_WAIT and ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_UNACTIVE
	handleCommonSubTaskDelayRandomState();

	return true; // Terminal the subTask after handle
}

void StateSix_CorrectLocations_ReadSelfVectorAndFlagHanlder(uint8_t* pui8RequestData)
{
	uint32_t ui32NeighborId = construct4Byte(pui8RequestData);
	uint32_t ui32NeighborLocalLoop = construct4Byte(&pui8RequestData[4]);

	if (!g_bIsActiveCoordinatesFixing)
	{
		reponseCommandToNeighbor(ui32NeighborId, ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_UNACTIVE);
	}
	else if (g_ui32LocalLoop < ui32NeighborLocalLoop)
	{
		reponseCommandToNeighbor(ui32NeighborId, ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_PLEASE_WAIT);
	}
	else
	{
		responseSelfVectorAndFlagToRequestRobot(ui32NeighborId);
	}
}

void StateSix_CorrectLocations_ReceivedSelfVectorAndFlagHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	uint32_t ui32NeighborId = construct4Byte(pui8MessageData);

	int32_t i32Template;
	i32Template = construct4Byte(&pui8MessageData[4]);
	float fXAxisOfNeighbor = i32Template / 65536.0f;

	i32Template = construct4Byte(&pui8MessageData[8]);
	float fYAxisOfNeighbor = i32Template / 65536.0f;

	bool bNeighborGradientSearchStopFlag = (pui8MessageData[12] == 0x01) ? (true) : (false);

	RobotLocationsTable_add(ui32NeighborId, fXAxisOfNeighbor, fYAxisOfNeighbor);

	g_bIsGradientSearchStopFlag = g_bIsGradientSearchStopFlag && bNeighborGradientSearchStopFlag;

	g_ui8TargetNeighborPointer++;
}

void StateSix_CorrectLocations_PleaseWaitHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	resetRobotTaskTimer();
}

void StateSix_CorrectLocations_UnActiveHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	g_ui8TargetNeighborPointer++;
}

void sendRequestSelfVectorAndFlagCommandToNeighbor(uint32_t ui32NeighborId)
{
	uint8_t pui8MessageData[8];	// <4-byte SelfId><4-byte local loop>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);
	parse32bitTo4Bytes(&pui8MessageData[4], g_ui32LocalLoop);

	DEBUG_PRINTS("send ROBOT_REQUEST_SELF_VECTOR to 0x%06x\n", ui32NeighborId);

	sendMessageToNeighbor(ui32NeighborId, ROBOT_REQUEST_SELF_VECTOR_AND_FLAG, pui8MessageData, 8);
}

void responseSelfVectorAndFlagToRequestRobot(uint32_t ui32NeighborId)
{
	uint8_t pui8MessageData[13];	// <4-byte sefId><4-byte vectSelf.x><4-byte vectSelf.y><1-byte Flag>

	parse32bitTo4Bytes(pui8MessageData, g_RobotIdentity.Self_ID);

	int32_t i32Template;

	i32Template = (int32_t)(g_RobotIdentity.x * 65536 + 0.5);
	parse32bitTo4Bytes(&pui8MessageData[4], i32Template);

	i32Template = (int32_t)(g_RobotIdentity.y * 65536 + 0.5);
	parse32bitTo4Bytes(&pui8MessageData[8], i32Template);

	pui8MessageData[12] = (g_bIsGradientSearchStopFlag == true) ? (0x01) : (0x00);

	responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG, pui8MessageData, 13);
}

void indicatesLocalLoopToLEDs(void)
{
	if (g_ui32LocalLoop & 0x01)
		turnOnLED(LED_GREEN);
	else
		turnOffLED(LED_GREEN);
}

#endif

#ifdef REGION_BASIC_CALIBRATE

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

	//uint16_t* pui16TestData = malloc(sizeof(*pui16TestData) * (ui32TestDataSize >> 1));
	uint16_t* pui16TestData = new uint16_t[ui32TestDataSize >> 1];
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

	//free(pui16TestData);
	delete[] pui16TestData;
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

void sendIrProximityValueToHost(void)
{
	triggerSamplingIrProximitySensor(true);
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

	Motors_configure(mLeftMotor, mRightMotor);
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
//	uint8_t* pui8ResponseBuffer = malloc(sizeof(*pui8ResponseBuffer) * ui8ResponseSize);
	uint8_t* pui8ResponseBuffer = new uint8_t[ui8ResponseSize];
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

//	free(pui8ResponseBuffer);
	delete[] pui8ResponseBuffer;
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
//	uint8_t* pui8ResponseBuffer = malloc(sizeof(*pui8ResponseBuffer) * ui32ResponseSize);
	uint8_t* pui8ResponseBuffer = new uint8_t[ui32ResponseSize];
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

//	free(pui8ResponseBuffer);
	delete[] pui8ResponseBuffer;
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
//	uint8_t* pui8ResponseToHostBuffer = malloc(sizeof(*pui8ResponseToHostBuffer) * ui32ResponseLength);
	uint8_t* pui8ResponseToHostBuffer = new uint8_t[ui32ResponseLength];
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

//	free(pui8ResponseToHostBuffer);
	delete[] pui8ResponseToHostBuffer;
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
	uint16_t ui16TemplatePeak;

	parse32bitTo4Bytes(pui8ResponseData, g_RobotIdentity.Self_ID);

	ui16TemplatePeak = (uint16_t)(fPeakA * 256 + 0.5);
	parse16bitTo2Bytes(&pui8ResponseData[4], ui16TemplatePeak);

	ui16TemplatePeak = (uint16_t)(fPeakB * 256 + 0.5);
	parse16bitTo2Bytes(&pui8ResponseData[6], ui16TemplatePeak);

	return responseMessageToNeighbor(ui32NeighborId, ROBOT_RESPONSE_TDOA_RESULT, pui8ResponseData, 8);
}

void robotMoveCommandWithPeriod(uint8_t* pui8Data)
{
	uint16_t ui16DelayMs = construct2Byte(&pui8Data[1]);

	Robot_move((e_RobotMoveDirection)pui8Data[0]);

	delay_ms(ui16DelayMs);

	Motors_stop();
}

void robotRotateCommandWithPeriod(uint8_t* pui8Data)
{
	uint16_t ui16DelayMs = construct2Byte(&pui8Data[1]);

	Robot_rotate((e_RobotRotateDirection)pui8Data[0]);

	delay_ms(ui16DelayMs);

	Motors_stop();
}

#endif

#ifdef REGION_CONTOLLER_CALIBRATE

void calculateNewPositionAndOrentation(RobotIdentity_t& robotIdentity, float& theta_old, Motor_t& mLeft, Motor_t& mRight);

  //========================//
 // Step Rotate Controller //
//========================//
uint8_t g_ui8StepActivateInMs;
uint8_t g_ui8StepPauseInMs;
float g_fEndThetaOfCalibrateController;
int32_t g_i32LastIMUAngle;
uint8_t g_ui8RepeatIMUAngleCounter;
void testStepRotateController(uint8_t* pui8Data)
{
	g_ui8StepActivateInMs = pui8Data[0];
	g_ui8StepPauseInMs = pui8Data[1];
	int32_t i32Angle = construct4Byte(&pui8Data[2]);
	float goalAngleInDeg = i32Angle / 65536.0f;

	float startAngle = IMU_getYawAngle();
	g_fEndThetaOfCalibrateController = startAngle + goalAngleInDeg * MATH_DEG2RAD;

	g_ui8RepeatIMUAngleCounter = 0;
	g_i32LastIMUAngle = (int32_t)(startAngle * MATH_RAD2DEG);
	DEBUG_PRINTS2("Collision test: alpha = %d, collapse %d\n", g_i32LastIMUAngle, g_ui8RepeatIMUAngleCounter);

	setRobotState(ROBOT_STATE_ROTATE_TO_ANGLE_USE_STEP);
	DEBUG_PRINT("goto state ROBOT_STATE_ROTATE_TO_ANGLE_USE_STEP\n");
}

bool rotateToAngleUseStepController(void)
{
	turnOffLED(LED_ALL);

	//g_RobotIdentity.theta = IMU_getYawAngle();
	float theta = IMU_getYawAngle();

	if(detectedRotateCollision(theta, 16))
	{
		Motors_stop();
		turnOnLED(LED_BLUE);
		DEBUG_PRINT("Detected collision...\n");
		return true;
	}

	if (isTwoAngleOverlay(theta, g_fEndThetaOfCalibrateController, CONTROLLER_ANGLE_ERROR_DEG))
	{
		Motors_stop();
		turnOffLED(LED_GREEN | LED_BLUE);
		DEBUG_PRINT("Arrived the goal!!!\n");
		return true;
	}

	float fAngleInRadian = g_fEndThetaOfCalibrateController - theta;
	fAngleInRadian = atan2f(sinf(fAngleInRadian), cosf(fAngleInRadian));

//	 if(fAngleInRadian > 0)
//	 {
//		 turnOnLED(LED_BLUE);
//		 Robot_stepRotate_tunning(ROBOT_ROTATE_CLOCKWISE, g_ui8StepActivateInMs, g_ui8StepPauseInMs);
//	 }
//	 else
//	 {
//		 turnOnLED(LED_GREEN);
//		 Robot_stepRotate_tunning(ROBOT_ROTATE_COUNTERCLOSEWISE, g_ui8StepActivateInMs, g_ui8StepPauseInMs);
//	 }

	Motor_t mRight;
	Motor_t mLeft;
	if (fAngleInRadian > 0) // ROBOT_ROTATE_CLOCKWISE
	{
		mLeft.eDirection = REVERSE;
		mRight.eDirection = FORWARD;
	}
	else // ROBOT_ROTATE_COUNTERCLOSEWISE
	{
		mLeft.eDirection = FORWARD;
		mRight.eDirection = REVERSE;
	}

	static int flagMotorSelect = 1;
	flagMotorSelect ^= 1;
	if(flagMotorSelect) // Left
	{
		turnOnLED(LED_BLUE);
		mLeft.ui8Speed = STEP_MAX_SPEED;
		mRight.ui8Speed = STEP_MIN_SPEED;
	}
	else // Right
	{
		turnOnLED(LED_GREEN);
		mLeft.ui8Speed = STEP_MIN_SPEED;
		mRight.ui8Speed = STEP_MAX_SPEED;
	}
	
	if(g_ui8StepActivateInMs > 0)
	{
		Motors_configure(mLeft, mRight);
		Motor_delay_timer_ms(g_ui8StepActivateInMs);
	}

	if(g_ui8StepPauseInMs > 0)
	{
		Motors_stop();
		Motor_delay_timer_ms(g_ui8StepPauseInMs);
	}

	calculateNewPositionAndOrentation(g_RobotIdentity, theta, mLeft, mRight);

	return false;
}

bool detectedRotateCollision(float fCurrentAngle, int times)
{
	int32_t i32CurrentAngle = (int32_t)(fCurrentAngle * MATH_RAD2DEG);

	DEBUG_PRINTS2("Collision test: g_i32LastIMUAngle = %d, i32CurrentAngle %d\n", g_i32LastIMUAngle, i32CurrentAngle);

	if(i32CurrentAngle == g_i32LastIMUAngle)
	{
		g_ui8RepeatIMUAngleCounter++;
		if(g_ui8RepeatIMUAngleCounter > times)
			return true;
		else
			return false;
	}

	g_i32LastIMUAngle = i32CurrentAngle;
	g_ui8RepeatIMUAngleCounter = 0;

	return false;
}

  //=========================//
 // Step Forward Controller //
//=========================//
uint8_t g_ui8StepFwLeftActiveMs;
uint8_t g_ui8StepFwRightActiveMs;
void testStepForwardInPeriodController(uint8_t* pui8Data)
{
	g_ui8StepFwLeftActiveMs = pui8Data[0];
	g_ui8StepFwRightActiveMs = pui8Data[1];

	uint32_t ui32StepFwPeriodInMs = construct4Byte(&pui8Data[2]);
	Motor_task_timer_start(ui32StepFwPeriodInMs);

	setRobotState(ROBOT_STATE_FORWARD_IN_PERIOD_USE_STEP);
	DEBUG_PRINT("goto state ROBOT_STATE_FORWARD_IN_PERIOD_USE_STEP\n");
}

bool forwardInPeriodUseStepController(void)
{
	static int flag = 0;
	flag ^= 1;

	Motor_t mLeft, mRight;

	mLeft.eDirection = FORWARD;
	mRight.eDirection = FORWARD;

	turnOffLED(LED_ALL);

	if(flag)
	{
		turnOnLED(LED_GREEN);

		mLeft.ui8Speed = STEP_MAX_SPEED;
		mRight.ui8Speed = STEP_MIN_SPEED;

		Motors_configure(mLeft, mRight);

		if(g_ui8StepFwLeftActiveMs > 0)
			Motor_delay_timer_ms(g_ui8StepFwLeftActiveMs);
	}
	else
	{
		turnOnLED(LED_BLUE);

		mLeft.ui8Speed = STEP_MIN_SPEED;
		mRight.ui8Speed = STEP_MAX_SPEED;

		Motors_configure(mLeft, mRight);

		if(g_ui8StepFwRightActiveMs > 0)
			Motor_delay_timer_ms(g_ui8StepFwRightActiveMs);
	}

	return Motor_task_timer_isExpired();
}

  //======================//
 // Step Step Controller //
//======================//
uint8_t g_ui8StepFwRtActivateInMs;
uint8_t g_ui8StepFwRtPauseInMs;
uint8_t g_ui8StepFwRt;
float g_pfTrackingAngle[4];
uint8_t g_ui8TrackingPoint;
void testStepForwardInRotateController(uint8_t* pui8Data)
{
	g_ui8StepFwRtActivateInMs = pui8Data[0];
	g_ui8StepFwRtPauseInMs = pui8Data[1];
	g_ui8StepFwRt = pui8Data[2];

	int32_t i32Angle = construct4Byte(&pui8Data[3]);
	float goalAngleInDeg = i32Angle / 65536.0f;

	float fCurrentAngle = IMU_getYawAngle();

	g_pfTrackingAngle[0] = fCurrentAngle - (goalAngleInDeg * MATH_DEG2RAD);
	g_pfTrackingAngle[1] = fCurrentAngle;
	g_pfTrackingAngle[2] = fCurrentAngle + (goalAngleInDeg * MATH_DEG2RAD);
	g_pfTrackingAngle[3] = fCurrentAngle;

	g_ui8TrackingPoint = 2;

	setRobotState(ROBOT_STATE_FORWARD_IN_ROTATE_USE_STEP);
	DEBUG_PRINT("goto state ROBOT_STATE_FORWARD_IN_ROTATE_USE_STEP\n");
}

bool forwardInRotateUseStepController(void)
{
	Motor_t mLeft, mRight;

	mLeft.eDirection = FORWARD;
	mRight.eDirection = FORWARD;

	turnOffLED(LED_ALL);

	//g_RobotIdentity.theta = IMU_getYawAngle();
	float theta = IMU_getYawAngle();

	if(detectedRotateCollision(theta, 6))
	{
		Motors_stop();
		turnOnLED(LED_BLUE | LED_GREEN);
		DEBUG_PRINT("Detected collision...\n");
		return true;
	}

	if (isTwoAngleOverlay(theta, g_pfTrackingAngle[g_ui8TrackingPoint], CONTROLLER_ANGLE_ERROR_DEG))
	{
		g_ui8TrackingPoint++;
		if(g_ui8TrackingPoint >= 4)
			g_ui8TrackingPoint = 0;

		g_ui8StepFwRt--;
		if(g_ui8StepFwRt == 0)
			return true; // idle
		return false; // continue
	}

	float fAngleInRadian = g_pfTrackingAngle[g_ui8TrackingPoint] - theta;
	fAngleInRadian = atan2f(sinf(fAngleInRadian), cosf(fAngleInRadian));

	if (fAngleInRadian > 0) // Lock left, move right
	{
		turnOnLED(LED_BLUE);
		mLeft.ui8Speed = STEP_MIN_SPEED;
		mRight.ui8Speed = STEP_MAX_SPEED;
	}
	else // Lock right, move left
	{
		turnOnLED(LED_GREEN);
		mLeft.ui8Speed = STEP_MAX_SPEED;
		mRight.ui8Speed = STEP_MIN_SPEED;
	}

	if(g_ui8StepFwRtActivateInMs > 0)
	{
		Motors_configure(mLeft, mRight);
		Motor_delay_timer_ms(g_ui8StepFwRtActivateInMs);
	}

	if(g_ui8StepFwRtPauseInMs > 0)
	{
		Motors_stop();
		Motor_delay_timer_ms(g_ui8StepFwRtPauseInMs);
	}

	calculateNewPositionAndOrentation(g_RobotIdentity, theta, mLeft, mRight);

	return false;
}

//------------------------------------------------------------------------------------------------------------------------
void calculateNewPositionAndOrentation(RobotIdentity_t& robotIdentity, float& theta_old, Motor_t& mLeft, Motor_t& mRight)
{
	float phi = IMU_getYawAngle() - theta_old;
	phi = atan2f(sinf(phi), cosf(phi));

	int sign;
	e_MotorDirection direction;
	if(mRight.ui8Speed - mLeft.ui8Speed > 0)
	{
		sign = 1;
		direction = mRight.eDirection;
	}
	else
	{
		sign = -1;
		direction = mLeft.eDirection;
	}

	float to;
	if (direction == FORWARD)
		to = robotIdentity.theta;
	else
		to = robotIdentity.theta + phi;

	#define R_CENTER 5.0f		// Wheel base line divived by two
	float factor = sign * R_CENTER;
	float sinTo = sinf(to);
	float cosTo = cosf(to);
	float sinPhi = sinf(phi);
	float one_minus_cosPhi = 1 - cosf(phi);

	robotIdentity.x = robotIdentity.x + factor * (sinPhi * cosTo - sinTo * one_minus_cosPhi);
	robotIdentity.y = robotIdentity.y + factor * (cosTo * one_minus_cosPhi + sinTo * sinPhi);
	robotIdentity.theta += phi;

//	float rx, ry;
//	e_MotorDirection direction;
//	if(mRight.ui8Speed - mLeft.ui8Speed > 0) // Wheel Base Line / 2
//	{
//		rx = 5.0f;
//		ry = 5.0f;
//		direction = mRight.eDirection;
//	}
//	else
//	{
//		rx = -5.0f;
//		ry = -5.0f;
//		direction = mLeft.eDirection;
//	}
//
//	int sign;
//	float theta = pRobotIdentity->theta;
//	float thetaC;
//	pRobotIdentity->theta = pRobotIdentity->theta + phi;
//	if(direction == FORWARD)
//	{
//		sign = 1;
//		thetaC = theta;
//	}
//	else
//	{
//		sign = -1;
//		thetaC = theta - phi;
//		phi = -1 * phi;
//	}
//
//	float sinThetaC = sinf(thetaC);
//	float cosThetaC = cosf(thetaC);
//	float sinPhi = sinf(phi);
//	float one_minus_cosPhi = 1 - cosf(phi);
//
//	pRobotIdentity->x = pRobotIdentity->x + sign * rx * (sinPhi * cosThetaC - sinThetaC * one_minus_cosPhi);
//	pRobotIdentity->y = pRobotIdentity->y + sign * ry * (cosThetaC * one_minus_cosPhi + sinThetaC * sinPhi);
}
//------------------------------------------------------------------------------------------------------------------------

//void testMotorLeft(bool autoSwitchState, int offset)
//{
//	turnOffLED(LED_ALL);
//	turnOnLED(LED_GREEN);
//
//	Motor_t mLeft, mRight;
//	mLeft.eDirection = FORWARD;
//	mRight.eDirection = FORWARD;
//
//	mLeft.ui8Speed = TESTONLY_MAX_PWN;
//	mRight.ui8Speed = TESTONLY_MIN_PWN;
//
//	Motors_configure(mLeft, mRight);
//	if((TESTONLY_ACTIVE_MOTORS_LEFT_MS + offset) > 0)
//		delay_ms(TESTONLY_ACTIVE_MOTORS_LEFT_MS + offset);
//
//	if(TESTONLY_PAUSE_MOTORS_MS > 0)
//	{
//		Motors_stop();
//		delay_ms(TESTONLY_PAUSE_MOTORS_MS);
//	}
//
//	if(autoSwitchState)
//	{
//		if(getRobotState() == ROBOT_STATE_TEST_MOROT_LEFT)
//			setRobotState(ROBOT_STATE_TEST_MOROT_RIGHT);
//	}
//}
//
//void testMotorRight(bool autoSwitchState, int offset)
//{
//	turnOffLED(LED_ALL);
//	turnOnLED(LED_BLUE);
//
//	Motor_t mLeft, mRight;
//	mLeft.eDirection = FORWARD;
//	mRight.eDirection = FORWARD;
//
//	mLeft.ui8Speed = TESTONLY_MIN_PWN;
//	mRight.ui8Speed = TESTONLY_MAX_PWN;
//
//	Motors_configure(mLeft, mRight);
//	if((TESTONLY_ACTIVE_MOTORS_RIGHT_MS + offset) > 0)
//		delay_ms(TESTONLY_ACTIVE_MOTORS_RIGHT_MS + offset);
//
//	if(TESTONLY_PAUSE_MOTORS_MS > 0)
//	{
//		Motors_stop();
//		delay_ms(TESTONLY_PAUSE_MOTORS_MS);
//	}
//
//	if(autoSwitchState)
//	{
//		if(getRobotState() == ROBOT_STATE_TEST_MOROT_RIGHT)
//			setRobotState(ROBOT_STATE_TEST_MOROT_LEFT);
//	}
//}

//=======================//
// Pure FwRT Controller //
//=====================//
bool g_bIsTrackingAngleOutOfDate = true;
float g_fTrackingAngle = 0;

bool forwardInRotatePureController(void)
{
	if(g_bIsTrackingAngleOutOfDate)
	{
		g_bIsTrackingAngleOutOfDate = false;
		g_fTrackingAngle = IMU_getYawAngle();
	}

	Motor_t mLeft, mRight;

	mLeft.eDirection = FORWARD;
	mRight.eDirection = FORWARD;

	turnOffLED(LED_ALL);

	g_RobotIdentity.theta = IMU_getYawAngle();

	float fAngleInRadian = g_fTrackingAngle - g_RobotIdentity.theta;
	fAngleInRadian = atan2f(sinf(fAngleInRadian), cosf(fAngleInRadian));

	if(fAngleInRadian > 0)
	{
		//Lock left, move right
		turnOnLED(LED_BLUE);

		mLeft.ui8Speed = STEP_MIN_SPEED;
		mRight.ui8Speed = STEP_MAX_SPEED;
	}
	else
	{
		//Lock right, move left
		turnOnLED(LED_GREEN);

		mLeft.ui8Speed = STEP_MAX_SPEED;
		mRight.ui8Speed = STEP_MIN_SPEED;
	}

	Motors_configure(mLeft, mRight);
	Motor_delay_timer_ms(50);

	Motors_stop();
	Motor_delay_timer_ms(75);

	if(getRobotState() == ROBOT_STATE_TEST_FORWARD_IN_ROTATE_PURE)
	{
		g_bIsTrackingAngleOutOfDate = false;
		return false;
	}

	g_bIsTrackingAngleOutOfDate = true;
	return true;
}

//=================//
// PID Controller //
//===============//
float fReferenceYawAngleInRad;
float kP, kI, kD;
float e_old, E;
void testPIDControllerSetup(uint8_t* pui8Data)
{
	int32_t i32Data;

	i32Data = construct4Byte(pui8Data);
	kP = i32Data / 65536.0f;

	i32Data = construct4Byte(&pui8Data[4]);
	kI = i32Data / 65536.0f;

	i32Data = construct4Byte(&pui8Data[8]);
	kD = i32Data / 65536.0f;

	// Init PID tracking references
	fReferenceYawAngleInRad = IMU_getYawAngle();

	// PID reset memory storages
	e_old = 0;
	E = 0;

	turnOnLED(LED_BLUE | LED_GREEN);
	setRobotState(ROBOT_STATE_TEST_PID_CONTROLLER);
}

bool testPIDController(void)
{
	// System Variables
	Motor_t m1_Left;
	Motor_t m2_Right;
	uint8_t ui8NextPWM;
	static int16_t u = 80;

	m2_Right.eDirection = FORWARD;
	m2_Right.ui8Speed = 80;

	// read current yaw angle
	g_RobotIdentity.theta = IMU_getYawAngle();

	// calculate the error
	float e = g_RobotIdentity.theta - fReferenceYawAngleInRad;
	e = atan2f(sinf(e), cosf(e));

	// calculate the error dynamic
	float e_dot = e - e_old;

	// integral the error
	E = E + e;

	// compute the control input
	u = u + kP*e + kI*E + kD*e_dot;

	// save 'e' for the next loop
	e_old = e;

	/* PWM control signal */
	if(u < 0)
	{
		u = 0 - u;
		m1_Left.eDirection = REVERSE;
	}
	else
	{
		m1_Left.eDirection = FORWARD;
	}
	ui8NextPWM = u * m2_Right.ui8Speed;

	if(ui8NextPWM < 60)
		ui8NextPWM = 60;
	else if(ui8NextPWM > 100)
		ui8NextPWM = 100;

	m1_Left.ui8Speed = ui8NextPWM;

	Motors_configure(m1_Left, m2_Right);

	if(getRobotState() == ROBOT_STATE_TEST_PID_CONTROLLER)
		return false;
	return true;
}

#endif

#ifdef REGION_DEBUG

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
	uint8_t pui8DataBuffer[120] = {0};

	turnOnLED(LED_ALL);

	RobotLocationsTable_fillContentToByteBuffer(pui8DataBuffer, 120);

	sendDataToHost(pui8DataBuffer, 120);

	turnOffLED(LED_ALL);
}

void selfCorrectLocationsTable(void)
{
	RobotLocationsTable_selfCorrectByGradientDescent(g_RobotIdentity.Self_ID, 0);
}

void selfCorrectLocationsTableExceptRotationHopID(void)
{
	RobotLocationsTable_selfCorrectByGradientDescent(g_RobotIdentity.Self_ID, g_RobotIdentity.RotationHop_ID);
}

void transmitRobotIdentityToHost(void)
{
	turnOnLED(LED_ALL);

	uint8_t pui8ResponseBuffer[35] = { 0 };

	parse32bitTo4Bytes(pui8ResponseBuffer, g_RobotIdentity.Self_ID);
	parse32bitTo4Bytes(&pui8ResponseBuffer[4], g_RobotIdentity.Origin_ID);
	parse32bitTo4Bytes(&pui8ResponseBuffer[8], g_RobotIdentity.RotationHop_ID);

	pui8ResponseBuffer[12] = g_RobotIdentity.Self_NeighborsCount;
	pui8ResponseBuffer[13] = g_RobotIdentity.Origin_NeighborsCount;
	pui8ResponseBuffer[14] = g_RobotIdentity.Origin_Hopth;

	int32_t i32Template;
	i32Template = (int32_t)(g_RobotIdentity.x * 65535 + 0.5);
	parse32bitTo4Bytes(&pui8ResponseBuffer[15], i32Template);

	i32Template = (int32_t)(g_RobotIdentity.y * 65535 + 0.5);
	parse32bitTo4Bytes(&pui8ResponseBuffer[19], i32Template);

	i32Template = (int32_t)(g_RobotIdentity.RotationHop_x * 65535 + 0.5);
	parse32bitTo4Bytes(&pui8ResponseBuffer[23], i32Template);

	i32Template = (int32_t)(g_RobotIdentity.RotationHop_y * 65535 + 0.5);
	parse32bitTo4Bytes(&pui8ResponseBuffer[27], i32Template);

	//remove:
	//g_RobotIdentity.theta = IMU_getYawAngle();

	i32Template = (int32_t)(g_RobotIdentity.theta * 65535 + 0.5);
	parse32bitTo4Bytes(&pui8ResponseBuffer[31], i32Template);

	sendDataToHost(pui8ResponseBuffer, 35);

	turnOffLED(LED_ALL);
}

void robotMoveCommandWithDistance(uint8_t* pui8Data)
{
	int32_t i32DistanceInCm = construct4Byte(pui8Data);
	float fDistanceInCm = i32DistanceInCm / 65536.0f;

	runForwardWithDistance(fDistanceInCm);
}

void robotRotateCommandWithAngle(uint8_t* pui8Data)
{
	int32_t i32AngleInDegree = construct4Byte(pui8Data);
	float fAngleInRadian = (i32AngleInDegree / 65536.0f) / _180_DIV_PI;

	rotateClockwiseWithAngle(fAngleInRadian);
}

int8_t* g_pi8ImageBuffer = 0;
uint32_t g_ui32GradientMapExpectedUpdatePacketId;

void updateGradientMap(uint8_t* pui8Data)
{
	DEBUG_PRINT("Update Gradient Map\n");

	// Get 12 bytes program header
	uint32_t ui32Height = construct4Byte(pui8Data);
	uint32_t ui32Width = construct4Byte(&pui8Data[4]);
	uint32_t ui32TrappedCount = construct4Byte(&pui8Data[8]);

	turnOffLED(LED_ALL);

	uint64_t ui64NewMapSize = ui32Height * ui32Width;
	g_pi8ImageBuffer = new int8_t[ui64NewMapSize];
	if(g_pi8ImageBuffer == 0)
		return;

	g_ui32GradientMapExpectedUpdatePacketId = 0;
	uint8_t ui8FailedToUpdateCounter = 0;
	do
	{
		turnOnLED(LED_BLUE);
		if (RfTryToCaptureRfSignal(SINGLE_PACKET_TIMEOUT_US, GradientMapUpdater_identifyPacket, ui64NewMapSize) == false)
		{
			ui8FailedToUpdateCounter++;
			if(ui8FailedToUpdateCounter > UPDATE_PACKET_WAIT_TIMES)
				break;
		}
	} while (g_ui32GradientMapExpectedUpdatePacketId < ui64NewMapSize);


	if (g_ui32GradientMapExpectedUpdatePacketId == ui64NewMapSize)
	{
		if(g_pGradientMap->modifyGradientMap(g_pi8ImageBuffer, ui32Height, ui32Width, ui32TrappedCount))
		{
			//WARNING: DO NOT DELETE g_pi8ImageBuffer because g_pGradientMap->pImage is pointing to it
			// Just only set the tempatory to NULL.
			g_pi8ImageBuffer = 0;

			turnOffLED(LED_BLUE);
		}

//		uint32_t ClockSpeed = ROM_SysCtlClockGet();
//		ROM_TimerLoadSet(TASK_TIMER_BASE, TIMER_A, ClockSpeed);
//		ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
//		ROM_TimerEnable(TASK_TIMER_BASE, TIMER_A);
//
//		g_RobotIdentity.Origin_ID = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//		g_pGradientMap->modifyGradientMap(g_pi8ImageBuffer, ui32Height, ui32Width, ui32TrappedCount);
//		g_RobotIdentity.RotationHop_ID = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//
//		ROM_TimerDisable(TASK_TIMER_BASE, TIMER_A);
//		ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
//
//		g_RobotIdentity.Self_ID = ClockSpeed;
//		g_RobotIdentity.x = (g_RobotIdentity.Origin_ID - g_RobotIdentity.RotationHop_ID) / (ClockSpeed * 1.0f);
//
//		g_pi8ImageBuffer = 0;
//		turnOffLED(LED_BLUE);
	}
	else
	{
		g_pGradientMap->reset();

		// Clean up heap
		if(g_pi8ImageBuffer != 0)
		{
			delete[] g_pi8ImageBuffer;
			g_pi8ImageBuffer = 0;
		}
	}

	/* Pseudo-code

	   --> calculate the End Packet ID: {
	   	   IF (ui64NewMapSize % PACKET_FULL_LENGTH == 0)
	   	   	   ui32LastPacketID = ((int32_t)(ui64NewMapSize / PACKET_FULL_LENGTH) - 1) * PACKET_FULL_LENGTH;
	   	   ELSE
	   	   	   ui32LastPacketID = (int32_t)(ui64NewMapSize / PACKET_FULL_LENGTH) * PACKET_FULL_LENGTH;
	   	   END
	   }

	   uint8_t ui8FailedToUpdateCounter = 0;
	   DO {
		   if (RfTryToCaptureRfSignal(SINGLE_PACKET_TIMEOUT_US, identifyPacket, ui32LastPacketID) == false) {
				   --> identifyPacket(uint32_t ui32LastPacketID) {
							IF valid length THEN
								get packet ID (uint32_t)
								get byte_count (uint8_t) to pui8GradientMapBuffer
								IF received ID match with expected ID THEN
									IF checksum (uint8_t) failed THEN
										call NACK()
										return true;
									END
									get Data (int8_t * byte_count)
									calculate next expected ID
								END
								IF received ID > expected ID THEN
									call NACK()
									return true;
								ELSE
									IF expected ID equal ui32LastPacketID THEN
										g_pGradientMap->isUpdated = true; // UPDATE SUCCESS!!!
										return true;
									END
								END
							END
							return false;
					-- }
				ui8FailedToUpdateCounter++;
				if(ui8FailedToUpdateCounter > UPDATE_PACKET_WAIT_TIMES)
					break;
			}
		} WHILE (!g_pGradientMap->isUpdated);

		IF g_pGradientMap->isUpdated is False THEN
			g_pGradientMap->reset();
		ELSE
			g_pGradientMap->modifyGradientMap(ui32Row, ui32Column, pui8GradientMapBuffer, i8OffsetHeight, i8OffsetWidth);
		END

		if(pui8GradientMapBuffer != 0)
			delete[] pui8GradientMapBuffer;
	 */
}

bool GradientMapUpdater_identifyPacket(va_list argp)
{
	//  ARGUMENTS:
	//		va_list argp
	//			This list containt one argument in order:
	//				1/ uint32_t ui64NewMapSize

	// Get the input arguments
	uint32_t ui64NewMapSize;
	ui64NewMapSize = va_arg(argp, uint32_t);

	uint32_t ui32ReceivedDataSize = 0;
	uint8_t* pui8DataHolder = 0;

	if (Network_receivedMessage(&pui8DataHolder, &ui32ReceivedDataSize))
	{
		// Process data in here:  <4-byte packet ID><1-byte byte_count><1-byte checksum><data...>
		if (ui32ReceivedDataSize > GRADIENT_MAP_PACKET_HEADER_LENGTH && ui32ReceivedDataSize <= GRADIENT_MAP_PACKET_FULL_LENGTH)
		{
			uint32_t ui32RxPacketId = construct4Byte(pui8DataHolder);
			uint8_t ui8RxByteCount = pui8DataHolder[4];

			if (ui32RxPacketId == g_ui32GradientMapExpectedUpdatePacketId)
			{
				uint32_t i;

				uint8_t ui8Checksum = 0;
				for(i = 0; i < ui32ReceivedDataSize; i++)
					ui8Checksum += pui8DataHolder[i];

				if(ui8Checksum != 0)
				{
					GradientMapUpdater_sendNACKToHost();
					Network_deleteBuffer(pui8DataHolder);
					return false;
				}

				for(i = 0; i < ui8RxByteCount; i++)
					g_pi8ImageBuffer[ui32RxPacketId + i] = pui8DataHolder[GRADIENT_MAP_PACKET_HEADER_LENGTH + i];

				g_ui32GradientMapExpectedUpdatePacketId += ui8RxByteCount;

				if (g_ui32GradientMapExpectedUpdatePacketId == ui64NewMapSize)
				{
					// UPDATE SUCCESS!!!
					Network_deleteBuffer(pui8DataHolder);
					return true;
				}
			}
			else if (ui32RxPacketId > g_ui32GradientMapExpectedUpdatePacketId)
			{
				GradientMapUpdater_sendNACKToHost();
				Network_deleteBuffer(pui8DataHolder);
				return false;
			}
			// else ui32PacketId < g_ui32GradientMapExpectedUpdatePacketId then do nothing!!!
		}
	}
	else
	{
		DEBUG_PRINT("Network_receivedMessage: Timeout...\n");
	}
	Network_deleteBuffer(pui8DataHolder);
	return false;
}

void GradientMapUpdater_sendNACKToHost(void)
{
	// Get random half-byte from random word
	uint8_t ui8Random = (g_ui32RandomW >> (4 * g_ui32RandomWIndex)) & 0x0000000F;
	g_ui32RandomWIndex++;
	g_ui32RandomWIndex &= 0x07;

	//  Delay 1ms * 3 + 50us * 3 * random
	SysCtlDelay((SysCtlClockGet() / 1000) + ((SysCtlClockGet() / 20000) * ui8Random));

	turnOnLED(LED_GREEN);
	
	MessageHeader responseNack;
	responseNack.eMessageType = MESSAGE_TYPE_ROBOT_RESPONSE;
	responseNack.ui8Cmd = ROBOT_RESPONSE_TO_HOST_NACK;

	sendDataToHost((uint8_t *) (&responseNack), 2);

	turnOffLED(LED_GREEN);
}

#endif
