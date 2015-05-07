/*
 * robot_process.h
 *
 *  Created on: Feb 25, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_PROCESS_H_
#define ROBOT_PROCESS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "librobot/inc/robot_motor.h"

// Region definition =========================
#define REGION_ROBOT_VARIABLES_AND_FUNCTIONS
#define REGION_STATE_PERIOD_PARAMETERS
#define REGION_STATE_ONE_MEASURE_DISTANCE
#define REGION_STATE_TWO_EXCHANGE_TABLE
#define REGION_STATE_THREE_VOTE_ORIGIN
#define REGION_STATE_FOUR_ROTATE_NETWORK
#define REGION_STATE_FIVE_CORRECT_COORDINATES
#define REGION_STATE_SIX_SYNCH_LOCS_TABLE
#define REGION_BASIC_CALIBRATE
#define REGION_DEBUG
#define REGION_CONTOLLER_CALIBRATE
//============================================

#ifdef REGION_ROBOT_VARIABLES_AND_FUNCTIONS
typedef enum tag_RobotState
{
	ROBOT_STATE_IDLE = 0,
	ROBOT_STATE_MEASURE_DISTANCE = 1,		// State One
	ROBOT_STATE_EXCHANGE_TABLE = 2,			// State Two
	ROBOT_STATE_VOTE_ORIGIN = 3,			// State Three
	ROBOT_STATE_ROTATE_COORDINATES = 4,		// State Four
	ROBOT_STATE_AVERAGE_VECTOR = 5,			// State Five
	ROBOT_STATE_CORRECT_LOCATIONS = 6,  	// State Six
	ROBOT_STATE_LOCOMOTION = 7,				// State Seven (optional)
	ROBOT_STATE_ROTATE_TO_ANGLE_USE_PID = 8,	// PID Controller
	ROBOT_STATE_ROTATE_TO_ANGLE_USE_CAL = 9,	// Calibrate Controller
	ROBOT_STATE_MOVE_FORWARD_USE_PID = 10,
	ROBOT_STATE_TEST_MOROT_LEFT = 11,
	ROBOT_STATE_TEST_MOROT_RIGHT = 12,
	ROBOT_STATE_CORRECT_ROTATE_TO_ANGLE = 13,
} e_RobotState;

typedef enum tag_RobotResponseState
{
	ROBOT_RESPONSE_STATE_NONE = 0,
	ROBOT_RESPONSE_STATE_CALIBRATE_SAMPLING_MICS,
	ROBOT_RESPONSE_STATE_CALIBRATE_TRIGGER_SPEAKER,
	ROBOT_RESPONSE_STATE_SAMPLING_BATTERY,
	ROBOT_RESPONSE_STATE_SAMPLING_MICS,
	ROBOT_RESPONSE_STATE_TRANSMIT_NEIGHBORS_TABLE,
	ROBOT_RESPONSE_STATE_ROTATE_COORDINATES,
	ROBOT_RESPONSE_STATE_READ_LOCATIONS_TABLE,
	ROBOT_RESPONSE_STATE_READ_NEIGHBOR_VECTOR,
	ROBOT_RESPONSE_STATE_READ_SELF_VECTOR_AND_FLAG
} e_RobotResponseState;

typedef struct tagRobotRotationFlag{
	uint32_t ID;
	bool isRotated;
} RobotRotationFlag_t;

void test(void); // Test Only

void initRobotProcess(void);
void resetRobotIdentity(void);

void setRobotState(e_RobotState eState);
e_RobotState getRobotState(void);

void setRobotResponseState(e_RobotResponseState eState);
e_RobotResponseState getRobotResponseState(void);
uint8_t* getRequestMessageDataPointer(void);

void triggerResponseState(e_RobotResponseState eResponse, uint8_t* pui8RequestData, uint32_t ui32DataSize);

void handleCommonSubTaskDelayRandomState(void);
void handleNeighborResponseSamplingCollision(void);
#endif

#ifdef REGION_STATE_PERIOD_PARAMETERS
#define MEASURE_DISTANCE_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1000000		// 1s

#define EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	50000		// 50ms
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	500000		// 500ms

#define VOTE_THE_OGIRIN_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	10000		// 10ms
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	100000		// 100ms

#define ROTATE_COORDINATES_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	10000		// 10ms
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	100000		// 100ms

#define AVERAGE_VECTOR_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	10000		// 100ms
#define AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	100000		// 1s

#define CORRECT_LOCATIONS_STATE_MAINTASK1_LIFE_TIME_IN_MS		2000	// 2s
#define CORRECT_LOCATIONS_STATE_MAINTASK2_LIFE_TIME_IN_MS		2000	// 2s
#define CORRECT_LOCATIONS_STATE_SUBTASK2_LIFE_TIME_IN_US_MIN		10000		// 10ms
#define CORRECT_LOCATIONS_STATE_SUBTASK2_LIFE_TIME_IN_US_MAX		100000		// 100ms
#endif

#ifdef REGION_STATE_ONE_MEASURE_DISTANCE
void StateOne_MeasureDistance(void);
void StateOne_MeasureDistance_ResetFlag(void);
bool StateOne_MeasureDistance_MainTask(va_list argp);
bool StateOne_MeasureDistance_SubTask_DelayRandom_Handler(va_list argp);
void StateOne_MeasureDistance_SamplingMicsHandler(uint8_t* pui8RequestData);
void StateOne_MeasureDistance_UpdateNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool tryToRequestLocalNeighborsForDistanceMeasurement(void);
void broadcastMeasureDistanceCommandToLocalNeighbors(uint8_t ui8Command, int16_t i16Intercept, int16_t i16Slope);
bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance);
#endif

#ifdef REGION_STATE_TWO_EXCHANGE_TABLE
void StateTwo_ExchangeTable(void);
void StateTwo_ExchangeTable_ResetFlag(void);
bool StateTwo_ExchangeTable_MainTask(va_list argp);
bool StateTwo_ExchangeTable_SubTask_DelayRandom_Handler(va_list argp);
void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData);
void StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborsTableCommandToNeighbor(uint32_t ui32NeighborId);
#endif

#ifdef REGION_STATE_THREE_VOTE_ORIGIN
#define BROADCAST_VOTE_TIMES 	3

void StateThree_VoteTheOrigin(void);
void StateThree_VoteTheOrigin_ResetFlag(void);
bool StateThree_VoteTheOrigin_MainTask(va_list argp);
bool StateThree_VoteTheOrigin_SubTask_DelayRandom_Handler(va_list argp);
void StateThree_VoteTheOrigin_VoteTheOriginHandler(uint8_t* pui8RequestData);

void broadcastVoteTheOriginCommandToLocalNeighbors(void);
void indicatesOriginIdToLEDs(uint32_t ui32Id);
#endif

#ifdef REGION_STATE_FOUR_ROTATE_NETWORK
void StateFour_RotateCoordinates(void);
bool StateFour_RotateCoordinates_ResetFlag(void);
bool StateFour_RotateCoordinates_MainTask(va_list argp);
bool StateFour_RotateCoordinates_SubTask_DelayRandom_Handler(va_list argp);
void StateFour_RotateCoordinates_RotateCoordinatesHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_UpdateRotationFlagTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateFour_RotateCoordinates_ReadLocationsTableHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_ReceivedLocationsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void prepareLocationsTableBuffer(void);
bool sendRequestRotateCoordinatesCommandToNeighbor(uint32_t ui32NeighborID);
void setRotationFlagOfRobotTo(uint32_t ui32RobotID, bool bFlag);
bool getRotationFlagOfRobot(uint32_t ui32RobotID);
#endif

#ifdef REGION_STATE_FIVE_CORRECT_COORDINATES
void StateFive_AverageVector(void);
void StateFive_AverageVector_ResetFlag(void);
bool StateFive_AverageVector_MainTask(va_list argp);
bool StateFive_AverageVector_SubTask_DelayRandom_Handler(va_list argp);
void StateFive_AverageVector_ReadNeighborVectorHandler(uint8_t* pui8RequestData);
void StateFive_AverageVector_ReceivedSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateFive_AverageVector_NotFoundSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborVectorCommandToNeighbor(uint32_t ui32NeighborId);
void responseNeighborVectorToRequestRobot(uint32_t ui32NeighborId);
#endif

#ifdef REGION_STATE_SIX_SYNCH_LOCS_TABLE
//========= State 6 - Correct Locations Table ===================================
void StateSix_CorrectLocations(void);

void StateSix_CorrectLocations_Task1_ResetFlag(void);
bool StateSix_CorrectLocations_MainTask1(va_list argp);
bool StateSix_CorrectLocations_SubTask1_Delay_Handler(va_list argp);

void StateSix_CorrectLocations_UpdateLocsByOtherRobotCurrentPosition(void);
void StateSix_CorrectLocations_Task2_ResetFlag(void);
bool StateSix_CorrectLocations_MainTask2(va_list argp);
bool StateSix_CorrectLocations_SubTask2_DelayRandom_Handler(va_list argp);
void StateSix_CorrectLocations_ReadSelfVectorAndFlagHanlder(uint8_t* pui8RequestData);
void StateSix_CorrectLocations_ReceivedSelfVectorAndFlagHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateSix_CorrectLocations_PleaseWaitHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateSix_CorrectLocations_UnActiveHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestSelfVectorAndFlagCommandToNeighbor(uint32_t ui32NeighborId);
void responseSelfVectorAndFlagToRequestRobot(uint32_t ui32NeighborId);
void indicatesLocalLoopToLEDs(void);
#endif

#ifdef REGION_BASIC_CALIBRATE
//========= Calibration Tab ============================================
void testRfReceiver(uint8_t* pui8Data);
bool checkForCorrectRxDataStream(va_list argp);
void testRfTransmister(uint8_t* pui8Data);
void sendBatteryVoltageToHost(void);
void indicateBatteryVoltage(void);
void sendIrProximityValueToHost(void);
void modifyMotorsConfiguration(uint8_t* pui8Data);
void transmitADCResultsToHost(uint8_t* pui8Buffer);
void transmitRequestDataInEeprom(uint8_t* pui8Data);
void synchronousEepromData(uint8_t* pui8Data);
void writeBulkToEeprom(uint8_t* pui8Data);
void transmitRequestBulkDataInEeprom(uint8_t* pui8Data);

void calibrationTx_TDOA(uint8_t* pui8Data);
bool tryToCalibrateLocalNeighborsForDistanceMeasurement(void);
bool isCorrectTDOAResponse(va_list argp);
void handleCalibrateSamplingMicsRequest(uint8_t* pui8RequestData);
bool responseTDOAResultsToNeighbor(uint32_t ui32NeighborId, float fPeakA, float fPeakB);

void robotMoveCommandWithPeriod(uint8_t* pui8Data);
void robotRotateCommandWithPeriod(uint8_t* pui8Data);
void robotMoveCommandWithDistance(uint8_t* pui8Data);
void robotRotateCommandWithAngle(uint8_t* pui8Data);

void sendNeighborsTableToHost(void);
void sendOneHopNeighborsTableToHost(void);
void sendRobotLocationsTableToHost(void);
void selfCorrectLocationsTable(void);
void selfCorrectLocationsTableExceptRotationHopID(void);
void transmitRobotIdentityToHost(void);
#endif

#ifdef REGION_CONTOLLER_CALIBRATE
void testPIDControllerForward(uint8_t* pui8Data);
void testPIDController(uint8_t* pui8Data);
void testCalibrateController(uint8_t* pui8Data);

bool moveForwardUseControllerFW(void);
bool rotateToAngleUseControllerRTA(void);
bool rotateToAngleUseCalibrateController(void);
void commandMotorLeftFast(e_MotorDirection leftD, e_MotorDirection rightD);
void commandMotorRightFast(e_MotorDirection leftD, e_MotorDirection rightD);

bool commandTwoMotors(void);
void commandMotorLeft(e_MotorDirection leftD, e_MotorDirection rightD);
void commandMotorRight(e_MotorDirection leftD, e_MotorDirection rightD);
bool isTwoAngleOverlay(float a, float b, float errorInDeg);
void applyUnicycleToMotorCommand(float w);
#endif

void testMotorLeft(void);
void testMotorRight(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H_ */
