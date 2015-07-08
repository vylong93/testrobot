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

#include "libstorage/inc/RobotIdentity.h"
#include "librobot/inc/robot_motor.h"

// Region definition =========================
#define REGION_ROBOT_VARIABLES_AND_FUNCTIONS
#define REGION_STATE_PERIOD_PARAMETERS
#define REGION_STATE_ONE_MEASURE_DISTANCE
#define REGION_STATE_TWO_EXCHANGE_TABLE
#define REGION_STATE_THREE_VOTE_ORIGIN
#define REGION_STATE_FOUR_ROTATE_NETWORK
#define REGION_STATE_FIVE_SYNCH_LOCS_TABLE
#define REGION_STATE_SIX_CORRECT_COORDINATES
#define REGION_STATE_SEVEN_LOCOMOTION
#define REGION_STATE_EIGHT_UPDATE_ORIENTATION
#define REGION_STATE_NINE_FOLLOW_GRADIENT_MAP
#define REGION_UPDATE_LOCATION
#define REGION_BASIC_CALIBRATE
#define REGION_DEBUG
#define REGION_CONTOLLER_CALIBRATE
#define REGION_MOVING_CONTOLLER
#define REGION_STATE_CHECKLOCATION
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
	ROBOT_STATE_UPDATE_ORIENTATION = 8,		// State Eight (optional)
	ROBOT_STATE_FOLLOW_GRADIENT_MAP = 9,	// State Nine

	ROBOT_STATE_UPDATE_LOCATION = 10,

	ROBOT_STATE_UPDATE_GRADIENT_GOAL = 11,
	ROBOT_STATE_ACTUATOR_EXECUTE = 12,

	ROBOT_STATE_ROTATE_TO_ANGLE_USE_STEP,	 	// Movement testing state
	ROBOT_STATE_FORWARD_IN_ROTATE_USE_STEP, 	// Movement testing state
	ROBOT_STATE_FORWARD_IN_PERIOD_USE_STEP,	 	// Movement testing state

	ROBOT_STATE_CHECK_LOCATION,

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
	ROBOT_RESPONSE_STATE_READ_SELF_VECTOR_AND_FLAG,
	ROBOT_RESPONSE_STATE_READ_VALID_LOCATION,
	ROBOT_RESPONSE_STATE_LOCALIZATION
} e_RobotResponseState;

typedef struct tagRobotRotationFlag{
	uint32_t ID;
	bool isRotated;
} RobotRotationFlag_t;

void test(void); // Test Only

int readChipRev(void);

void initRobotProcess(void);
void resetRobotIdentity(void);
void setRobotIdentity(void);

void setRobotState(e_RobotState eState);
e_RobotState getRobotState(void);
void switchBackToPreviousState(void);

void setRobotResponseState(e_RobotResponseState eState);
e_RobotResponseState getRobotResponseState(void);
uint8_t* getRequestMessageDataPointer(void);

void triggerResponseState(e_RobotResponseState eResponse, uint8_t* pui8RequestData, uint32_t ui32DataSize);

void blockingDelayInRobotState(uint32_t ui32MinPeriodInUs, uint32_t ui32MaxPeriodInUs);
void handleNeighborResponseSamplingCollision(void);
void broadcastNOPMessageToLocalNeighbors(void);
#endif

#ifdef REGION_STATE_PERIOD_PARAMETERS
#define SYNCHRONOUS_STATE_MARGIN_DELAY_PERIOD_IN_US			0

#define MEASURE_DISTANCE_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	200000		// 200ms
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1200000		// 1.2s

#define EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	200000		// 200ms
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	500000		// 500ms

#define VOTE_THE_OGIRIN_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	200000		// 200ms

#define ROTATE_COORDINATES_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	200000		// 200ms

#define AVERAGE_VECTOR_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	50000		// 50ms
#define AVERAGE_VECTOR_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	100000		// 100ms

#define CORRECT_LOCATIONS_STATE_MAINTASK_LIFE_TIME_IN_MS		2000	// 2s
#define CORRECT_LOCATIONS_STATE_SUBTASK_LIFE_TIME_IN_US_MIN		40000		// 40ms
#define CORRECT_LOCATIONS_STATE_SUBTASK_LIFE_TIME_IN_US_MAX		80000		// 80ms

#define LOCOMOTION_STATE_MAINTASK_LIFE_TIME_IN_MS		10000	// 10s
#define LOCOMOTION_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	2000000		// 2s
#define LOCOMOTION_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	4000000		// 4s

#define UPDATE_ORIENTATION_STATE_MAINTASK_LIFE_TIME_IN_MS	10000	// 10s
#define UPDATE_ORIENTATION_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	2000000		// 2s
#define UPDATE_ORIENTATION_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	4000000		// 4s

//#define FOLLOW_GRADIENT_MAP_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	1500000		// 1.5s
//#define FOLLOW_GRADIENT_MAP_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	2500000		// 2.5s
#define FOLLOW_GRADIENT_MAP_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	50000		// 50ms
#define FOLLOW_GRADIENT_MAP_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	250000		// 250ms

#define WAIT_FOR_LOCATION_RESPONSE_IN_US 750000 // 750ms

#define CHECK_LOCATION_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define CHECK_LOCATION_STATE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	1000000		// 1s
#define CHECK_LOCATION_STATE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	2000000		// 2s
#endif

#ifdef REGION_STATE_ONE_MEASURE_DISTANCE
void StateOne_MeasureDistance(void);
void StateOne_MeasureDistance_ResetFlag(void);
bool StateOne_MeasureDistance_MainTask(va_list argp);
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
void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData);
void StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborsTableCommandToNeighbor(uint32_t ui32NeighborId);
#endif

#ifdef REGION_STATE_THREE_VOTE_ORIGIN
#define BROADCAST_VOTE_TIMES 	3

void StateThree_VoteTheOrigin(void);
void StateThree_VoteTheOrigin_ResetFlag(void);
bool StateThree_VoteTheOrigin_MainTask(va_list argp);
void StateThree_VoteTheOrigin_VoteTheOriginHandler(uint8_t* pui8RequestData);

void broadcastVoteTheOriginCommandToLocalNeighbors(void);
void indicatesOriginIdToLEDs(uint32_t ui32Id);
#endif

#ifdef REGION_STATE_FOUR_ROTATE_NETWORK
void StateFour_RotateCoordinates(void);
bool StateFour_RotateCoordinates_ResetFlag(void);
bool StateFour_RotateCoordinates_MainTask(va_list argp);
void StateFour_RotateCoordinates_RotateCoordinatesHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_UpdateRotationFlagTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateFour_RotateCoordinates_ReadLocationsTableHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_ReceivedLocationsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void prepareLocationsTableBuffer(void);
bool sendRequestRotateCoordinatesCommandToNeighbor(uint32_t ui32NeighborID);
void setRotationFlagOfRobotTo(uint32_t ui32RobotID, bool bFlag);
bool getRotationFlagOfRobot(uint32_t ui32RobotID);
#endif

#ifdef REGION_STATE_FIVE_SYNCH_LOCS_TABLE
void StateFive_AverageVector(void);
void StateFive_AverageVector_ResetFlag(void);
bool StateFive_AverageVector_MainTask(va_list argp);
void StateFive_AverageVector_ReadNeighborVectorHandler(uint8_t* pui8RequestData);
void StateFive_AverageVector_ReceivedSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateFive_AverageVector_NotFoundSelfVectorHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborVectorCommandToNeighbor(uint32_t ui32NeighborId);
void responseNeighborVectorToRequestRobot(uint32_t ui32NeighborId);
#endif

#ifdef REGION_STATE_SIX_CORRECT_COORDINATES
//========= State 6 - Correct Locations Table ===================================
void StateSix_CorrectLocations(void);

void StateSix_CorrectLocations_Task1_ResetFlag(void);
bool StateSix_CorrectLocations_MainTask1(va_list argp);
void StateSix_CorrectLocations_ReadValidLocationHandler(uint8_t* pui8RequestData);
void StateSix_CorrectLocations_ReceivedValidLocationgHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestValidLocationCommandToNeighbor(uint32_t ui32NeighborId);
void responseValidLocationToRequestRobot(uint32_t ui32NeighborId);

void StateSix_CorrectLocations_UpdateLocsByOtherRobotCurrentPosition(void);
void StateSix_CorrectLocations_Task2_ResetFlag(void);
bool StateSix_CorrectLocations_MainTask2(va_list argp);
void StateSix_CorrectLocations_ReadSelfVectorAndFlagHanlder(uint8_t* pui8RequestData);
void StateSix_CorrectLocations_ReceivedSelfVectorAndFlagHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateSix_CorrectLocations_PleaseWaitHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateSix_CorrectLocations_UnActiveHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestSelfVectorAndFlagCommandToNeighbor(uint32_t ui32NeighborId);
void responseSelfVectorAndFlagToRequestRobot(uint32_t ui32NeighborId);
void indicatesLocalLoopToLEDs(void);
#endif

#ifdef REGION_STATE_SEVEN_LOCOMOTION
//========= State 7 - Locomotion ===================================
void StateSeven_Locomotion(void);
bool StateSeven_Locomotion_MainTask(va_list argp);
void StateSeven_Locomotion_updateLocomotionRequestHandler(uint8_t* pui8RequestData);

void broadcastLocomotionResultToLocalNeighbors(void);
#endif

#ifdef REGION_STATE_EIGHT_UPDATE_ORIENTATION
void StateEight_UpdateOrientation(void);
bool StateEight_UpdateOrientation_MainTask(va_list argp);
#endif

#ifdef REGION_STATE_NINE_FOLLOW_GRADIENT_MAP
#define MAXIMUM_MOVING_STEP_OF_FOUR	2	// 2 * 2cm = ~ 3-5cm
#define MAXIMUM_RF_CLEAR_COUNT	512
#define MAXIMUM_GOAL_STUCK_COUNT	5
void StateNine_FollowGradientMap(void);
void StateNine_FollowGradientMap_UpdateGoal();
void StateNine_FollowGradientMap_ExecuteActuator();
#endif

#ifdef REGION_STATE_CHECKLOCATION
void checkLocation(void);
bool checkLocationMainTask(va_list argp);
#endif

#ifdef REGION_UPDATE_LOCATION
bool updateLocation(void);
bool updateLocationDelayHandler(va_list argp);
void updateLocationResquestHanlder(uint8_t* pui8RequestData);
void updateLocationResponseHanlder(uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool tryToRequestLocalNeighborsFoLocolization(void);
bool responseDistanceAndLocationToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance);
void broadcastLocationMessageToLocalNeighbors(void);
void updateNeighborLocationRequestHandler(uint8_t* pui8RequestData);

#endif

#ifdef REGION_BASIC_CALIBRATE
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
#endif

#ifdef REGION_CONTOLLER_CALIBRATE
bool detectedRotateCollision(float fCurrentAngle, int windowTimes);

void testStepRotateController(uint8_t* pui8Data);
bool rotateToAngleUseStepController(void);

void testStepForwardInPeriodController(uint8_t* pui8Data);
bool forwardInPeriodUseStepController(void);

void testStepForwardInRotateController(uint8_t* pui8Data);
bool forwardInRotateUseStepController(void);
#endif

#ifdef REGION_DEBUG
void sendNeighborsTableToHost(void);
void sendOneHopNeighborsTableToHost(void);
void sendRobotLocationsTableToHost(void);
void selfCorrectLocationsTable(void);
void selfCorrectLocationsTableExceptRotationHopID(void);
void transmitRobotIdentityToHost(void);

void robotMoveCommandWithDistance(uint8_t* pui8Data);
void robotRotateCommandWithAngle(uint8_t* pui8Data);

#define GRADIENT_MAP_PACKET_HEADER_LENGTH		6
#define GRADIENT_MAP_PACKET_FULL_DATA_LENGTH 	8
#define GRADIENT_MAP_PACKET_FULL_LENGTH			14 // GRADIENT_MAP_PACKET_HEADER_LENGTH + GRADIENT_MAP_PACKET_FULL_DATA_LENGTH
#define SINGLE_PACKET_TIMEOUT_US				1000000
#define UPDATE_PACKET_WAIT_TIMES				1500

void updateGradientMap(uint8_t* pui8Data);
bool GradientMapUpdater_identifyPacket(va_list argp);
void GradientMapUpdater_sendNACKToHost(void);

#endif

#ifdef REGION_MOVING_CONTOLLER
bool moveStep(e_MotorDirection eDirection, int8_t i8StepOfFour, bool bBroadcastRF);
bool moveActivate(bool *bIsMoveCompleted, int8_t i8StepRotateLastCount, e_MotorDirection eDirection, bool bBroadcastRF);

bool rotateToAngleInRad(float fAngleInRad, bool bBroadcastRF);

bool rotateAngleInDeg(float fAngleInDeg, bool bBroadcastRF);
bool rotateAngleInRad(float fAngleInRad, bool bBroadcastRF);
bool rotateActivate(bool *bIsRotateCompleted, float fEndThetaAngle, bool bBroadcastRF);

void calculateNewRobotStateAfterRotated(float theta_old, Motor_t mLeft, Motor_t mRight);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H_ */
