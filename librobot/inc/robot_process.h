/*
 * robot_process.h
 *
 *  Created on: Feb 25, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_PROCESS_H_
#define ROBOT_PROCESS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "driverlib/rom.h"
#include "libprotocol/inc/network.h"

#include "libcustom/inc/custom_uart_debug.h"

typedef enum tag_RobotState
{
	ROBOT_STATE_IDLE = 0,
	ROBOT_STATE_MEASURE_DISTANCE = 1,	// State One
	ROBOT_STATE_EXCHANGE_TABLE = 2,		// State Two
	ROBOT_STATE_VOTE_ORIGIN = 3,		// State Three
	ROBOT_STATE_ROTATE_COORDINATES = 4,	// State Four
	ROBOT_STATE_REDUCE_ERROR,
	ROBOT_STATE_LOCOMOTION,
	ROBOT_STATE_T_SHAPE,
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
	ROBOT_RESPONSE_STATE_READ_LOCATIONS_TABLE
} e_RobotResponseState;

typedef struct tagRobotIdentity {
	uint32_t Self_ID;
	uint32_t Origin_ID;
	uint32_t RotationHop_ID;
	uint8_t Self_NeighborsCount;
	uint8_t Origin_NeighborsCount;
	uint8_t Origin_Hopth;
	float x;
	float y;
	float RotationHop_x;
	float RotationHop_y;
} RobotIdentity_t;

typedef struct tagRobotRotationFlag{
	uint32_t ID;
	bool isRotated;
} RobotRotationFlag_t;

void test(void);

void initRobotProcess(void);

void setRobotState(e_RobotState eState);
e_RobotState getRobotState(void);

void setRobotResponseState(e_RobotResponseState eState);
e_RobotResponseState getRobotResponseState(void);
uint8_t* getRequestMessageDataPointer(void);

void setRobotIdentityVector(float x, float y);

void triggerResponseState(e_RobotResponseState eResponse, uint8_t* pui8RequestData, uint32_t ui32DataSize);

void handleCommonSubTaskDelayRandomState(void);
void handleNeighborResponseSamplingCollision(void);

//========= State 1 - Measure Distances ================================
#define MEASURE_DISTANCE_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define MEASURE_DISTANCE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1000000		// 1s

#define EXCHANGE_TABLE_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define EXCHANGE_TABLE_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1000000		// 1s

#define VOTE_THE_OGIRIN_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define VOTE_THE_OGIRIN_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1000000		// 1s

#define ROTATE_COORDINATES_STATE_MAINTASK_LIFE_TIME_IN_MS		3000	// 3s
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MIN	100000		// 100ms
#define ROTATE_COORDINATES_STATE_SUBTASK_LIFE_TIME_IN_US_MAX	1000000		// 1s

void StateOne_MeasureDistance(void);
void StateOne_MeasureDistance_ResetFlag(void);
bool StateOne_MeasureDistance_MainTask(va_list argp);
bool StateOne_MeasureDistance_SubTask_DelayRandom_Handler(va_list argp);
void StateOne_MeasureDistance_SamplingMicsHandler(uint8_t* pui8RequestData);
void StateOne_MeasureDistance_UpdateNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool tryToRequestLocalNeighborsForDistanceMeasurement(void);
void broadcastMeasureDistanceCommandToLocalNeighbors(uint8_t ui8Command, int16_t i16Intercept, int16_t i16Slope);
bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance);


//========= State 2 - Exchange Table ===================================
void StateTwo_ExchangeTable(void);
void StateTwo_ExchangeTable_ResetFlag(void);
bool StateTwo_ExchangeTable_MainTask(va_list argp);
bool StateTwo_ExchangeTable_SubTask_DelayRandom_Handler(va_list argp);
void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData);
void StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborsTableCommandToNeighbor(uint32_t ui32NeighborId);


//========= State 3 - Vote The Origin ===================================
#define BROADCAST_VOTE_TIMES 	3

void StateThree_VoteTheOrigin(void);
void StateThree_VoteTheOrigin_ResetFlag(void);
bool StateThree_VoteTheOrigin_MainTask(va_list argp);
bool StateThree_VoteTheOrigin_SubTask_DelayRandom_Handler(va_list argp);
void StateThree_VoteTheOrigin_VoteTheOriginHandler(uint8_t* pui8RequestData);

void broadcastVoteTheOriginCommandToLocalNeighbors(void);
void indicatesOriginIdToLEDs(uint32_t ui32Id);

//========= State 4 - Rotate Network Coordinates ===================================
void StateFour_RotateCoordinates(void);
bool StateFour_RotateCoordinates_ResetFlag(void);
bool StateFour_RotateCoordinates_MainTask(va_list argp);
bool StateFour_RotateCoordinates_SubTask_DelayRandom_Handler(va_list argp);
void StateFour_RotateCoordinates_RotateCoordinatesHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_UpdateRotationFlagTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);
void StateFour_RotateCoordinates_ReadLocationsTableHandler(uint8_t* pui8RequestData);
void StateFour_RotateCoordinates_ReceivedLocationsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool sendRequestRotateCoordinatesCommandToNeighbor(uint32_t ui32NeighborID);
void setRotationFlagOfRobotTo(uint32_t ui32RobotID, bool bFlag);
bool getRotationFlagOfRobot(uint32_t ui32RobotID);

//========= Calibration Tab ============================================
void testRfReceiver(uint8_t* pui8Data);
bool checkForCorrectRxDataStream(va_list argp);
void testRfTransmister(uint8_t* pui8Data);
void sendBatteryVoltageToHost(void);
void indicateBatteryVoltage(void);
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
void testPIDController(uint8_t* pui8Data);

void sendNeighborsTableToHost(void);
void sendOneHopNeighborsTableToHost(void);
void sendRobotLocationsTableToHost(void);
void selfCorrectLocationsTable(void);
void selfCorrectLocationsTableExceptRotationHopID(void);
void transmitRobotIdentityToHost(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H_ */
