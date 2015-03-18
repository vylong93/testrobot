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
	ROBOT_STATE_MEASURE_DISTANCE,	// State One
	ROBOT_STATE_EXCHANGE_TABLE,		// State Two
	ROBOT_STATE_VOTE_ORIGIN,
	ROBOT_STATE_ROTATE_NETWORK,
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
	ROBOT_RESPONSE_STATE_TRANSMIT_NEIGHBORS_TABLE
} e_RobotResponseState;

void initRobotProcess(void);

void setRobotState(e_RobotState eState);
e_RobotState getRobotState(void);

void setRobotResponseState(e_RobotResponseState eState);
e_RobotResponseState getRobotResponseState(void);
uint8_t* getRequestMessageDataPointer(void);

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

void StateOne_MeasureDistance(void);
void StateOne_MeasureDistance_ResetFlag(void);
bool StateOne_MeasureDistance_MainTask(va_list argp);
bool StateOne_MeasureDistance_SubTask_DelayRandom_Handler(va_list argp);
void StateOne_MeasureDistance_UpdateNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool tryToRequestLocalNeighborsForDistanceMeasurement(void);
void broadcastMeasureDistanceCommandToLocalNeighbors(uint8_t ui8Command, int16_t i16Intercept, int16_t i16Slope);
void handleSamplingMicsRequest(uint8_t* pui8RequestData);
bool responseDistanceToNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance);


//========= State 2 - Exchange Table ===================================
void StateTwo_ExchangeTable(void);
void StateTwo_ExchangeTable_ResetFlag(void);
bool StateTwo_ExchangeTable_MainTask(va_list argp);
bool StateTwo_ExchangeTable_SubTask_DelayRandom_Handler(va_list argp);
void StateTwo_ExchangeTable_TransmitNeighborsTableHandler(uint8_t* pui8RequestData);
void StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(uint8_t* pui8MessageData, uint32_t ui32DataSize);

void sendRequestNeighborsTableCommandToNeighbor(uint32_t ui32NeighborId);

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

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H_ */
