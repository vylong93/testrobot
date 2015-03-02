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
	ROBOT_STATE_MEASURE_DISTANCE,
	ROBOT_STATE_EXCHANGE_TABLE,
	ROBOT_STATE_VOTE_ORIGIN,
	ROBOT_STATE_ROTATE_NETWORK,
	ROBOT_STATE_REDUCE_ERROR,
	ROBOT_STATE_LOCOMOTION,
	ROBOT_STATE_T_SHAPE,
} e_RobotState;

typedef enum tag_RobotResponseState
{
	ROBOT_RESPONSE_STATE_NONE = 0,
	ROBOT_RESPONSE_STATE_SAMPLING_MICS,
	ROBOT_RESPONSE_STATE_TRIGGER_SPEAKER
} e_RobotResponseState;

void initRobotProcess(void);

void setRobotState(e_RobotState eState);
e_RobotState getRobotState(void);

void setRobotResponseState(e_RobotResponseState eState);
e_RobotResponseState getRobotResponseState(void);
uint8_t* getRequestMessageDataPointer(void);

void triggerResponseState(e_RobotResponseState eResponse, uint8_t* pui8RequestData, uint32_t ui32DataSize);
bool tryToRequestLocalNeighborsForDistanceMeasurement(void);

void testRfReceiver(uint8_t* pui8Data);
bool checkForCorrectRxDataStream(va_list argp);
void testRfTransmister(uint8_t* pui8Data);
void sendBatteryVoltageToHost(void);
void modifyMotorsConfiguration(uint8_t* pui8Data);
void transmitADCResultsToHost(uint8_t* pui8Buffer);
void transmitRequestDataInEeprom(uint8_t* pui8Data);
void synchronousEepromData(uint8_t* pui8Data);
void writeBulkToEeprom(uint8_t* pui8Data);
void transmitRequestBulkDataInEeprom(uint8_t* pui8Data);

void calibrationTx_TDOA(uint8_t* pui8Data);
bool isCorrectTDOAResponse(va_list argp);
void responseSamplingMics(uint32_t ui32RequestRobotID);
bool responseTDOAResultsToNeighbor(uint32_t ui32NeighborId, float fPeakA, float fPeakB);

void broadcastCommandWithSelfIdToLocalNeighbors(uint8_t ui8Command);
bool responseMeasuredDistanceToNeighbor(uint32_t ui32NeighborId, float fPeakA, float fPeakB);

void testPIDController(uint8_t* pui8Data);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_PROCESS_H_ */
