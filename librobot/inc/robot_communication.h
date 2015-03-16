/*
 * robot_communication.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_COMMUNICATION_H_
#define ROBOT_COMMUNICATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#ifdef RF_USE_CC2500
#include "libcc2500/inc/TM4C123_CC2500.h"
#include "libcc2500/inc/cc2500.h"
#endif

#ifdef RF_USE_nRF24L01
#include "libnrf24l01/inc/nRF24L01.h"
#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#endif

#include "libprotocol/inc/network.h"

#define RF_DEFAULT_ROBOT_ID		0x00BEAD00
#define RF_CONTOLBOARD_ADDR		0x00C1AC02

typedef enum tag_MessageType
{
	MESSAGE_TYPE_HOST_REQUEST = 0x00,
	MESSAGE_TYPE_HOST_COMMAND = 0x01,
	MESSAGE_TYPE_ROBOT_REQUEST = 0x02,
	MESSAGE_TYPE_ROBOT_RESPONSE = 0x03,
	MESSAGE_TYPE_SMARTPHONE_REQUEST = 0x04,
	MESSAGE_TYPE_SMARTPHONE_COMMAND = 0x05
} e_MessageType;

typedef struct tag_MessageHeader
{
	// WARNING!!! Do not change this order unless you know what you are doing!
	e_MessageType eMessageType :8;
	uint8_t ui8Cmd;
} MessageHeader;

#define MESSAGE_HEADER_LENGTH		2
#define MESSAGE_TYPE_IDX			0
#define MESSAGE_COMMAND_IDX			1
#define MESSAGE_DATA_START_IDX		MESSAGE_HEADER_LENGTH

//----------------------------------------------------
#define HOST_COMMAND_RESET			0x01
#define HOST_COMMAND_SLEEP			0x02
#define	HOST_COMMAND_DEEP_SLEEP		0x03
#define	HOST_COMMAND_WAKE_UP		0x04

#define	HOST_COMMAND_TEST_RF_TRANSMISTER		0x05
#define	HOST_COMMAND_TEST_RF_RECEIVER			0x06
#define	HOST_COMMAND_TOGGLE_ALL_STATUS_LEDS		0x07
#define	HOST_COMMAND_START_SAMPLING_MIC			0x08
#define	HOST_COMMAND_DATA_ADC0_TO_HOST			0x09
#define	HOST_COMMAND_DATA_ADC1_TO_HOST			0x0A
#define HOST_COMMAND_REQUEST_BATT_VOLT			0x0B
#define	HOST_COMMAND_START_SPEAKER				0x0C
#define	HOST_COMMAND_CHANGE_MOTORS_SPEED		0x0D
#define	HOST_COMMAND_STOP_MOTOR_LEFT			0x0E
#define	HOST_COMMAND_STOP_MOTOR_RIGHT			0x0F

#define HOST_COMMAND_EEPROM_DATA_READ			0x10
#define HOST_COMMAND_EEPROM_DATA_WRITE			0x11
#define HOST_COMMAND_EEPROM_DATA_READ_BULK		0x12
#define HOST_COMMAND_EEPROM_DATA_WRITE_BULK		0x13

#define HOST_COMMAND_CONFIG_PID_CONTROLLER		0x14
#define HOST_COMMAND_CALIBRATE_TDOA_TX			0x15

#define HOST_COMMAND_INDICATE_BATT_VOLT				0x16
#define HOST_COMMAND_START_LOCALIZATION				0x17
#define HOST_COMMAND_READ_NEIGHBORS_TABLE			0x18
#define HOST_COMMAND_READ_ONEHOP_NEIGHBORS_TABLE	0x19
#define HOST_COMMAND_READ_LOCATIONS_TABLE			0x1A

//------------------------------------------------------
#define ROBOT_RESPONSE_TO_HOST_OK 					0x0A
#define ROBOT_RESPONSE_TDOA_RESULT					0xB0
#define ROBOT_RESPONSE_DISTANCE_RESULT				0xB1
#define ROBOT_RESPONSE_SAMPLING_COLLISION			0xB2
#define ROBOT_RESPONSE_NEIGHBORS_TABLE				0xB3
//------------------------------------------------------
#define ROBOT_REQUEST_CALIBRATE_SAMPLING_MICS		0xA0
#define ROBOT_REQUEST_SAMPLING_MICS					0xA1
#define ROBOT_REQUEST_NEIGHBORS_TABLE				0xA2
//------------------------------------------------------

void RobotResponseIntHandler(void);
void decodeMessage(uint8_t* pui8Message, uint32_t ui32MessSize);

bool decodeBasicHostCommand(uint8_t ui8Cmd);
void decodeAdvanceHostCommand(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize);
void decodeRobotRequestMessage(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize);
void decodeRobotResponseMessage(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool sendMessageToHost(e_MessageType eMessType, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize);
bool sendDataToHost(uint8_t* pui8Data, uint32_t ui32DataLength);

void broadcastToLocalNeighbors(uint8_t ui8Command, uint8_t* pui8MessageData, uint8_t ui32DataSize);

void broatcastMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command, uint8_t* pui8MessageData, uint32_t ui32DataSize);
void broadcastDataToNeighbor(uint32_t ui32NeighborId, uint8_t* pui8Data, uint32_t ui32DataSize);

bool responseMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize);
bool sendMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize);
bool reponseCommandToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command);
bool transmitMessageToNeighbor(e_MessageType eMessType, uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize);

bool sendMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command, uint8_t* pui8MessageData, uint32_t ui32DataSize);
bool sendDataToNeighbor(uint32_t ui32NeighborId, uint8_t* pui8Data, uint32_t ui32DataSize);

void constructMessage(uint8_t* puiMessageBuffer, e_MessageType eMessType, uint8_t ui8Command, uint8_t* pui8Data, uint32_t ui32DataSize);

//=============================================================================
#define PC_SEND_READ_NEIGHBORS_TABLE	0xA6
#define PC_SEND_READ_ONEHOP_TABLE		0xA7
#define PC_SEND_READ_LOCS_TABLE			0xA8

#define PC_SEND_READ_VECTOR				0xB1
#define PC_SEND_LOCAL_LOOP_STOP			0xB2
#define PC_SEND_SET_STEPSIZE			0xB3
#define PC_SEND_SET_STOP_CONDITION_ONE	0xB4
#define PC_SEND_SET_STOP_CONDITION_TWO	0xB5
#define PC_SEND_ROTATE_CLOCKWISE		0xB6
#define PC_SEND_ROTATE_CLOCKWISE_ANGLE	0xB7
#define PC_SEND_FORWARD_PERIOD			0xB8
#define PC_SEND_FORWARD_DISTANCE		0xB9
#define PC_SEND_SET_ROBOT_STATE			0xBA
#define PC_SEND_ROTATE_CORRECTION_ANGLE	0xBB
#define PC_SEND_READ_CORRECTION_ANGLE	0xBC

#define PC_SEND_ROTATE_CORRECTION_ANGLE_DIFF	0xBD
#define PC_SEND_ROTATE_CORRECTION_ANGLE_SAME	0xBE

#define ROBOT_RESPONSE_HELLO_NEIGHBOR			0xD2
#define ROBOT_RESPONSE_NOT_YOUR_NEIGHBOR 		0xD3
#define ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN		0xD4
#define ROBOT_REQUEST_ROTATE_NETWORK			0xD5
#define ROBOT_REQUEST_MY_VECTOR					0xD6
#define ROBOT_RESPONSE_MY_VECTOR_PLEASE_WAIT	0xD7
#define ROBOT_RESPONSE_MY_VECTOR				0xD8
#define ROBOT_RESPONSE_MY_VECTOR_NOT_FOUND 		0xD9
#define ROBOT_REQUEST_VECTOR_AND_FLAG			0xDA
#define ROBOT_RESPONSE_VECTOR_AND_FLAG			0xDB
#define ROBOT_RESPONSE_PLEASE_WAIT				0xDC
#define ROBOT_RESPONSE_UNACTIVE					0xDD
#define ROBOT_REQUEST_VECTOR					0xDE
#define ROBOT_RESPONSE_VECTOR					0xDF

#define ROBOT_REQUEST_TO_RUN				0x90
#define ROBOT_REQUEST_UPDATE_VECTOR			0x93
#define ROBOT_ALLOW_MOVE_TO_T_SHAPE			0x94
#define ROBOT_REPONSE_MOVE_COMPLETED		0x95

// Format <SMART_PHONE_COMMAND><SP_SEND_...>
#define SMART_PHONE_COMMAND				0xF0
#define SP_SEND_STOP_TWO_MOTOR			0xF1
#define SP_SEND_FORWAR					0xF2
#define SP_SEND_SPIN_CLOCKWISE			0xF3
#define SP_SEND_SPIN_COUNTERCLOCKWISE	0xF4
#define SP_SEND_RESERVED				0xF5

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_COMMUNICATION_H_ */
