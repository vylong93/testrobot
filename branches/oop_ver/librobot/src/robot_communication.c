/*
 * robot_communication.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_communication.h"
#include "librobot/inc/robot_process.h"
#include "librobot/inc/robot_lpm.h"
#include "librobot/inc/robot_speaker.h"
#include "librobot/inc/robot_analog.h"
#include "librobot/inc/robot_motor.h"
#include "librobot/inc/robot_eeprom.h"

#include "libcustom/inc/custom_led.h"

#include "libalgorithm/inc/TDOA.h"

//#define Robot_TX_DELAY
#define Robot_TX
//#define Robot_RX

//------------------------ Message Decoder ------------------------
void RobotResponseIntHandler(void)
{
	uint8_t* pui8RequestData = getRequestMessageDataPointer();

	switch (getRobotResponseState())
	{
		case ROBOT_RESPONSE_STATE_CALIBRATE_SAMPLING_MICS:
			handleCalibrateSamplingMicsRequest(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_CALIBRATE_TRIGGER_SPEAKER:
			calibrationTx_TDOA(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_SAMPLING_BATTERY:
			indicateBatteryVoltage();
			break;

		case ROBOT_RESPONSE_STATE_SAMPLING_MICS:
			handleSamplingMicsRequest(pui8RequestData);
			break;

		default:	// ROBOT_RESPONSE_STATE_NONE
			break;
	}
	setRobotResponseState(ROBOT_RESPONSE_STATE_NONE);

	free(pui8RequestData);
}

void decodeMessage(uint8_t* pui8MessageBuffer, uint32_t ui32MessSize)
{
	MessageHeader* pMessageHeader = (MessageHeader*) pui8MessageBuffer;

	if (getCpuMode() != CPU_MODE_RUN)
	{
		if (pMessageHeader->eMessageType != MESSAGE_TYPE_HOST_COMMAND)
			returnToSleep();

		if (!decodeBasicHostCommand(pMessageHeader->ui8Cmd))
			returnToSleep();
	}
	else
	{
		switch (pMessageHeader->eMessageType)
		{
		case MESSAGE_TYPE_HOST_COMMAND:
			decodeAdvanceHostCommand(pMessageHeader->ui8Cmd, &pui8MessageBuffer[MESSAGE_DATA_START_IDX], ui32MessSize - MESSAGE_HEADER_LENGTH);
			break;

		case MESSAGE_TYPE_ROBOT_REQUEST:
			decodeRobotRequestMessage(pMessageHeader->ui8Cmd, &pui8MessageBuffer[MESSAGE_DATA_START_IDX], ui32MessSize - MESSAGE_HEADER_LENGTH);
			break;

		case MESSAGE_TYPE_ROBOT_RESPONSE:
			decodeRobotResponseMessage(pMessageHeader->ui8Cmd, &pui8MessageBuffer[MESSAGE_DATA_START_IDX], ui32MessSize - MESSAGE_HEADER_LENGTH);
			break;

		default:
			break;
		}

//		case ROBOT_REQUEST_SAMPLING_MIC:
//			// DO NOT INSERT ANY CODE IN HERE!
//			startSamplingMicSignals();
//			if (g_bIsValidVector)
//			{
//				turnOnLED(LED_GREEN);
//				g_ui32RequestRobotID = construct4BytesToUint32(&RF24_RX_buffer[1]);
//				g_eRobotResponseState = TDOA;
//				IntTrigger(INT_SW_TRIGGER_ROBOT_RESPONSE);
//			}
//			break;
//
//		// EXCHANGE_TABLE state
//		case ROBOT_REQUEST_NEIGHBORS_TABLE:
//			reloadDelayTimerA();
//			checkAndResponeMyNeighborsTableToOneRobot();
//			delayTimerB(g_ui8RandomNumber, false);
//			break;
//
//		// VOTE_ORIGIN state
//		case ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN:
//			turnOnLED(LED_RED);
//			reloadDelayTimerA();
//			updateOrRejectNetworkOrigin(RF24_RX_buffer);
//			turnOffLED(LED_RED);
//			break;
//
//		// ROTATE_NETWORK state
//		case ROBOT_REQUEST_ROTATE_NETWORK:
//			getHopOriginTableAndRotate(RF24_RX_buffer);
//			break;
//
//		// REDUCE_ERROR state
//		case ROBOT_REQUEST_MY_VECTOR:
//			tryToResponeNeighborVector();
//			delayTimerB(g_ui8RandomNumber, false);
//			break;

//		case ROBOT_REQUEST_VECTOR_AND_FLAG:
//			tryToResponeVectorAndFlag();
//			delayTimerB(g_ui8RandomNumber, false);
//			break;
//
//		case ROBOT_REQUEST_VECTOR:
//			tryToResponseVector();
//			delayTimerB(g_ui8RandomNumber, false);
//			break;
//
//		// LOCOMOTION state
//		case ROBOT_REQUEST_TO_RUN:
//			clearRequestNeighbor(RF24_RX_buffer);
//			reloadDelayTimerB();
//			break;
//
//		case ROBOT_REQUEST_UPDATE_VECTOR:
//			updateNeighborVectorInLocsTableByRequest(RF24_RX_buffer);
//			break;
//
//		// T shape
//		case ROBOT_ALLOW_MOVE_TO_T_SHAPE:
//			if (construct4BytesToUint32(&RF24_RX_buffer[1]) == g_ui32RobotID)
//				g_bIsAllowToMove = true;
//			break;
//
//		case ROBOT_REPONSE_MOVE_COMPLETED:
//			g_bIsRobotResponse = true;
//			break;

//		// DeBug command
//		case PC_SEND_ROTATE_CORRECTION_ANGLE:
//			if(g_bIsCounterClockwiseOriented)
//				rotateClockwiseWithAngle(0 - g_fRobotOrientedAngle);
//			else
//				rotateClockwiseWithAngle(g_fRobotOrientedAngle);
//			g_fRobotOrientedAngle = 0;
//			break;
//
//		case PC_SEND_ROTATE_CORRECTION_ANGLE_DIFF:
//			rotateClockwiseWithAngle(g_fRobotOrientedAngle);
//			g_fRobotOrientedAngle = 0;
//			break;
//
//		case PC_SEND_ROTATE_CORRECTION_ANGLE_SAME:
//			rotateClockwiseWithAngle(0 - g_fRobotOrientedAngle);
//			g_fRobotOrientedAngle = 0;
//			break;
//
//		case PC_SEND_READ_CORRECTION_ANGLE:
//			responseCorrectionAngleAndOriented();
//			break;
//
//		case PC_SEND_SET_ROBOT_STATE:
//			g_eProcessState = (ProcessState_t)(RF24_RX_buffer[1]);
//			break;
//
//		case PC_SEND_ROTATE_CLOCKWISE:
//			rotateClockwiseTest(RF24_RX_buffer);
//			break;
//
//		case PC_SEND_ROTATE_CLOCKWISE_ANGLE:
//			rotateClockwiseAngleTest(RF24_RX_buffer);
//			break;
//
//		case PC_SEND_FORWARD_PERIOD:
//			forwardPeriodTest(RF24_RX_buffer);
//			break;
//
//		case PC_SEND_FORWARD_DISTANCE:
//			forwardDistanceTest(RF24_RX_buffer);
//			break;
//
//		case PC_SEND_LOCAL_LOOP_STOP:
//			g_ui32LocalLoopStop = construct4BytesToUint32(&RF24_RX_buffer[1]);
//			break;
//
//		case PC_SEND_SET_STEPSIZE:
//			g_fStepSize = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
//			break;
//
//		case PC_SEND_SET_STOP_CONDITION_ONE:
//			g_fStopCondition = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
//			break;
//
//		case PC_SEND_SET_STOP_CONDITION_TWO:
//			g_fStopCondition2 = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
//			break;

//		case PC_SEND_MEASURE_DISTANCE:
//
//			turnOffLED(LED_ALL);
//
//			g_eProcessState = MEASURE_DISTANCE;
//
//			for (g_ui8NeighborsCounter = 0;
//					g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH;
//					g_ui8NeighborsCounter++)
//			{
//				NeighborsTable[g_ui8NeighborsCounter].ID = 0;
//				NeighborsTable[g_ui8NeighborsCounter].distance = 0;
//				OneHopNeighborsTable[g_ui8NeighborsCounter].firstHopID = 0;
//			}
//
//			Tri_clearLocs(locs, &g_ui8LocsCounter);
//
//			g_ui8NeighborsCounter = 0;
//
//			break;
//
//		case PC_SEND_READ_VECTOR:
//			sendVectorToControlBoard();
//			break;
//
//		case PC_SEND_READ_NEIGHBORS_TABLE:
//			sendNeighborsTableToControlBoard();
//			break;
//
//		case PC_SEND_READ_LOCS_TABLE:
//			sendLocationsTableToControlBoard();
//
//		case PC_SEND_READ_ONEHOP_TABLE:
//			sendOneHopNeighborsTableToControlBoard();
//			break;

//		case SMART_PHONE_COMMAND:
//			switch (RF24_RX_buffer[1])
//			{
//			case SP_SEND_STOP_TWO_MOTOR:
//				stopMotors();
//				break;
//
//			case SP_SEND_FORWAR:
//				goStraight();
//				break;
//
//			case SP_SEND_SPIN_CLOCKWISE:
//				spinClockwise();
//				break;
//
//			case SP_SEND_SPIN_COUNTERCLOCKWISE:
//				spinCounterclockwise();
//				break;
//
//			case SP_SEND_RESERVED:
//				goBackward();
//				break;
//
//			default:
//				break;
//			}
//			break;
//
//		default:
//			break;
//		}
//		break;

	}
}

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

void decodeAdvanceHostCommand(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	switch (ui8Cmd)
	{
	case HOST_COMMAND_TEST_RF_TRANSMISTER:
		testRfTransmister(pui8MessageData);
		break;

	case HOST_COMMAND_TEST_RF_RECEIVER:
		testRfReceiver(pui8MessageData);
		break;

	case HOST_COMMAND_TOGGLE_ALL_STATUS_LEDS:
		toggleLED(LED_ALL);
		break;

	case HOST_COMMAND_START_SAMPLING_MIC:
		triggerSamplingMicSignalsWithPreDelay(DELAY_SAMPING_MICS_US);
		break;

	case HOST_COMMAND_REQUEST_BATT_VOLT:
		sendBatteryVoltageToHost();
		break;

	case HOST_COMMAND_INDICATE_BATT_VOLT:
		triggerResponseState(ROBOT_RESPONSE_STATE_SAMPLING_BATTERY, 0, 0);
		break;

	case HOST_COMMAND_START_SPEAKER:
#ifdef Robot_TX_DELAY
		triggerSamplingMicSignalsWithPreDelay(0);	// DELAY_SAMPING_MICS_US
		triggerSpeakerWithPreDelay(60 + 1000);	 	// DELAY_BEFORE_START_SPEAKER_US
#endif
#ifdef Robot_TX
		triggerSamplingMicSignalsWithPreDelay(0);	// DELAY_SAMPING_MICS_US
		triggerSpeakerWithPreDelay(60);				// DELAY_BEFORE_START_SPEAKER_US
#endif
#ifdef Robot_RX
		triggerSamplingMicSignalsWithPreDelay(0);
#endif
		break;

	case HOST_COMMAND_CHANGE_MOTORS_SPEED:
		modifyMotorsConfiguration(pui8MessageData);
		break;

	case HOST_COMMAND_STOP_MOTOR_LEFT:
		MotorLeft_stop();
		break;

	case HOST_COMMAND_STOP_MOTOR_RIGHT:
		MotorRight_stop();
		break;

	case HOST_COMMAND_DATA_ADC0_TO_HOST:
		transmitADCResultsToHost((uint8_t*)getMicrophone0BufferPointer());
		break;

	case HOST_COMMAND_DATA_ADC1_TO_HOST:
		transmitADCResultsToHost((uint8_t*)getMicrophone1BufferPointer());
		break;

	case HOST_COMMAND_EEPROM_DATA_READ:
		transmitRequestDataInEeprom(pui8MessageData);
		break;

	case HOST_COMMAND_EEPROM_DATA_WRITE:
		synchronousEepromData(pui8MessageData);
		break;

	case HOST_COMMAND_EEPROM_DATA_READ_BULK:
		transmitRequestBulkDataInEeprom(pui8MessageData);
		break;

	case HOST_COMMAND_EEPROM_DATA_WRITE_BULK:
		writeBulkToEeprom(pui8MessageData);
		break;

	case HOST_COMMAND_CONFIG_PID_CONTROLLER:
		testPIDController(pui8MessageData);
		break;

	case HOST_COMMAND_CALIBRATE_TDOA_TX:
		triggerResponseState(ROBOT_RESPONSE_STATE_CALIBRATE_TRIGGER_SPEAKER, pui8MessageData, ui32DataSize);
		break;

	default:
		decodeBasicHostCommand(ui8Cmd);
		break;
	}
}

void decodeRobotRequestMessage(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	switch (ui8Cmd)
	{
	case ROBOT_REQUEST_CALIBRATE_SAMPLING_MICS:
		triggerResponseState(ROBOT_RESPONSE_STATE_CALIBRATE_SAMPLING_MICS, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_SAMPLING_MICS:
		triggerResponseState(ROBOT_RESPONSE_STATE_SAMPLING_MICS, pui8MessageData, ui32DataSize);
		break;

	default:
		// Invalid Request
		break;
	}
}

void decodeRobotResponseMessage(uint8_t ui8Cmd, uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	switch (ui8Cmd)
	{
	case ROBOT_RESPONSE_TDOA_RESULT:
		//TODO: get result in pui8MessageData
		break;

	default:
		// Invalid Response
		break;
	}
}

//------------------------ Host side ------------------------
bool sendMessageToHost(e_MessageType eMessType, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	bool bReturn;

	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32DataSize;
	uint8_t* puiMessageBuffer = malloc(ui32TotalSize);
	if(puiMessageBuffer == 0)
		return false;

	constructMessage(puiMessageBuffer, eMessType, ui8Command, pui8MessageData, ui32DataSize);

	bReturn = sendDataToHost(puiMessageBuffer, ui32TotalSize);

	free(puiMessageBuffer);

	return bReturn;
}

bool sendDataToHost(uint8_t* pui8Data, uint32_t ui32DataLength)
{
	if (Network_sendMessage(RF_CONTOLBOARD_ADDR, pui8Data, ui32DataLength, true))
		return true;
	return false;
}

//------------------------ Robot side ------------------------
void broadcastToLocalNeighbors(uint8_t ui8Command, uint8_t* pui8MessageData, uint8_t ui32DataSize)
{
	broatcastMessageToNeighbor(NETWORK_BROADCAST_ADDRESS, ui8Command, pui8MessageData, ui32DataSize);
}

void broatcastMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32DataSize;
	uint8_t* puiMessageBuffer = malloc(ui32TotalSize);
	if(puiMessageBuffer == 0)
		return;

	constructMessage(puiMessageBuffer, MESSAGE_TYPE_ROBOT_REQUEST, ui8Command, pui8MessageData, ui32DataSize);

	broadcastDataToNeighbor(ui32NeighborId, puiMessageBuffer, ui32TotalSize);

	free(puiMessageBuffer);
}

void broadcastDataToNeighbor(uint32_t ui32NeighborId, uint8_t* pui8Data, uint32_t ui32DataSize)
{
	Network_sendMessage(ui32NeighborId, pui8Data, ui32DataSize, false);
}

bool sendMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	bool bReturn;

	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32DataSize;
	uint8_t* puiMessageBuffer = malloc(ui32TotalSize);
	if(puiMessageBuffer == 0)
		return false;

	constructMessage(puiMessageBuffer, MESSAGE_TYPE_ROBOT_REQUEST, ui8Command, pui8MessageData, ui32DataSize);

	bReturn = sendDataToNeighbor(ui32NeighborId, puiMessageBuffer, ui32TotalSize);

	free(puiMessageBuffer);

	return bReturn;
}

bool sendDataToNeighbor(uint32_t ui32NeighborId, uint8_t* pui8Data, uint32_t ui32DataSize)
{
	if (Network_sendMessage(ui32NeighborId, pui8Data, ui32DataSize, true))
		return true;
	return false;
}

void constructMessage(uint8_t* puiMessageBuffer, e_MessageType eMessType,
		uint8_t ui8Command, uint8_t* pui8Data, uint32_t ui32DataSize)
{
	int32_t i;

	// Construct header
	puiMessageBuffer[MESSAGE_TYPE_IDX] = eMessType;
	puiMessageBuffer[MESSAGE_COMMAND_IDX] = ui8Command;

	// Fill data
	for (i = 0; i < ui32DataSize; i++)
	{
		puiMessageBuffer[i + MESSAGE_DATA_START_IDX] = pui8Data[i];
	}
}

