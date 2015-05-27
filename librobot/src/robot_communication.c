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
#include "libcustom/inc/custom_ir.h"

#include "libalgorithm/inc/TDOA.h"

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"

//------------------------ Message Decoder ------------------------
void RobotResponseIntHandler(void)
{
	MCU_RF_DisableInterrupt();

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
			StateOne_MeasureDistance_SamplingMicsHandler(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_TRANSMIT_NEIGHBORS_TABLE:
			StateTwo_ExchangeTable_TransmitNeighborsTableHandler(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_ROTATE_COORDINATES:
			StateFour_RotateCoordinates_RotateCoordinatesHandler(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_READ_LOCATIONS_TABLE:
			StateFour_RotateCoordinates_ReadLocationsTableHandler(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_READ_NEIGHBOR_VECTOR:
			StateFive_AverageVector_ReadNeighborVectorHandler(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_READ_SELF_VECTOR_AND_FLAG:
			StateSix_CorrectLocations_ReadSelfVectorAndFlagHanlder(pui8RequestData);
			break;

		case ROBOT_RESPONSE_STATE_LOCALIZATION:
			updateLocationResquestHanlder(pui8RequestData);
			break;

		default:	// ROBOT_RESPONSE_STATE_NONE
			break;
	}
	setRobotResponseState(ROBOT_RESPONSE_STATE_NONE);

	free(pui8RequestData);

	MCU_RF_EnableInterrupt();
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
//
//		// DeBug command
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
//
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
//			}
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

	case HOST_COMMAND_DATA_ADC0_TO_HOST:
		transmitADCResultsToHost((uint8_t*)getMicrophone0BufferPointer());
		break;

	case HOST_COMMAND_DATA_ADC1_TO_HOST:
		transmitADCResultsToHost((uint8_t*)getMicrophone1BufferPointer());
		break;

	case HOST_COMMAND_REQUEST_BATT_VOLT:
		sendBatteryVoltageToHost();
		break;

	case HOST_COMMAND_START_SPEAKER:
		triggerSamplingMicSignalsWithPreDelay(0);
		triggerSpeakerWithPreDelay(DELAY_BEFORE_START_SPEAKER_US);
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

	case HOST_COMMAND_INDICATE_BATT_VOLT:
		triggerResponseState(ROBOT_RESPONSE_STATE_SAMPLING_BATTERY, 0, 0);
		break;

	case HOST_COMMAND_CALIBRATE_TDOA_TX:
		triggerResponseState(ROBOT_RESPONSE_STATE_CALIBRATE_TRIGGER_SPEAKER, pui8MessageData, ui32DataSize);
		break;

	case HOST_COMMAND_MOVE_WITH_PERIOD:
		robotMoveCommandWithPeriod(pui8MessageData);
		break;

	case HOST_COMMAND_ROTATE_WITH_PERIOD:
		robotRotateCommandWithPeriod(pui8MessageData);
		break;

	case HOST_COMMAND_TOGGLE_IR_LED:
//		toggleIRLED(); // Not use yet
		break;

	case HOST_COMMAND_REQUEST_PROXIMITY_RAW:
//		sendIrProximityValueToHost(); // Not use yet
		break;

	case HOST_COMMAND_READ_ROBOT_IDENTITY:
		transmitRobotIdentityToHost();
		break;

	case HOST_COMMAND_READ_NEIGHBORS_TABLE:
		sendNeighborsTableToHost();
		break;

	case HOST_COMMAND_READ_ONEHOP_NEIGHBORS_TABLE:
		sendOneHopNeighborsTableToHost();
		break;

	case HOST_COMMAND_READ_LOCATIONS_TABLE:
		sendRobotLocationsTableToHost();
		break;

	case HOST_COMMAND_SELF_CORRECT_LOCATIONS_TABLE:
		selfCorrectLocationsTable();
		break;

	case HOST_COMMAND_SELF_CORRECT_LOCATIONS_TABLE_EXCEPT_ROTATION_HOP:
		selfCorrectLocationsTableExceptRotationHopID();
		break;

	case HOST_COMMAND_GOTO_STATE:
		setRobotState((e_RobotState)pui8MessageData[0]);
		break;

	case HOST_COMMAND_MOVE_WITH_DISTANCE:
		robotMoveCommandWithDistance(pui8MessageData);
		break;

	case HOST_COMMAND_ROTATE_WITH_ANGLE:
		robotRotateCommandWithAngle(pui8MessageData);
		break;

	case HOST_COMMAND_CONFIG_STEP_ROTATE_CONTROLLER:
		testStepRotateController(pui8MessageData);
		break;

	case HOST_COMMAND_CONFIG_STEP_FORWARD_IN_PERIOD_CONTROLLER:
		testStepForwardInPeriodController(pui8MessageData);
		break;

	case HOST_COMMAND_CONFIG_STEP_FORWARD_IN_ROTATE_CONTOLLER:
		testStepForwardInRotateController(pui8MessageData);
		break;

	case HOST_COMMAND_UPDATE_GRADIENT_MAP:
		updateGradientMap(pui8MessageData);
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

	case ROBOT_REQUEST_NEIGHBORS_TABLE:
		triggerResponseState(ROBOT_RESPONSE_STATE_TRANSMIT_NEIGHBORS_TABLE, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_VOTE_THE_ORIGIN:
		StateThree_VoteTheOrigin_VoteTheOriginHandler(pui8MessageData);
		break;

	case ROBOT_REQUEST_ROTATE_COORDINATES:
		triggerResponseState(ROBOT_RESPONSE_STATE_ROTATE_COORDINATES, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_READ_LOCATIONS_TABLE:
		triggerResponseState(ROBOT_RESPONSE_STATE_READ_LOCATIONS_TABLE, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_NEIGHBOR_VECTOR:
		triggerResponseState(ROBOT_RESPONSE_STATE_READ_NEIGHBOR_VECTOR, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_SELF_VECTOR_AND_FLAG:
		triggerResponseState(ROBOT_RESPONSE_STATE_READ_SELF_VECTOR_AND_FLAG, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_LOCOLIZATION:
		triggerResponseState(ROBOT_RESPONSE_STATE_LOCALIZATION, pui8MessageData, ui32DataSize);
		break;

	case ROBOT_REQUEST_UPDATE_NEIGHBOR_VECTOR:
		updateNeighborLocationRequestHandler(pui8MessageData);
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
	case ROBOT_RESPONSE_DISTANCE_RESULT:
		StateOne_MeasureDistance_UpdateNeighborsTableHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_SAMPLING_COLLISION:
		handleNeighborResponseSamplingCollision();
		break;

	case ROBOT_RESPONSE_NEIGHBORS_TABLE:
		StateTwo_ExchangeTable_UpdateOneHopNeighborsTableHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_COORDINATES_ROTATED:
		StateFour_RotateCoordinates_UpdateRotationFlagTableHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_LOCATIONS_TABLE:
		StateFour_RotateCoordinates_ReceivedLocationsTableHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_NEIGHBOR_VECTOR:
		StateFive_AverageVector_ReceivedSelfVectorHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_NOT_FOUND_NEIGHBOR_VECTOR:
		StateFive_AverageVector_NotFoundSelfVectorHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG:
		StateSix_CorrectLocations_ReceivedSelfVectorAndFlagHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_PLEASE_WAIT:
		StateSix_CorrectLocations_PleaseWaitHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_SELF_VECTOR_AND_FLAG_UNACTIVE:
		StateSix_CorrectLocations_UnActiveHandler(pui8MessageData, ui32DataSize);
		break;

	case ROBOT_RESPONSE_DISTANCE_RESULT_AND_VECTOR:
		updateLocationResponseHanlder(pui8MessageData, ui32DataSize);
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
	uint8_t* pui8MessageBuffer = malloc(sizeof(*pui8MessageBuffer) * ui32TotalSize);
	if(pui8MessageBuffer == 0)
		return false;

	constructMessage(pui8MessageBuffer, eMessType, ui8Command, pui8MessageData, ui32DataSize);

	bReturn = sendDataToHost(pui8MessageBuffer, ui32TotalSize);

	free(pui8MessageBuffer);

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
	broadcastMessageToNeighbor(NETWORK_BROADCAST_ADDRESS, ui8Command, pui8MessageData, ui32DataSize);
}

void broadcastMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32DataSize;
	uint8_t* pui8MessageBuffer = malloc(sizeof(*pui8MessageBuffer) * ui32TotalSize);
	if(pui8MessageBuffer == 0)
		return;

	constructMessage(pui8MessageBuffer, MESSAGE_TYPE_ROBOT_REQUEST, ui8Command, pui8MessageData, ui32DataSize);

	broadcastDataToNeighbor(ui32NeighborId, pui8MessageBuffer, ui32TotalSize);

	free(pui8MessageBuffer);
}

void broadcastDataToNeighbor(uint32_t ui32NeighborId, uint8_t* pui8Data, uint32_t ui32DataSize)
{
	Network_sendMessage(ui32NeighborId, pui8Data, ui32DataSize, false);
}

bool responseMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	return transmitMessageToNeighbor(MESSAGE_TYPE_ROBOT_RESPONSE, ui32NeighborId, ui8Command, pui8MessageData, ui32DataSize);
}

bool sendMessageToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	return transmitMessageToNeighbor(MESSAGE_TYPE_ROBOT_REQUEST, ui32NeighborId, ui8Command, pui8MessageData, ui32DataSize);
}

bool reponseCommandToNeighbor(uint32_t ui32NeighborId, uint8_t ui8Command)
{
	return transmitMessageToNeighbor(MESSAGE_TYPE_ROBOT_RESPONSE, ui32NeighborId, ui8Command, 0, 0);
}

bool transmitMessageToNeighbor(e_MessageType eMessType, uint32_t ui32NeighborId, uint8_t ui8Command,
		uint8_t* pui8MessageData, uint32_t ui32DataSize)
{
	bool bReturn;

	uint32_t ui32TotalSize = MESSAGE_HEADER_LENGTH + ui32DataSize;
	uint8_t* pui8MessageBuffer = malloc(sizeof(*pui8MessageBuffer) * ui32TotalSize);
	if(pui8MessageBuffer == 0)
		return false;

	constructMessage(pui8MessageBuffer, eMessType, ui8Command, pui8MessageData, ui32DataSize);

	bReturn = sendDataToNeighbor(ui32NeighborId, pui8MessageBuffer, ui32TotalSize);

	free(pui8MessageBuffer);

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

