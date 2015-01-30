//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_nvic.h"
//#include "inc/hw_gpio.h"
//#include "inc/hw_adc.h"
//#include "inc/hw_udma.h"
//#include "inc/hw_timer.h"
//#include "inc/hw_ssi.h"
//#include "inc/hw_i2c.h"
//#include "inc/hw_eeprom.h"
//#include "driverlib/debug.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/gpio.h"
//#include "driverlib/adc.h"
//#include "driverlib/udma.h"
//#include "driverlib/timer.h"
//#include "driverlib/ssi.h"
//#include "driverlib/fpu.h"
//#include "driverlib/rom.h"
//#include "driverlib/pwm.h"
//#include "driverlib/eeprom.h"
//
//#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
//#include "libnrf24l01/inc/nRF24L01.h"
//#include "librobot/inc/TDOA.h"
//#include "librobot/inc/Trilateration.h"
//
//extern float32_t g_f32PeakEnvelopeA;
//extern float32_t g_f32MaxEnvelopeA;
//extern float32_t g_f32PeakEnvelopeB;
//extern float32_t g_f32MaxEnvelopeB;
//
//extern CpuState_t g_eCPUState;
//
//extern bool g_bDelayTimerAFlagAssert;
//extern bool g_bDelayTimerBFlagAssert;
//
//extern uint32_t g_ui32RobotID;
//extern vector2_t g_vector;
//extern bool g_bIsValidVector;
//extern bool g_bIsCounterClockwiseOriented;
//extern float g_fRobotOrientedAngle;
//
//extern uint32_t g_ui32RequestRobotID;
//
//extern bool g_bIsNetworkRotated;
//extern bool g_bIsActiveCoordinatesFixing;
//extern bool g_bIsGradientSearchStop;
//extern uint32_t g_ui32LocalLoop;
//uint32_t g_ui32LocalLoopStop; // Debug Only
//
//extern ProcessState_t g_eProcessState;
//extern RobotResponseState_t g_eRobotResponseState;
//extern bool g_bBypassThisState;
//extern uint8_t g_ui8ReBroadcastCounter;
//
//extern robotMeas_t NeighborsTable[];
//extern oneHopMeas_t OneHopNeighborsTable[];
//extern uint8_t g_ui8NeighborsCounter;
//
//extern location_t locs[];
//extern location_t oriLocs[];
//
//extern uint8_t g_ui8LocsCounter;
//
//extern uint32_t g_ui32OriginID;
//extern uint32_t g_ui32RotationHopID;
//extern uint8_t g_ui8Hopth;
//extern uint8_t g_ui8OriginNumberOfNeighbors;
//
//extern uint8_t RF24_RX_buffer[];
//extern uint8_t RF24_TX_buffer[];
//extern uint8_t g_ui8ReTransmitCounter;
//
//extern uint8_t g_ui8RandomNumber;
//extern uint16_t g_pui16ADC0Result[];
//extern uint16_t g_pui16ADC1Result[];
//extern bool g_bIsNewTDOAResults;
//
//extern "C"
//{
//	void RF24_IntHandler();
//	void RobotResponseIntHandler(void);
//	void LowPowerModeIntHandler(void);
//	void DelayTimerAIntHanler(void);
//	void DelayTimerBIntHanler(void);
//}
//
//void RobotProcess();
//void StateOne_MeasureDistance();
//void StateTwo_ExchangeTableAndCalculateLocsTable();
//void StateThree_VoteOrigin();
//void StateFour_RequestRotateNetwork();
//void StateFive_ReduceCoordinatesError();
//void StateSix_Locomotion();
//
//void SwarmStateOne_TShape();
//
//extern float g_f32Intercept;
//extern float g_f32Slope;
//
//float g_fStepSize;
//float g_fStopCondition;
//float g_fStopCondition2;
//
//bool g_bIsRobotResponse; // T shape
//bool g_bIsAllowToMove;	// T shape

#include <stdint.h>
#include <stdbool.h>
#include "libcustom\inc\custom_clock.h"
#include "libcustom\inc\custom_led.h"
#include "libcustom\inc\custom_delay.h"
#include "libcustom\inc\custom_uart_debug.h"

#include "librobot\inc\robot_lpm.h"
#include "librobot\inc\robot_timer_delay.h"
#include "librobot\inc\robot_speaker.h"
#include "librobot\inc\robot_analog.h"
#include "librobot\inc\robot_motor.h"
#include "librobot\inc\robot_eeprom.h"
#include "librobot\inc\robot_communication.h"

#include "interrupt_definition.h"

extern "C"
{
void TI_CC_IRQ_handler();

void RobotResponseIntHandler(void);
}

void decodeMessage(uint8_t* pui8Message, uint32_t ui32MessSize);

void RobotResponseIntHandler(void)
{
}

typedef enum tag_RobotState
{
	ROBOT_STATE_IDLE = 0,
	ROBOT_STATE_MEASURE_DISTANCE = 1,
	ROBOT_STATE_EXCHANGE_TABLE = 2,
	ROBOT_STATE_VOTE_ORIGIN = 3,
	ROBOT_STATE_ROTATE_NETWORK = 4,
	ROBOT_STATE_REDUCE_ERROR = 5,
	ROBOT_STATE_LOCOMOTION = 6,
	ROBOT_STATE_T_SHAPE = 7,
} e_RobotState;

e_RobotState g_eCurrentRobotState = ROBOT_STATE_IDLE;

int main(void)
{
	initSysClock();

	initUartDebug();

	initLeds();
	DEBUG_PRINT("init UART DEBUG: OK\n");

	initLowPowerMode();

	initRobotTimerDelay();
	DEBUG_PRINT("init Robot timer delay: OK\n");

	initPeripheralsForAnalogFunction();
	DEBUG_PRINT("init Peripherals for analog feature: OK\n");

	initSpeaker();
	DEBUG_PRINT("init Speaker: OK\n");

	initMotors();
	DEBUG_PRINT("init Motors: OK\n");

	initEEPROM();
	DEBUG_PRINT("init EEPROM: OK\n");

	initDelay();
	DEBUG_PRINT("init Delay: OK\n");

	initRfModule(true);
	DEBUG_PRINT("init RF module: OK, in rx mode.\n");

	turnOffLED(LED_ALL);

	Network_setSelfAddress(RF_DEFAULT_ROBOT_ID);
	DEBUG_PRINTS("set Self Address to 0x%08x\n", RF_DEFAULT_ROBOT_ID);

//	initEEPROM();
//	initLowPowerMode();
//	initRobotProcess();
//	initMotor();
//	initPeripheralsForAnalogFunction();
//	initSpeaker();
//	initTimerDelay();
//	TDOA_initFilters();

	DEBUG_PRINT("--loop start--\n");

	while (1)
	{
		switch (g_eCurrentRobotState)
		{
//		case MEASURE_DISTANCE:
//			StateOne_MeasureDistance();
//			break;
//
//		case EXCHANGE_TABLE:
//			StateTwo_ExchangeTableAndCalculateLocsTable();
//			break;
//
//		case VOTE_ORIGIN:
//			StateThree_VoteOrigin();
//			break;
//
//		case ROTATE_NETWORK:
//			StateFour_RequestRotateNetwork();
//			break;
//
//		case REDUCE_ERROR:
//			StateFive_ReduceCoordinatesError();
//			break;
//
//		case LOCOMOTION:
//			StateSix_Locomotion();
//			break;
//
//		case T_SHAPE:
//			SwarmStateOne_TShape();
//			break;

		default:// ROBOT_STATE_IDLE state
			delay_ms(750);
			toggleLED(LED_RED);
			break;
		}
	}
}

void decodeMessage(uint8_t* pui8MessageBuffer, uint32_t ui32MessSize)
{
	MessageHeader* pMessageHeader = (MessageHeader*) pui8MessageBuffer;

	if (getCpuMode() != CPU_MODE_RUN)
	{
		if (pMessageHeader->eMessageType != MESSAGE_TYPE_HOST_COMMAND)
			returnSleep();

		if (!decodeBasicHostCommand(pMessageHeader->ui8Cmd))
			returnSleep();
	}
	else
	{
		switch (pMessageHeader->eMessageType)
		{
		case MESSAGE_TYPE_HOST_COMMAND:
			decodeAdvanceHostCommand(pMessageHeader->ui8Cmd, pui8MessageBuffer);
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

void TI_CC_IRQ_handler(void)
{
	uint32_t ui32MessageSize;

	uint8_t* pui8RxBuffer = 0;

	if (TI_CC_IsInterruptPinAsserted())
	{
		TI_CC_ClearIntFlag();

		TI_CC_DisableInterrupt();

		turnOnLED(LED_RED);

		if (Network_receivedMessage(&pui8RxBuffer, &ui32MessageSize))
		{
			decodeMessage(pui8RxBuffer, ui32MessageSize);
		}

		if (pui8RxBuffer != 0)
		{
			delete[] pui8RxBuffer;
			pui8RxBuffer = 0;
		}

		TI_CC_EnableInterrupt();

		turnOffLED(LED_RED);
	}

	if(getCpuMode() != CPU_MODE_RUN)
		returnSleep();
}


//void StateOne_MeasureDistance()
//{
//	uint16_t ui16RandomValue;
//	float32_t f32_inputValue;
//	float32_t f32_outputValue;
//
//	// set this as origin
//	g_ui32OriginID = g_ui32RobotID;
//	g_vector.x = 0;
//	g_vector.y = 0;
//
//	//init variables
//	g_bIsNetworkRotated = false;
//
//	turnOnLED(LED_BLUE);
//
//	g_bBypassThisState = false;
//
//	delayTimerA(DELAY_MEASURE_DISTANCE_STATE*4, false);
//
//	while (!g_bDelayTimerAFlagAssert)
//	{
//		generateRandomByte();
//
//		while (g_ui8RandomNumber == 0)
//			;
//		g_ui8RandomNumber =
//				(g_ui8RandomNumber < 100) ?
//						(g_ui8RandomNumber + 100) : (g_ui8RandomNumber);
//		ui16RandomValue = (g_ui32RobotID << 8) | g_ui8RandomNumber;
//
//		delayTimerB(ui16RandomValue, false);
//
//		while (!g_bDelayTimerBFlagAssert)
//		{
//			g_bIsNewTDOAResults = false;
//
//			if (NeighborsTable[g_ui8NeighborsCounter].ID != 0)
//			{
//				turnOnLED(LED_GREEN);
//
//				reloadDelayTimerA();
//
//				while(!g_bIsNewTDOAResults);
//
//				g_bIsNewTDOAResults = false;
//
//				if (g_f32MaxEnvelopeA < MAX_THRESHOLD
//						|| g_f32MaxEnvelopeB < MAX_THRESHOLD
//						|| g_f32PeakEnvelopeA > MAX_SAMPLE_POSITION
//						|| g_f32PeakEnvelopeB > MAX_SAMPLE_POSITION)
//				{
//					NeighborsTable[g_ui8NeighborsCounter].ID = 0;
//				}
//				else
//				{
//					g_f32PeakEnvelopeA = (g_f32PeakEnvelopeA - g_f32Intercept)
//							/ g_f32Slope;
//					g_f32PeakEnvelopeB = (g_f32PeakEnvelopeB - g_f32Intercept)
//							/ g_f32Slope;
//
//					f32_inputValue =
//							(((g_f32PeakEnvelopeA * g_f32PeakEnvelopeA
//									+ g_f32PeakEnvelopeB
//											* g_f32PeakEnvelopeB) / 2.0)
//									- (DISTANCE_BETWEEN_TWO_MICS_SQR / 4.0))
//									* 65536.0; // * 256^2
//
//					f32_outputValue = vsqrtf(f32_inputValue);
//
//					NeighborsTable[g_ui8NeighborsCounter].distance =
//							(uint16_t) (f32_outputValue + 0.5);
//
//					g_ui8NeighborsCounter++;
//
//					//TODO: search the worth result and replace it by the current result if better
//					g_ui8NeighborsCounter =
//							(g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH) ?
//									(g_ui8NeighborsCounter) :
//									(NEIGHBOR_TABLE_LENGTH);
//				}
//				turnOffLED(LED_GREEN);
//			}
//		}
//
//		if (!g_bBypassThisState)
//		{
//			turnOffLED(LED_BLUE);
//
//			reloadDelayTimerA();
//
//			RF24_TX_buffer[0] = ROBOT_REQUEST_SAMPLING_MIC;
//			RF24_TX_buffer[1] = g_ui32RobotID >> 24;
//			RF24_TX_buffer[2] = g_ui32RobotID >> 16;
//			RF24_TX_buffer[3] = g_ui32RobotID >> 8;
//			RF24_TX_buffer[4] = g_ui32RobotID;
//
//			broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
//			// WARING!!! DO NOT INSERT ANY CODE IN HERE!
//			ROM_SysCtlDelay(DELAY_START_SPEAKER);
//			// WARING!!! DO NOT INSERT ANY CODE IN HERE!
//			startSpeaker();
//
//			RF24_RX_activate();
//
//			g_bBypassThisState = true;
//		}
//	}
//
//	turnOnLED(LED_ALL);
//
//	g_eProcessState = EXCHANGE_TABLE;
//}

//void StateTwo_ExchangeTableAndCalculateLocsTable()
//{
//	uint16_t ui16RandomValue;
//	uint8_t ui8RandomRfChannel;
//	uint8_t neighborsTablePointer;
//
//	neighborsTablePointer = 0;
//
//	Tri_clearLocs(locs, &g_ui8LocsCounter);
//
//	delayTimerA(DELAY_EXCHANGE_TABLE_STATE, false);
//
//	while (!g_bDelayTimerAFlagAssert)
//	{
//		while (neighborsTablePointer < g_ui8NeighborsCounter)
//		{
//			generateRandomByte();
//
//			while (g_ui8RandomNumber == 0)
//				;
//
//			ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
//
//			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)
//
//			while (1)
//			{
//				reloadDelayTimerA();
//
//				generateRandomByte();
//				while (g_ui8RandomNumber == 0)
//					;
//				g_ui8RandomNumber =
//						(g_ui8RandomNumber < 100) ?
//								(g_ui8RandomNumber + 100) :
//								(g_ui8RandomNumber);
//
//				ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);
//
//				delayTimerB(ui16RandomValue, true); // maybe Received Request table command here!
//
//				while (!g_bDelayTimerBFlagAssert)
//					; // this line make sure robot will re delay after send neighbors table to another robot
//
//				// delay timeout
//				RF24_TX_buffer[0] = ROBOT_REQUEST_NEIGHBORS_TABLE;
//				RF24_TX_buffer[1] = g_ui32RobotID >> 24;
//				RF24_TX_buffer[2] = g_ui32RobotID >> 16;
//				RF24_TX_buffer[3] = g_ui32RobotID >> 8;
//				RF24_TX_buffer[4] = g_ui32RobotID;
//				RF24_TX_buffer[5] = ui8RandomRfChannel;
//
//				if (sendMessageToOneNeighbor(
//						NeighborsTable[neighborsTablePointer].ID,
//						RF24_TX_buffer, 6))
//				{
//					turnOffLED(LED_RED);
//
//					RF24_setChannel(ui8RandomRfChannel);
//					RF24_TX_flush();
//					RF24_clearIrqFlag(RF24_IRQ_MASK);
//					RF24_RX_activate();
//
//					getNeighborNeighborsTable();
//
//					neighborsTablePointer++;
//
//					RF24_setChannel(0);
//					RF24_TX_flush();
//					RF24_clearIrqFlag(RF24_IRQ_MASK);
//					RF24_RX_activate();
//
//					turnOnLED(LED_RED);
//					break;
//				}
//				else if (g_ui8ReTransmitCounter == 0)
//				{
//					break;
//				}
//			}
//
//			if (neighborsTablePointer >= g_ui8NeighborsCounter) // exchane table finished
//			{
//				neighborsTablePointer = g_ui8NeighborsCounter;
//
//				if (g_ui8LocsCounter == 0) // try to calculate neighbor's coordination
//				{
//					Tri_addLocation(g_ui32RobotID, 0, 0);
//
//					Tri_findLocs(NeighborsTable, OneHopNeighborsTable);
//				}
//			}
//		}
//	}
//
//	turnOffLED(LED_BLUE);
//
//	g_eProcessState = VOTE_ORIGIN;
//}

//void StateThree_VoteOrigin()
//{
//	if(g_ui32OriginID == g_ui32RobotID) // haven't update
//	{
//		// set This As Origin Node
//		g_ui8OriginNumberOfNeighbors = g_ui8LocsCounter;
//		g_ui8Hopth = 0;
//		g_ui32RotationHopID = g_ui32RobotID;
//	}
//	else // haven't update
//	{
//		if (g_ui8OriginNumberOfNeighbors < g_ui8LocsCounter)
//		{
//			// set This As Origin Node
//			g_ui32OriginID = g_ui32RobotID;
//			g_ui8OriginNumberOfNeighbors = g_ui8LocsCounter;
//			g_ui8Hopth = 0;
//			g_ui32RotationHopID = g_ui32RobotID;
//		}
//	}
//
//	g_ui8ReBroadcastCounter = 0;
//
//	g_bBypassThisState = false;
//
//	delayTimerA(DELAY_ROTATE_NETWORK, false);	// start State Timer
//
//	while (!g_bDelayTimerAFlagAssert)
//	{
//		turnOffLED(LED_GREEN);
//
//		generateRandomByte();
//		while (g_ui8RandomNumber == 0)
//			;
//
//		delayTimerB(DELAY_REBROADCAST, true);
//
//		// delay timeout
//		if (!g_bBypassThisState)
//		{
//			reloadDelayTimerA();
//
//			turnOnLED(LED_GREEN);
//
//			// construct message: <Update Network Origin COMMAND> <thisOriginID> <thisOriginNumberOfNeighbors> <thisHopth>
//			RF24_TX_buffer[0] = ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN;
//			RF24_TX_buffer[1] = g_ui32OriginID >> 24;
//			RF24_TX_buffer[2] = g_ui32OriginID >> 16;
//			RF24_TX_buffer[3] = g_ui32OriginID >> 8;
//			RF24_TX_buffer[4] = g_ui32OriginID;
//			RF24_TX_buffer[5] = g_ui8OriginNumberOfNeighbors;
//			RF24_TX_buffer[6] = g_ui8Hopth;
//
//			broadcastLocalNeighbor(RF24_TX_buffer, 7);
//			RF24_RX_activate();
//
//			g_ui8ReBroadcastCounter++;
//
//			if (g_ui8ReBroadcastCounter >= REBROADCAST_TIMES)
//				g_bBypassThisState = true;
//		}
//	}
//
//	g_eProcessState = ROTATE_NETWORK;
//}

//void StateFour_RequestRotateNetwork()
//{
//	uint8_t ui8RandomRfChannel;
//	uint32_t tableSizeInByte;
//	uint32_t neighborID;
//	int8_t i;
//	int32_t tempValue;
//
//	if (g_ui32OriginID & 0x01)
//		turnOnLED(LED_GREEN);
//	else
//		turnOffLED(LED_GREEN);
//
//	if (g_ui32OriginID & 0x02)
//		turnOnLED(LED_BLUE);
//	else
//		turnOffLED(LED_BLUE);
//
//	if (g_ui32OriginID & 0x04)
//		turnOnLED(LED_RED);
//	else
//		turnOffLED(LED_RED);
//
//	if (g_ui32OriginID == g_ui32RobotID)
//	{
//		// I'am Original
//		g_bIsNetworkRotated = true;
//
//		g_ui32RotationHopID = g_ui32RobotID;
//
//		g_vector.x = 0;
//		g_vector.y = 0;
//	}
//
//	// waiting request and rotate
//	while(!g_bIsNetworkRotated);
//
//	// Debug Only
//	if (g_ui32RobotID == g_ui32OriginID)
//	{
//		g_ui8ReTransmitCounter = 0; // disable software trigger retransmit
//
//		tableSizeInByte = sizeof(location_t) * g_ui8LocsCounter;
//
//		i = 0;
//		while (i < g_ui8LocsCounter)
//		{
//			generateRandomByte();
//			while (g_ui8RandomNumber == 0)
//				;
//			g_ui8RandomNumber = (g_ui8RandomNumber < 100) ? (g_ui8RandomNumber + 100) : (g_ui8RandomNumber);
//
//			ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
//
//			delayTimerB(g_ui8RandomNumber + 1000, true);
//
//			neighborID = locs[i].ID;
//
//			if (neighborID == g_ui32RotationHopID || neighborID == g_ui32RobotID)
//			{
//				i++;
//				continue;
//			}
//
//			// send rotate network command and locs size
//			RF24_TX_buffer[0] = ROBOT_REQUEST_ROTATE_NETWORK;
//
//			RF24_TX_buffer[1] = g_ui32RobotID >> 24;
//			RF24_TX_buffer[2] = g_ui32RobotID >> 16;
//			RF24_TX_buffer[3] = g_ui32RobotID >> 8;
//			RF24_TX_buffer[4] = g_ui32RobotID;
//
//			RF24_TX_buffer[5] = tableSizeInByte >> 24;
//			RF24_TX_buffer[6] = tableSizeInByte >> 16;
//			RF24_TX_buffer[7] = tableSizeInByte >> 8;
//			RF24_TX_buffer[8] = tableSizeInByte;
//
//			tempValue = (int8_t)(g_vector.x * 65536.0 + 0.5);
//			RF24_TX_buffer[9] = tempValue >> 24;
//			RF24_TX_buffer[10] = tempValue >> 16;
//			RF24_TX_buffer[11] = tempValue >> 8;
//			RF24_TX_buffer[12] = tempValue;
//
//			tempValue = (int8_t)(g_vector.y * 65536.0 + 0.5);
//			RF24_TX_buffer[13] = tempValue >> 24;
//			RF24_TX_buffer[14] = tempValue >> 16;
//			RF24_TX_buffer[15] = tempValue >> 8;
//			RF24_TX_buffer[16] = tempValue;
//
//			RF24_TX_buffer[17] = ui8RandomRfChannel;
//
//			if (sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, 18))
//			{
//				turnOffLED(LED_RED);
//
//				RF24_setChannel(ui8RandomRfChannel);
//				RF24_TX_flush();
//				RF24_clearIrqFlag(RF24_IRQ_MASK);
//				RF24_RX_activate();
//
//				SysCtlDelay(3000); // delay 150us
//				sendMessageToOneNeighbor(neighborID, (uint8_t*)locs, tableSizeInByte);
//				i++;
//
//				RF24_setChannel(0);
//				RF24_TX_flush();
//				RF24_clearIrqFlag(RF24_IRQ_MASK);
//				RF24_RX_activate();
//
//				turnOnLED(LED_RED);
//			}
//		}
//	}
//	// Coordinates rotated!!!
//
//	// offset locs table to real vector
//	for(i = 0; i < g_ui8LocsCounter; i++)
//	{
//		locs[i].vector.x += g_vector.x;
//		locs[i].vector.y += g_vector.y;
//	}
//
////	g_eProcessState = IDLE;
//
//	turnOffLED(LED_ALL);
//	g_eProcessState = REDUCE_ERROR;
//}

//void StateFive_ReduceCoordinatesError()
//{
//	int8_t i;
//	uint8_t ui8RandomRfChannel;
//	uint16_t ui16RandomValue;
//	bool isSuccess;
//
//	g_fStepSize = 0.2f;
//	g_fStopCondition = 0.3f;
//	g_fStopCondition2 = 1.2f;
//
////	float fRandomeStepSize;					// random 2.0f -> 4.0f
////	float const fStepSize = 0.2f; 			//
////	float const fStopCondition = 0.003f; 	// unit m
////	float const fStopCondition2 = 0.12f; 	// unit m
//
//	uint8_t ui8VectorCounter;
//	vector2_t vectGradienNew;
//	vector2_t vectGradienOld;
//	vector2_t vectEstimatePosNew;
//	vector2_t vectEstimatePosOld;
//
//	g_ui32LocalLoopStop = 1; // DEBUG only
//
//	if (g_ui32OriginID == g_ui32RobotID)
//	{
//		g_bIsActiveCoordinatesFixing = false;
//		g_bIsGradientSearchStop = true;
//		g_vector.x = 0;
//		g_vector.y = 0;
//
////		while(1)
////		{
////			rfDelayLoop(DELAY_CYCLES_5MS * 500); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
////			toggleLED(LED_ALL);
////
////			for(i = 0; i < g_ui8NeighborsCounter; i++)
////			{
////				if (NeighborsTable[i].ID == g_ui32RobotID)
////					continue;
////
////				g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)
////
////				isSuccess = false;
////
////				while(1)
////				{
////					generateRandomByte();
////					while (g_ui8RandomNumber == 0);
////
////					ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
////
////					g_ui8RandomNumber =
////							(g_ui8RandomNumber < 100) ?
////									(g_ui8RandomNumber + 100) :
////									(g_ui8RandomNumber);
////
////					ui16RandomValue = g_ui8RandomNumber * 5;
////
////					delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
////
////					RF24_TX_buffer[0] = ROBOT_REQUEST_VECTOR;
////					parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
////					RF24_TX_buffer[5] = ui8RandomRfChannel;
////
////					// send request neighbor send there g_vector coordinates: <x>, <y>
////					if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 10))
////					{
////						turnOffLED(LED_RED);
////
////						RF24_setChannel(ui8RandomRfChannel);
////						RF24_TX_flush();
////						RF24_clearIrqFlag(RF24_IRQ_MASK);
////						RF24_RX_activate();
////
////						isSuccess = getNeighborVector(NeighborsTable[i].ID);
////
////						RF24_setChannel(0);
////						RF24_TX_flush();
////						RF24_clearIrqFlag(RF24_IRQ_MASK);
////						RF24_RX_activate();
////
////						turnOnLED(LED_RED);
////
////						if (isSuccess)
////							break;
////					}
////					else if (g_ui8ReTransmitCounter == 0)
////						break;
////				}
////			}
////		}
//	}
//	else
//	{
//		g_bIsActiveCoordinatesFixing = true;
//
//		// Calculate average position
//		g_ui32LocalLoop = 0;
//
//		ui8VectorCounter = 1;
//
//		for(i = 0; i < g_ui8NeighborsCounter; i++)
//		{
//			if (NeighborsTable[i].ID == g_ui32RobotID)
//				continue;
//
//			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)
//
//			isSuccess = false;
//
//			while (1) // wait for neighbor g_bIsNetworkRotated = true and get my vector
//			{
//				generateRandomByte();
//				while (g_ui8RandomNumber == 0);
//
//				ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
//
//				g_ui8RandomNumber =
//						(g_ui8RandomNumber < 100) ?
//								(g_ui8RandomNumber + 100) :
//								(g_ui8RandomNumber);
//
//				ui16RandomValue = g_ui8RandomNumber * 10;
//
//				delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
//
//				while (!g_bDelayTimerBFlagAssert)
//					; // this line make sure robot will re delay after response to another robot
//
//				// delay timeout
//				RF24_TX_buffer[0] = ROBOT_REQUEST_MY_VECTOR;
//				parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
//				RF24_TX_buffer[5] = ui8RandomRfChannel;
//
//				if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 6))
//				{
//					turnOffLED(LED_RED);
//
//					RF24_setChannel(ui8RandomRfChannel);
//					RF24_TX_flush();
//					RF24_clearIrqFlag(RF24_IRQ_MASK);
//					RF24_RX_activate();
//
//					isSuccess = getMyVector(&ui8VectorCounter);
//
//					RF24_setChannel(0);
//					RF24_TX_flush();
//					RF24_clearIrqFlag(RF24_IRQ_MASK);
//					RF24_RX_activate();
//
//					turnOnLED(LED_RED);
//
//					if (isSuccess)
//						break;
//				}
//				else if (g_ui8ReTransmitCounter == 0)
//					break;
//			}
//		}
//
//		g_vector.x /= ui8VectorCounter;
//		g_vector.y /= ui8VectorCounter;
//
//		synchronousLocsTableAndMyVector();
//
//		g_ui32LocalLoop = 1;
//
//		turnOnLED(LED_ALL); // OK
//
//		// Update locs table
//		updateLocsByOtherRobotCurrentPosition(true);
//
//		turnOffLED(LED_GREEN); // OK
//
//		vectEstimatePosNew.x = g_vector.x;
//		vectEstimatePosNew.y = g_vector.y;
//
//		vectGradienNew.x = 0;
//		vectGradienNew.y = 0;
//
//		g_bIsGradientSearchStop = false;
//
//		g_ui32LocalLoop++;
//
//		while(!g_bIsGradientSearchStop)
//		{
//			// Algorithm 1
//			while(!g_bIsGradientSearchStop)
//			{
//				//DEBUG only
//				while(g_ui32LocalLoop > g_ui32LocalLoopStop);
//
//				turnOffLED(LED_BLUE);
//
//				updateGradient(&vectGradienNew, false);
//
//				updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, g_fStepSize);
//				synchronousLocsTableAndMyVector();
//
//				g_ui32LocalLoop++;
//
//				g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition);
//
//				updateLocsByOtherRobotCurrentPosition(false);
//
//				if (g_ui32LocalLoop & 0x01)
//					turnOnLED(LED_GREEN);
//				else
//					turnOffLED(LED_GREEN);
//			}
//
//			turnOnLED(LED_BLUE);
////
////			//TODO: uncomment: Escape Local Minima
////			updateGradient(&vectGradienNew, true);
////
////			fRandomeStepSize = generateRandomRange(2.0f, 4.0f);
////
////			updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, fRandomeStepSize);
////			synchronousLocsTableAndMyVector();
////
////			g_ui32LocalLoop++;
////
////			g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition2);
////
////			updateLocsByOtherRobotCurrentPosition(false);
//		}
//
////		g_ui32LocalLoop = 0;
////		g_vector.x = vectEstimatePosOld.x;
////		g_vector.y = vectEstimatePosOld.y;
////		updateLocsByOtherRobotCurrentPosition(true);
//
//		g_bIsActiveCoordinatesFixing = false;
//
//		turnOffLED(LED_BLUE);
//	}
//
//	g_bIsValidVector = true;
//
//	//g_eProcessState = LOCOMOTION;
//	g_eProcessState = IDLE;
//}

//void StateSix_Locomotion()
//{
//	vector2_t vectZero;
//	vector2_t vectOne;
//	vector2_t vectDiff;
//
////	float fAngleOffer = 0.2094395102; // ~12 degree
//	float fRotateAngle = MATH_PI_DIV_2; // ~90 degree
////	float fThetaOne;
////	float fThetaTwo;
//
//	float alphaP;
//	float alphaQ;
//
//	uint16_t ui16RandomValue;
//
//	g_bIsNewTDOAResults = false;
//
//	turnOffLED(LED_ALL);
//
//	// save my old vector for initialize position V0
//	vectZero.x = g_vector.x;
//	vectZero.y = g_vector.y;
//
//	// delay random
////	generateRandomByte();
////	while (g_ui8RandomNumber == 0)
////		;
////	g_ui8RandomNumber =
////			(g_ui8RandomNumber < 100) ?
////					(g_ui8RandomNumber + 100) :
////					(g_ui8RandomNumber);
//
////	ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);
////
////	ui16RandomValue = g_ui8RandomNumber * 10;
//
//	ui16RandomValue = (g_ui32RobotID & 0xFF) * DELAY_LOCOMOTION_PERIOD;
//
//	delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_TO_RUN command here!
//										// if received neighbor run command then redelay and wait
//
//	// delay timeout
//	g_bIsValidVector = false;
//
//	runForwardAndCalculatteNewPosition(7.0); // g_vector may be modified to new position
//	if (g_ui8NeighborsCounter < 3)
//	{
//		//TODO: reserved
//		runForwardWithDistance(-7.0);
//
//		// WARNING!!! This vector may be not correct
//		Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);
//		g_bIsValidVector = true;
//		g_eProcessState = IDLE;
//		return;
//	}
//
//	vectOne.x = g_vector.x;
//	vectOne.y = g_vector.y;
//
//	rotateClockwiseWithAngle(fRotateAngle);
//
//	runForwardAndCalculatteNewPosition(6.0); // g_vector may be modified to new position
//	if (g_ui8NeighborsCounter < 3)
//	{
//		//TODO: reserved
//		runForwardWithDistance(-6.0);
//
//		// WARNING!!! This vector may be not correct
//		Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);
//		g_bIsValidVector = true;
//		g_eProcessState = IDLE;
//		return; // Not enough neighbors
//	}
//
//	g_bIsValidVector = true;
//
//	notifyNewVectorToNeigbors();
//
////	vectDiff.x = vectOne.x - vectZero.x;
////	vectDiff.y = vectOne.y - vectZero.y;
////	fThetaOne = calculateRobotAngleWithXAxis(vectDiff);
////
////	vectDiff.x = g_vector.x - vectOne.x;
////	vectDiff.y = g_vector.y - vectOne.y;
////	fThetaTwo = calculateRobotAngleWithXAxis(vectDiff);
//
//	vectDiff.x = g_vector.x - vectOne.x;
//	vectDiff.y = g_vector.y - vectOne.y;
//	g_fRobotOrientedAngle = calculateRobotAngleWithXAxis(vectDiff);
//
////	if (fRotateAngle < 0)
////		fRotateAngle = fThetaOne - fRotateAngle;
////	else
////		fRotateAngle += fThetaOne;
////
////	while(fRotateAngle > MATH_PI_MUL_2)
////		fRotateAngle -= MATH_PI_MUL_2;
//
//	// one - zero = P
//	vectDiff.x = vectOne.x - vectZero.x;
//	vectDiff.y = vectOne.y - vectZero.y;
//	alphaP = calculateRobotAngleWithXAxis(vectDiff);
//
//	// g_vector - zero = Q
//	vectDiff.x = g_vector.x - vectZero.x;
//	vectDiff.y = g_vector.y - vectZero.y;
//	alphaQ = calculateRobotAngleWithXAxis(vectDiff);
//
////	if(fRotateAngle <= (fThetaTwo + fAngleOffer) && (fRotateAngle >= (fThetaTwo - fAngleOffer)))
//	if (((alphaP > alphaQ) && ((alphaP - alphaQ) < MATH_PI))
//			|| ((alphaQ - alphaP) > MATH_PI))
//		g_bIsCounterClockwiseOriented = false; // DIFFERENT
//	else
//		g_bIsCounterClockwiseOriented = true; // SAME
//
//	turnOnLED(LED_GREEN);
//
//	g_eProcessState = IDLE;
//}

//void SwarmStateOne_TShape()	// WARNING!!! This state only use for 5 robot and their all have 4 neigbors coordinates
//{
//	float const RESOLUTION = 18; // in cm
//	float fTotalDistance[4];
//	int8_t i8BestCase;
//	uint32_t ui32SelectedResult;
//
//	vector2_t vectDestination;
//
//	vector2_t vect[9];
//	int8_t i8VectBest[4];
//	uint8_t pointer[4];
//
//	int8_t i;
//
//	vector2_t vectZero;
//	vector2_t vectDiff;
//	float fAngleOffer = 0.2094395102; // ~12 degree
//	float fThetaDestinate;
//	float fDeltaAngle;
//	float fGramaAngle;
//	float fCompareAngle;
//	float fDistanceToDestinate;
//	float fStopCondition = 4; // in cm
//
//	if (g_ui32RobotID == g_ui32OriginID)
//	{
////		bool isSuccess;
////		uint8_t ui8RandomRfChannel;
////		uint16_t ui16RandomValue;
////
////		for(i = 0; i < g_ui8NeighborsCounter; i++)
////		{
////			if (NeighborsTable[i].ID == g_ui32RobotID)
////				continue;
////
////			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)
////
////			isSuccess = false;
////
////			while(1)
////			{
////				generateRandomByte();
////				while (g_ui8RandomNumber == 0);
////
////				ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
////
////				g_ui8RandomNumber =
////						(g_ui8RandomNumber < 100) ?
////								(g_ui8RandomNumber + 100) :
////								(g_ui8RandomNumber);
////
////				ui16RandomValue = g_ui8RandomNumber * 5;
////
////				delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
////
////				RF24_TX_buffer[0] = ROBOT_REQUEST_VECTOR;
////				parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
////				RF24_TX_buffer[5] = ui8RandomRfChannel;
////
////				// send request neighbor send there g_vector coordinates: <x>, <y>
////				if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 10))
////				{
////					turnOffLED(LED_RED);
////
////					RF24_setChannel(ui8RandomRfChannel);
////					RF24_TX_flush();
////					RF24_clearIrqFlag(RF24_IRQ_MASK);
////					RF24_RX_activate();
////
////					isSuccess = getNeighborVector(NeighborsTable[i].ID);
////
////					RF24_setChannel(0);
////					RF24_TX_flush();
////					RF24_clearIrqFlag(RF24_IRQ_MASK);
////					RF24_RX_activate();
////
////					turnOnLED(LED_RED);
////
////					if (isSuccess)
////						break;
////				}
////				else if (g_ui8ReTransmitCounter == 0)
////					break;
////			}
////		}
//		return;
//	}
//
//	g_bIsAllowToMove = false;
//
//	// init vectors
//	vect[0].x = 0; vect[0].y = 0;
//
//	vect[5].x = 0; vect[5].y = RESOLUTION * 2;
//	vect[1].x = 0; vect[1].y = RESOLUTION;
//	vect[3].x = 0; vect[3].y = RESOLUTION * (-1);
//	vect[7].x = 0; vect[7].y = RESOLUTION * (-2);
//
//	vect[8].x = RESOLUTION * (-2); 	vect[8].y = 0;
//	vect[4].x = RESOLUTION * (-1); 	vect[4].y = 0;
//	vect[2].x = RESOLUTION; 		vect[2].y = 0;
//	vect[6].x = RESOLUTION * 2; 	vect[6].y = 0;
//
//	// choose the best T case base on locs table
//	fTotalDistance[0] = calculateTotalDistance(vect[1], vect[3], vect[2], vect[6], &ui32SelectedResult);
//	fTotalDistance[1] = calculateTotalDistance(vect[1], vect[3], vect[4], vect[8], &ui32SelectedResult);
//	fTotalDistance[2] = calculateTotalDistance(vect[2], vect[4], vect[3], vect[7], &ui32SelectedResult);
//	fTotalDistance[3] = calculateTotalDistance(vect[2], vect[4], vect[1], vect[5], &ui32SelectedResult);
//
//	i8BestCase = 0;
//	for(i = 1; i < 4; i++)
//	{
//		if (fTotalDistance[i] < fTotalDistance[i8BestCase])
//			i8BestCase = i;
//	}
//
//	switch (i8BestCase){
//	case 0:
//		i8VectBest[0] = 1; i8VectBest[1] = 3;
//		i8VectBest[2] = 2; i8VectBest[3] = 6;
//		break;
//	case 1:
//		i8VectBest[0] = 1; i8VectBest[1] = 3;
//		i8VectBest[2] = 4; i8VectBest[3] = 8;
//		break;
//	case 2:
//		i8VectBest[0] = 2; i8VectBest[1] = 4;
//		i8VectBest[2] = 3; i8VectBest[3] = 7;
//		break;
////		i8VectBest[0] = 2; i8VectBest[1] = 4;
////		i8VectBest[2] = 1; i8VectBest[3] = 5;
//	default:
//		break;
//	}
//
//	calculateTotalDistance(vect[i8VectBest[0]], vect[i8VectBest[1]], vect[i8VectBest[2]], vect[i8VectBest[3]], &ui32SelectedResult);
//	//Note: ui32SelectedResult structure: <node1 - best0><node2 - best1><node3 - best2><node4 - best3> position in locs
//	pointer[0] = (ui32SelectedResult >> 24) & 0xFF; // best 0
//	pointer[1] = (ui32SelectedResult >> 16) & 0xFF; // best 1
//	pointer[2] = (ui32SelectedResult >> 8) & 0xFF; 	// best 2
//	pointer[3] = ui32SelectedResult & 0xFF; 		// best 3
//
//	turnOffLED(LED_ALL);
//
////	if (g_ui32RobotID == g_ui32OriginID)
////	{
////
////		for(i = 0; i < 4; )
////		{
////			g_bIsRobotResponse = false;
////			RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
////			parse32BitTo4Bytes(locs[pointer[i]].ID, &RF24_TX_buffer[1]); // 1->4
////
////			if (sendMessageToOneNeighbor(locs[pointer[i]].ID, RF24_TX_buffer, 5))
////			{
////				while(!g_bIsRobotResponse);
////				i++;
////			}
////			delayTimerNonInt(3000);
////			toggleLED(LED_RED);
////		}
////		g_eProcessState = IDLE;
////		return;
////
////		while(1)
////		{
////			g_bIsRobotResponse = false;
////			RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
////			if (sendMessageToOneNeighbor(locs[pointer[0]].ID, RF24_TX_buffer, 1))
////			{
////				while(!g_bIsRobotResponse);
////
////				while(1)
////				{
////					g_bIsRobotResponse = false;
////					RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
////					if (sendMessageToOneNeighbor(locs[pointer[3]].ID, RF24_TX_buffer, 1))
////					{
////						while(!g_bIsRobotResponse);
////
////						while(1)
////						{
////							g_bIsRobotResponse = false;
////							RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
////							if (sendMessageToOneNeighbor(locs[pointer[2]].ID, RF24_TX_buffer, 1))
////							{
////								while(!g_bIsRobotResponse);
////
////								while(1)
////								{
////									g_bIsRobotResponse = false;
////									RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
////									if (sendMessageToOneNeighbor(locs[pointer[1]].ID, RF24_TX_buffer, 1))
////									{
////										g_eProcessState = IDLE;
////										return;
////									}
////									toggleLED(LED_RED);
////									delayTimerNonInt(1000);
////								}
////							}
////							toggleLED(LED_RED);
////							delayTimerNonInt(1000);
////						}
////					}
////					toggleLED(LED_RED);
////					delayTimerNonInt(1000);
////				}
////			}
////			toggleLED(LED_RED);
////			delayTimerNonInt(1000);
////		}
////	}
//
//	if (g_ui32RobotID != g_ui32OriginID)
//	{
//		// get my destinations
//		for(i = 0; i < 4; i++)
//		{
//			if (locs[pointer[i]].ID == g_ui32RobotID)
//			{
//				vectDestination.x = vect[i8VectBest[i]].x;
//				vectDestination.y = vect[i8VectBest[i]].y;
//
//				if (i8VectBest[i] & 0x01)
//					turnOnLED(LED_GREEN);
//				else
//					turnOffLED(LED_GREEN);
//
//				if (i8VectBest[i] & 0x02)
//					turnOnLED(LED_BLUE);
//				else
//					turnOffLED(LED_BLUE);
//
//				if (i8VectBest[i] & 0x04)
//					turnOnLED(LED_RED);
//				else
//					turnOffLED(LED_RED);
//
//				break;
//			}
//		}
//
//		g_bIsCounterClockwiseOriented = false; // DIFFERENT
//		vectZero.x = g_vector.x;
//		vectZero.y = g_vector.y;
//
//		fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);
//
////		// wait for allow moving
////		while(!g_bIsAllowToMove);
//
//		uint32_t ui32DelayPeriod;
//
//		ui32DelayPeriod = (g_ui32RobotID & 0xFF) * DELAY_T_SHAPE_PERIOD;
//
//		delayTimerB(ui32DelayPeriod, true); // maybe Received ROBOT_REQUEST_TO_RUN command here!
//											// if received neighbor run command then redelay and wait
//
//		// delay timeout
//		runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2); // g_vector may be modified to new position
//		while (g_ui8NeighborsCounter < 3)
//		{
//			runForwardWithDistance(fDistanceToDestinate / (-2));
//			rotateClockwiseWithAngle(MATH_PI_DIV_2);
//			runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2);
//		}
//		vectDiff.x = g_vector.x - vectZero.x;
//		vectDiff.y = g_vector.y - vectZero.y;
//		g_fRobotOrientedAngle = calculateRobotAngleWithXAxis(vectDiff);
//
//		vectDiff.x = vectDestination.x - g_vector.x;
//		vectDiff.y = vectDestination.y - g_vector.y;
//		fThetaDestinate = calculateRobotAngleWithXAxis(vectDiff);
//
//		if (g_fRobotOrientedAngle > fThetaDestinate)
//			fDeltaAngle = g_fRobotOrientedAngle - fThetaDestinate;
//		else
//			fDeltaAngle = fThetaDestinate - g_fRobotOrientedAngle;
//
//		fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);
//
//		while(1)
//		{
//			if (g_bIsCounterClockwiseOriented) 	// SAME
//				rotateClockwiseWithAngle(fDeltaAngle / (-2));
//			else 								// DIFFERENT
//				rotateClockwiseWithAngle(fDeltaAngle / 2);
//
//			vectZero.x = g_vector.x;
//			vectZero.y = g_vector.y;
//			runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2);
//			if (g_ui8NeighborsCounter < 3)
//			{
//				runForwardWithDistance(fDistanceToDestinate / (-2));
//
////				// response done
////				RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
////				sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);
//
//				break; //TODO: fix to run reserved
//			}
//
//			vectDiff.x = g_vector.x - vectZero.x;
//			vectDiff.y = g_vector.y - vectZero.y;
//			fGramaAngle = calculateRobotAngleWithXAxis(vectDiff);
//
//			fCompareAngle = g_fRobotOrientedAngle + fDeltaAngle;
//
//			if(fGramaAngle <= (fCompareAngle + fAngleOffer) && (fGramaAngle >= (fCompareAngle - fAngleOffer)))
//			{
//				runForwardAndCalculatteNewPosition(fDistanceToDestinate / (-2));
//				if (g_ui8NeighborsCounter < 3)
//				{
//					runForwardWithDistance(fDistanceToDestinate / (-2));
//
////					// response done
////					RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
////					sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);
//
//					break; //TODO: fix to run reserved
//				}
//
//				if (g_bIsCounterClockwiseOriented) // SAME
//				{
//					rotateClockwiseWithAngle(fDeltaAngle / 2);
//					g_bIsCounterClockwiseOriented = false; // DIFF
//				}
//				else // DIFF
//				{
//					rotateClockwiseWithAngle(fDeltaAngle / (-2));
//					g_bIsCounterClockwiseOriented = true; // SAME
//				}
//			}
//			else
//			{
//				// correct
//				g_fRobotOrientedAngle = fGramaAngle;
//
//				vectDiff.x = vectDestination.x - g_vector.x;
//				vectDiff.y = vectDestination.y - g_vector.y;
//				fThetaDestinate = calculateRobotAngleWithXAxis(vectDiff);
//
//				if (g_fRobotOrientedAngle > fThetaDestinate)
//					fDeltaAngle = g_fRobotOrientedAngle - fThetaDestinate;
//				else
//					fDeltaAngle = fThetaDestinate - g_fRobotOrientedAngle;
//
//				fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);
//
//				notifyNewVectorToNeigbors();
//
//				if (fDistanceToDestinate < fStopCondition)
//				{
////					// response done
////					RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
////					sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);
//					break;
//				}
//			}
//		}
//	}
//
//	g_eProcessState = IDLE;
//}

//void RobotResponseIntHandler(void)
//{
//	switch (g_eRobotResponseState)
//	{
//		case TDOA:
//			responseTDOAResultsToNeighbor(g_ui32RequestRobotID);
//			g_eRobotResponseState = DONE;
//			break;
//
//		default:
//			break;
//	}
//}


