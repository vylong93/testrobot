#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "inc/hw_timer.h"
#include "inc/hw_ssi.h"
#include "inc/hw_i2c.h"
#include "inc/hw_eeprom.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/eeprom.h"

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"
#include "librobot/inc/TDOA.h"
#include "librobot/inc/Trilateration.h"

float32_t* FilterCoeffs;

extern float32_t g_f32PeakEnvelopeA;
extern float32_t g_f32MaxEnvelopeA;
extern float32_t g_f32PeakEnvelopeB;
extern float32_t g_f32MaxEnvelopeB;

extern CpuState_t g_eCPUState;

extern bool g_bDelayTimerAFlagAssert;
extern bool g_bDelayTimerBFlagAssert;

extern uint32_t g_ui32RobotID;
extern vector2_t g_vector;
extern bool g_bIsValidVector;
extern bool g_bIsCounterClockwiseOriented;
extern float g_fRobotOrientedAngle;

extern uint32_t g_ui32RequestRobotID;

extern bool g_bIsNetworkRotated;
extern bool g_bIsActiveCoordinatesFixing;
extern bool g_bIsGradientSearchStop;
extern uint32_t g_ui32LocalLoop;
uint32_t g_ui32LocalLoopStop; // Debug Only

extern ProcessState_t g_eProcessState;
extern RobotResponseState_t g_eRobotResponseState;
extern bool g_bBypassThisState;
extern uint8_t g_ui8ReBroadcastCounter;

extern robotMeas_t NeighborsTable[];
extern oneHopMeas_t OneHopNeighborsTable[];
extern uint8_t g_ui8NeighborsCounter;

extern location_t locs[];
extern location_t oriLocs[];

extern uint8_t g_ui8LocsCounter;

extern uint32_t g_ui32OriginID;
extern uint32_t g_ui32RotationHopID;
extern uint8_t g_ui8Hopth;
extern uint8_t g_ui8OriginNumberOfNeighbors;

extern uint8_t RF24_RX_buffer[];
extern uint8_t RF24_TX_buffer[];
extern uint8_t g_ui8ReTransmitCounter;

extern uint8_t g_ui8RandomNumber;
extern uint16_t g_pui16ADC0Result[];
extern uint16_t g_pui16ADC1Result[];
extern bool g_bIsNewTDOAResults;

void RobotProcess();
void StateOne_MeasureDistance();
void StateTwo_ExchangeTableAndCalculateLocsTable();
void StateThree_VoteOrigin();
void StateFour_RequestRotateNetwork();
void StateFive_ReduceCoordinatesError();
void StateSix_Locomotion();

void SwarmStateOne_TShape();

extern float g_f32Intercept;
extern float g_f32Slope;

float g_fStepSize;
float g_fStopCondition;
float g_fStopCondition2;

bool g_bIsRobotResponse; // T shape
bool g_bIsAllowToMove;	// T shape

int main(void)
{
// Initialize Filter Coefficients
	float32_t FilterCoeffsLocal[FILTER_ORDER] =
	{ -0.0029093551, 0.0004931756, 0.0055870397, 0.0128101468, 0.0204126528,
			0.0242806916, 0.0194602352, 0.0028870622, -0.0239973079,
			-0.0542697369, -0.0771116411, -0.0816930439, -0.0618794434,
			-0.0196709112, 0.0343706554, 0.0842319561, 0.1140602871,
			0.1140602871, 0.0842319561, 0.0343706554, -0.0196709112,
			-0.0618794434, -0.0816930439, -0.0771116411, -0.0542697369,
			-0.0239973079, 0.0028870622, 0.0194602352, 0.0242806916,
			0.0204126528, 0.0128101468, 0.0055870397, 0.0004931756,
			-0.0029093551 };
	FilterCoeffs = FilterCoeffsLocal;

	FPULazyStackingEnable();
	FPUEnable();

	// External Crystal 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
	SysCtlClockSet(
	SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	initEEPROM();

	initRobotProcess();

	initLED();

	initMotor();

	initRfModule();

	initPeripheralsForAnalogFunction();

	initSpeaker();

	initLowPowerMode();

	initTimerDelay();

	TDOA_initFilters(FilterCoeffs);

	while (1)
	{
		RobotProcess();
	}
}

void StateOne_MeasureDistance()
{
	uint16_t ui16RandomValue;
	float32_t f32_inputValue;
	float32_t f32_outputValue;

	// set this as origin
	g_ui32OriginID = g_ui32RobotID;
	g_vector.x = 0;
	g_vector.y = 0;

	//init variables
	g_bIsNetworkRotated = false;

	turnOnLED(LED_BLUE);

	g_bBypassThisState = false;

	delayTimerA(DELAY_MEASURE_DISTANCE_STATE*4, false);

	while (!g_bDelayTimerAFlagAssert)
	{
		generateRandomByte();

		while (g_ui8RandomNumber == 0)
			;
		g_ui8RandomNumber =
				(g_ui8RandomNumber < 100) ?
						(g_ui8RandomNumber + 100) : (g_ui8RandomNumber);
		ui16RandomValue = (g_ui32RobotID << 8) | g_ui8RandomNumber;

		delayTimerB(ui16RandomValue, false);

		while (!g_bDelayTimerBFlagAssert)
		{
			g_bIsNewTDOAResults = false;

			if (NeighborsTable[g_ui8NeighborsCounter].ID != 0)
			{
				turnOnLED(LED_GREEN);

				reloadDelayTimerA();

				while(!g_bIsNewTDOAResults);

				g_bIsNewTDOAResults = false;

				if (g_f32MaxEnvelopeA < MAX_THRESHOLD
						|| g_f32MaxEnvelopeB < MAX_THRESHOLD
						|| g_f32PeakEnvelopeA > MAX_SAMPLE_POSITION
						|| g_f32PeakEnvelopeB > MAX_SAMPLE_POSITION)
				{
					NeighborsTable[g_ui8NeighborsCounter].ID = 0;
				}
				else
				{
					g_f32PeakEnvelopeA = (g_f32PeakEnvelopeA - g_f32Intercept)
							/ g_f32Slope;
					g_f32PeakEnvelopeB = (g_f32PeakEnvelopeB - g_f32Intercept)
							/ g_f32Slope;

					f32_inputValue =
							(((g_f32PeakEnvelopeA * g_f32PeakEnvelopeA
									+ g_f32PeakEnvelopeB
											* g_f32PeakEnvelopeB) / 2.0)
									- (DISTANCE_BETWEEN_TWO_MICS_SQR / 4.0))
									* 65536.0; // * 256^2

					f32_outputValue = vsqrtf(f32_inputValue);

					NeighborsTable[g_ui8NeighborsCounter].distance =
							(uint16_t) (f32_outputValue + 0.5);

					g_ui8NeighborsCounter++;

					//TODO: search the worth result and replace it by the current result if better
					g_ui8NeighborsCounter =
							(g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH) ?
									(g_ui8NeighborsCounter) :
									(NEIGHBOR_TABLE_LENGTH);
				}
				turnOffLED(LED_GREEN);
			}
		}

		if (!g_bBypassThisState)
		{
			turnOffLED(LED_BLUE);

			reloadDelayTimerA();

			RF24_TX_buffer[0] = ROBOT_REQUEST_SAMPLING_MIC;
			RF24_TX_buffer[1] = g_ui32RobotID >> 24;
			RF24_TX_buffer[2] = g_ui32RobotID >> 16;
			RF24_TX_buffer[3] = g_ui32RobotID >> 8;
			RF24_TX_buffer[4] = g_ui32RobotID;

			broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
			// WARING!!! DO NOT INSERT ANY CODE IN HERE!
			ROM_SysCtlDelay(DELAY_START_SPEAKER);
			// WARING!!! DO NOT INSERT ANY CODE IN HERE!
			startSpeaker();

			RF24_RX_activate();

			g_bBypassThisState = true;
		}
	}

	turnOnLED(LED_ALL);

	g_eProcessState = EXCHANGE_TABLE;
}

void StateTwo_ExchangeTableAndCalculateLocsTable()
{
	uint16_t ui16RandomValue;
	uint8_t ui8RandomRfChannel;
	uint8_t neighborsTablePointer;

	neighborsTablePointer = 0;

	Tri_clearLocs(locs, &g_ui8LocsCounter);

	delayTimerA(DELAY_EXCHANGE_TABLE_STATE, false);

	while (!g_bDelayTimerAFlagAssert)
	{
		while (neighborsTablePointer < g_ui8NeighborsCounter)
		{
			generateRandomByte();

			while (g_ui8RandomNumber == 0)
				;

			ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125

			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)

			while (1)
			{
				reloadDelayTimerA();

				generateRandomByte();
				while (g_ui8RandomNumber == 0)
					;
				g_ui8RandomNumber =
						(g_ui8RandomNumber < 100) ?
								(g_ui8RandomNumber + 100) :
								(g_ui8RandomNumber);

				ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);

				delayTimerB(ui16RandomValue, true); // maybe Received Request table command here!

				while (!g_bDelayTimerBFlagAssert)
					; // this line make sure robot will re delay after send neighbors table to another robot

				// delay timeout
				RF24_TX_buffer[0] = ROBOT_REQUEST_NEIGHBORS_TABLE;
				RF24_TX_buffer[1] = g_ui32RobotID >> 24;
				RF24_TX_buffer[2] = g_ui32RobotID >> 16;
				RF24_TX_buffer[3] = g_ui32RobotID >> 8;
				RF24_TX_buffer[4] = g_ui32RobotID;
				RF24_TX_buffer[5] = ui8RandomRfChannel;

				if (sendMessageToOneNeighbor(
						NeighborsTable[neighborsTablePointer].ID,
						RF24_TX_buffer, 6))
				{
					turnOffLED(LED_RED);

					RF24_setChannel(ui8RandomRfChannel);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					getNeighborNeighborsTable();

					neighborsTablePointer++;

					RF24_setChannel(0);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					turnOnLED(LED_RED);
					break;
				}
				else if (g_ui8ReTransmitCounter == 0)
				{
					break;
				}
			}

			if (neighborsTablePointer >= g_ui8NeighborsCounter) // exchane table finished
			{
				neighborsTablePointer = g_ui8NeighborsCounter;

				if (g_ui8LocsCounter == 0) // try to calculate neighbor's coordination
				{
					Tri_addLocation(g_ui32RobotID, 0, 0);

					Tri_findLocs(NeighborsTable, OneHopNeighborsTable);
				}
			}
		}
	}

	turnOffLED(LED_BLUE);

	g_eProcessState = VOTE_ORIGIN;
}

void StateThree_VoteOrigin()
{
	if(g_ui32OriginID == g_ui32RobotID) // haven't update
	{
		// set This As Origin Node
		g_ui8OriginNumberOfNeighbors = g_ui8LocsCounter;
		g_ui8Hopth = 0;
		g_ui32RotationHopID = g_ui32RobotID;
	}
	else // haven't update
	{
		if (g_ui8OriginNumberOfNeighbors < g_ui8LocsCounter)
		{
			// set This As Origin Node
			g_ui32OriginID = g_ui32RobotID;
			g_ui8OriginNumberOfNeighbors = g_ui8LocsCounter;
			g_ui8Hopth = 0;
			g_ui32RotationHopID = g_ui32RobotID;
		}
	}

	g_ui8ReBroadcastCounter = 0;

	g_bBypassThisState = false;

	delayTimerA(DELAY_ROTATE_NETWORK, false);	// start State Timer

	while (!g_bDelayTimerAFlagAssert)
	{
		turnOffLED(LED_GREEN);

		generateRandomByte();
		while (g_ui8RandomNumber == 0)
			;

		delayTimerB(DELAY_REBROADCAST, true);

		// delay timeout
		if (!g_bBypassThisState)
		{
			reloadDelayTimerA();

			turnOnLED(LED_GREEN);

			// construct message: <Update Network Origin COMMAND> <thisOriginID> <thisOriginNumberOfNeighbors> <thisHopth>
			RF24_TX_buffer[0] = ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN;
			RF24_TX_buffer[1] = g_ui32OriginID >> 24;
			RF24_TX_buffer[2] = g_ui32OriginID >> 16;
			RF24_TX_buffer[3] = g_ui32OriginID >> 8;
			RF24_TX_buffer[4] = g_ui32OriginID;
			RF24_TX_buffer[5] = g_ui8OriginNumberOfNeighbors;
			RF24_TX_buffer[6] = g_ui8Hopth;

			broadcastLocalNeighbor(RF24_TX_buffer, 7);
			RF24_RX_activate();

			g_ui8ReBroadcastCounter++;

			if (g_ui8ReBroadcastCounter >= REBROADCAST_TIMES)
				g_bBypassThisState = true;
		}
	}

	g_eProcessState = ROTATE_NETWORK;
}

void StateFour_RequestRotateNetwork()
{
	uint8_t ui8RandomRfChannel;
	uint32_t tableSizeInByte;
	uint32_t neighborID;
	int8_t i;
	int32_t tempValue;

	if (g_ui32OriginID & 0x01)
		turnOnLED(LED_GREEN);
	else
		turnOffLED(LED_GREEN);

	if (g_ui32OriginID & 0x02)
		turnOnLED(LED_BLUE);
	else
		turnOffLED(LED_BLUE);

	if (g_ui32OriginID & 0x04)
		turnOnLED(LED_RED);
	else
		turnOffLED(LED_RED);

	if (g_ui32OriginID == g_ui32RobotID)
	{
		// I'am Original
		g_bIsNetworkRotated = true;

		g_ui32RotationHopID = g_ui32RobotID;

		g_vector.x = 0;
		g_vector.y = 0;
	}

	// waiting request and rotate
	while(!g_bIsNetworkRotated);

	g_ui8ReTransmitCounter = 0; // disable software trigger retransmit

	tableSizeInByte = sizeof(location_t) * g_ui8LocsCounter;

	i = 0;
	while (i < g_ui8LocsCounter)
	{
		generateRandomByte();
		while (g_ui8RandomNumber == 0)
			;
		g_ui8RandomNumber = (g_ui8RandomNumber < 100) ? (g_ui8RandomNumber + 100) : (g_ui8RandomNumber);

		ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125

		delayTimerB(g_ui8RandomNumber + 1000, true);

		neighborID = locs[i].ID;

		if (neighborID == g_ui32RotationHopID || neighborID == g_ui32RobotID)
		{
			i++;
			continue;
		}

		// send rotate network command and locs size
		RF24_TX_buffer[0] = ROBOT_REQUEST_ROTATE_NETWORK;

		RF24_TX_buffer[1] = g_ui32RobotID >> 24;
		RF24_TX_buffer[2] = g_ui32RobotID >> 16;
		RF24_TX_buffer[3] = g_ui32RobotID >> 8;
		RF24_TX_buffer[4] = g_ui32RobotID;

		RF24_TX_buffer[5] = tableSizeInByte >> 24;
		RF24_TX_buffer[6] = tableSizeInByte >> 16;
		RF24_TX_buffer[7] = tableSizeInByte >> 8;
		RF24_TX_buffer[8] = tableSizeInByte;

		tempValue = (int8_t)(g_vector.x * 65536.0 + 0.5);
		RF24_TX_buffer[9] = tempValue >> 24;
		RF24_TX_buffer[10] = tempValue >> 16;
		RF24_TX_buffer[11] = tempValue >> 8;
		RF24_TX_buffer[12] = tempValue;

		tempValue = (int8_t)(g_vector.y * 65536.0 + 0.5);
		RF24_TX_buffer[13] = tempValue >> 24;
		RF24_TX_buffer[14] = tempValue >> 16;
		RF24_TX_buffer[15] = tempValue >> 8;
		RF24_TX_buffer[16] = tempValue;

		RF24_TX_buffer[17] = ui8RandomRfChannel;

		if (sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, 18))
		{
			turnOffLED(LED_RED);

			RF24_setChannel(ui8RandomRfChannel);
			RF24_TX_flush();
			RF24_clearIrqFlag(RF24_IRQ_MASK);
			RF24_RX_activate();

			SysCtlDelay(3000); // delay 150us
			sendMessageToOneNeighbor(neighborID, (uint8_t*)locs, tableSizeInByte);
			i++;

			RF24_setChannel(0);
			RF24_TX_flush();
			RF24_clearIrqFlag(RF24_IRQ_MASK);
			RF24_RX_activate();

			turnOnLED(LED_RED);
		}
	}
	// Coordinates rotated!!!

	// offset locs table to real vector
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		locs[i].vector.x += g_vector.x;
		locs[i].vector.y += g_vector.y;
	}

//	g_eProcessState = IDLE;

	turnOffLED(LED_ALL);
	g_eProcessState = REDUCE_ERROR;
}

void StateFive_ReduceCoordinatesError()
{
	int8_t i;
	uint8_t ui8RandomRfChannel;
	uint16_t ui16RandomValue;
	bool isSuccess;

	g_fStepSize = 0.2f;
	g_fStopCondition = 0.3f;
	g_fStopCondition2 = 1.2f;

//	float fRandomeStepSize;					// random 2.0f -> 4.0f
//	float const fStepSize = 0.2f; 			//
//	float const fStopCondition = 0.003f; 	// unit m
//	float const fStopCondition2 = 0.12f; 	// unit m

	uint8_t ui8VectorCounter;
	vector2_t vectGradienNew;
	vector2_t vectGradienOld;
	vector2_t vectEstimatePosNew;
	vector2_t vectEstimatePosOld;

	g_ui32LocalLoopStop = 1; // DEBUG only

	if (g_ui32OriginID == g_ui32RobotID)
	{
		g_bIsActiveCoordinatesFixing = false;
		g_bIsGradientSearchStop = true;
		g_vector.x = 0;
		g_vector.y = 0;

//		while(1)
//		{
//			rfDelayLoop(DELAY_CYCLES_5MS * 500); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
//			toggleLED(LED_ALL);
//
//			for(i = 0; i < g_ui8NeighborsCounter; i++)
//			{
//				if (NeighborsTable[i].ID == g_ui32RobotID)
//					continue;
//
//				g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)
//
//				isSuccess = false;
//
//				while(1)
//				{
//					generateRandomByte();
//					while (g_ui8RandomNumber == 0);
//
//					ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125
//
//					g_ui8RandomNumber =
//							(g_ui8RandomNumber < 100) ?
//									(g_ui8RandomNumber + 100) :
//									(g_ui8RandomNumber);
//
//					ui16RandomValue = g_ui8RandomNumber * 5;
//
//					delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!
//
//					RF24_TX_buffer[0] = ROBOT_REQUEST_VECTOR;
//					parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
//					RF24_TX_buffer[5] = ui8RandomRfChannel;
//
//					// send request neighbor send there g_vector coordinates: <x>, <y>
//					if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 10))
//					{
//						turnOffLED(LED_RED);
//
//						RF24_setChannel(ui8RandomRfChannel);
//						RF24_TX_flush();
//						RF24_clearIrqFlag(RF24_IRQ_MASK);
//						RF24_RX_activate();
//
//						isSuccess = getNeighborVector(NeighborsTable[i].ID);
//
//						RF24_setChannel(0);
//						RF24_TX_flush();
//						RF24_clearIrqFlag(RF24_IRQ_MASK);
//						RF24_RX_activate();
//
//						turnOnLED(LED_RED);
//
//						if (isSuccess)
//							break;
//					}
//					else if (g_ui8ReTransmitCounter == 0)
//						break;
//				}
//			}
//		}
	}
	else
	{
		g_bIsActiveCoordinatesFixing = true;

		// Calculate average position
		g_ui32LocalLoop = 0;

		ui8VectorCounter = 1;

		for(i = 0; i < g_ui8NeighborsCounter; i++)
		{
			if (NeighborsTable[i].ID == g_ui32RobotID)
				continue;

			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)

			isSuccess = false;

			while (1) // wait for neighbor g_bIsNetworkRotated = true and get my vector
			{
				generateRandomByte();
				while (g_ui8RandomNumber == 0);

				ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125

				g_ui8RandomNumber =
						(g_ui8RandomNumber < 100) ?
								(g_ui8RandomNumber + 100) :
								(g_ui8RandomNumber);

				ui16RandomValue = g_ui8RandomNumber * 10;

				delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!

				while (!g_bDelayTimerBFlagAssert)
					; // this line make sure robot will re delay after response to another robot

				// delay timeout
				RF24_TX_buffer[0] = ROBOT_REQUEST_MY_VECTOR;
				parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
				RF24_TX_buffer[5] = ui8RandomRfChannel;

				if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 6))
				{
					turnOffLED(LED_RED);

					RF24_setChannel(ui8RandomRfChannel);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					isSuccess = getMyVector(&ui8VectorCounter);

					RF24_setChannel(0);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					turnOnLED(LED_RED);

					if (isSuccess)
						break;
				}
				else if (g_ui8ReTransmitCounter == 0)
					break;
			}
		}

		g_vector.x /= ui8VectorCounter;
		g_vector.y /= ui8VectorCounter;

		synchronousLocsTableAndMyVector();

		g_ui32LocalLoop = 1;

		turnOnLED(LED_ALL); // OK

		// Update locs table
		updateLocsByOtherRobotCurrentPosition(true);

		turnOffLED(LED_GREEN); // OK

		vectEstimatePosNew.x = g_vector.x;
		vectEstimatePosNew.y = g_vector.y;

		vectGradienNew.x = 0;
		vectGradienNew.y = 0;

		g_bIsGradientSearchStop = false;

		g_ui32LocalLoop++;

		while(!g_bIsGradientSearchStop)
		{
			// Algorithm 1
			while(!g_bIsGradientSearchStop)
			{
				//DEBUG only
				while(g_ui32LocalLoop > g_ui32LocalLoopStop);

				turnOffLED(LED_BLUE);

				updateGradient(&vectGradienNew, false);

				updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, g_fStepSize);
				synchronousLocsTableAndMyVector();

				g_ui32LocalLoop++;

				g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition);

				updateLocsByOtherRobotCurrentPosition(false);

				if (g_ui32LocalLoop & 0x01)
					turnOnLED(LED_GREEN);
				else
					turnOffLED(LED_GREEN);
			}

			turnOnLED(LED_BLUE);
//
//			//TODO: uncomment: Escape Local Minima
//			updateGradient(&vectGradienNew, true);
//
//			fRandomeStepSize = generateRandomRange(2.0f, 4.0f);
//
//			updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, fRandomeStepSize);
//			synchronousLocsTableAndMyVector();
//
//			g_ui32LocalLoop++;
//
//			g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition2);
//
//			updateLocsByOtherRobotCurrentPosition(false);
		}

//		g_ui32LocalLoop = 0;
//		g_vector.x = vectEstimatePosOld.x;
//		g_vector.y = vectEstimatePosOld.y;
//		updateLocsByOtherRobotCurrentPosition(true);

		g_bIsActiveCoordinatesFixing = false;

		turnOffLED(LED_BLUE);
	}

	g_bIsValidVector = true;

	//g_eProcessState = LOCOMOTION;
	g_eProcessState = IDLE;
}

void StateSix_Locomotion()
{
	vector2_t vectZero;
	vector2_t vectOne;
	vector2_t vectDiff;

	float fAngleOffer = 0.2094395102; // ~12 degree
	float fRotateAngle = MATH_PI_DIV_2; // ~90 degree
	float fThetaOne;
	float fThetaTwo;

	uint16_t ui16RandomValue;

	g_bIsNewTDOAResults = false;

	turnOffLED(LED_ALL);

	// save my old vector for initialize position V0
	vectZero.x = g_vector.x;
	vectZero.y = g_vector.y;

	// delay random
	generateRandomByte();
	while (g_ui8RandomNumber == 0)
		;
	g_ui8RandomNumber =
			(g_ui8RandomNumber < 100) ?
					(g_ui8RandomNumber + 100) :
					(g_ui8RandomNumber);

	ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);

	ui16RandomValue = g_ui8RandomNumber * 10;

	ui16RandomValue = ui16RandomValue + (g_ui32RobotID & 0xFF) * DELAY_LOCOMOTION_PERIOD;

	delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_TO_RUN command here!
										// if received neighbor run command then redelay and wait

	// delay timeout
	g_bIsValidVector = false;

	runForwardAndCalculatteNewPosition(7.0); // g_vector may be modified to new position
	if (g_ui8NeighborsCounter < 3)
	{
		//TODO: reserved
		runForwardWithDistance(-7.0);

		// WARNING!!! This vector may be not correct
		Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);
		g_bIsValidVector = true;
		g_eProcessState = IDLE;
		return;
	}

	vectOne.x = g_vector.x;
	vectOne.y = g_vector.y;

	rotateClockwiseWithAngle(fRotateAngle);

	runForwardAndCalculatteNewPosition(6.0); // g_vector may be modified to new position
	if (g_ui8NeighborsCounter < 3)
	{
		//TODO: reserved
		runForwardWithDistance(-6.0);

		// WARNING!!! This vector may be not correct
		Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);
		g_bIsValidVector = true;
		g_eProcessState = IDLE;
		return; // Not enough neighbors
	}

	g_bIsValidVector = true;

	notifyNewVectorToNeigbors();

	vectDiff.x = vectOne.x - vectZero.x;
	vectDiff.y = vectOne.y - vectZero.y;
	fThetaOne = calculateRobotAngleWithXAxis(vectDiff);

	vectDiff.x = g_vector.x - vectOne.x;
	vectDiff.y = g_vector.y - vectOne.y;
	fThetaTwo = calculateRobotAngleWithXAxis(vectDiff);

	g_fRobotOrientedAngle = fThetaTwo;

	if (fRotateAngle < 0)
		fRotateAngle = fThetaOne - fRotateAngle;
	else
		fRotateAngle += fThetaOne;

	while(fRotateAngle > MATH_PI_MUL_2)
		fRotateAngle -= MATH_PI_MUL_2;

	if(fRotateAngle <= (fThetaTwo + fAngleOffer) && (fRotateAngle >= (fThetaTwo - fAngleOffer)))
		g_bIsCounterClockwiseOriented = true; // SAME
	else
		g_bIsCounterClockwiseOriented = false; // DIFFERENT

	turnOnLED(LED_GREEN);

	g_eProcessState = IDLE;
}

void SwarmStateOne_TShape()	// WARNING!!! This state only use for 5 robot and their all have 4 neigbors coordinates
{
	uint16_t ui16RandomValue;

	float const RESOLUTION = 18; // in cm
	float fTotalDistance[4];
	int8_t i8BestCase;
	uint32_t ui32SelectedResult;

	vector2_t vectDestination;

	vector2_t vect[9];
	int8_t i8VectBest[4];
	uint8_t pointer[4];

	int8_t i;

	vector2_t vectZero;
	vector2_t vectDiff;
	float fAngleOffer = 0.2094395102; // ~12 degree
	float fThetaDestinate;
	float fDeltaAngle;
	float fGramaAngle;
	float fCompareAngle;
	float fDistanceToDestinate;
	float fStopCondition = 4; // in cm

	if (g_ui32RobotID == g_ui32OriginID)
	{
		bool isSuccess;
		uint8_t ui8RandomRfChannel;
		uint16_t ui16RandomValue;

		for(i = 0; i < g_ui8NeighborsCounter; i++)
		{
			if (NeighborsTable[i].ID == g_ui32RobotID)
				continue;

			g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)

			isSuccess = false;

			while(1)
			{
				generateRandomByte();
				while (g_ui8RandomNumber == 0);

				ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125

				g_ui8RandomNumber =
						(g_ui8RandomNumber < 100) ?
								(g_ui8RandomNumber + 100) :
								(g_ui8RandomNumber);

				ui16RandomValue = g_ui8RandomNumber * 5;

				delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_MY_VECTOR command here!

				RF24_TX_buffer[0] = ROBOT_REQUEST_VECTOR;
				parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
				RF24_TX_buffer[5] = ui8RandomRfChannel;

				// send request neighbor send there g_vector coordinates: <x>, <y>
				if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 10))
				{
					turnOffLED(LED_RED);

					RF24_setChannel(ui8RandomRfChannel);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					isSuccess = getNeighborVector(NeighborsTable[i].ID);

					RF24_setChannel(0);
					RF24_TX_flush();
					RF24_clearIrqFlag(RF24_IRQ_MASK);
					RF24_RX_activate();

					turnOnLED(LED_RED);

					if (isSuccess)
						break;
				}
				else if (g_ui8ReTransmitCounter == 0)
					break;
			}
		}
	}

	g_bIsAllowToMove = false;

	// init vectors
	vect[0].x = 0; vect[0].y = 0;

	vect[5].x = 0; vect[5].y = RESOLUTION * 2;
	vect[1].x = 0; vect[1].y = RESOLUTION;
	vect[3].x = 0; vect[3].y = RESOLUTION * (-1);
	vect[7].x = 0; vect[7].y = RESOLUTION * (-2);

	vect[8].x = RESOLUTION * (-2); 	vect[8].y = 0;
	vect[4].x = RESOLUTION * (-1); 	vect[4].y = 0;
	vect[2].x = RESOLUTION; 		vect[2].y = 0;
	vect[6].x = RESOLUTION * 2; 	vect[6].y = 0;

	// choose the best T case base on locs table
	fTotalDistance[0] = calculateTotalDistance(vect[1], vect[3], vect[2], vect[6], &ui32SelectedResult);
	fTotalDistance[1] = calculateTotalDistance(vect[1], vect[3], vect[4], vect[8], &ui32SelectedResult);
	fTotalDistance[2] = calculateTotalDistance(vect[2], vect[4], vect[3], vect[7], &ui32SelectedResult);
	fTotalDistance[3] = calculateTotalDistance(vect[2], vect[4], vect[1], vect[5], &ui32SelectedResult);

	i8BestCase = 0;
	for(i = 1; i < 4; i++)
	{
		if (fTotalDistance[i] < fTotalDistance[i8BestCase])
			i8BestCase = i;
	}

	switch (i8BestCase){
	case 0:
		i8VectBest[0] = 1; i8VectBest[1] = 3;
		i8VectBest[2] = 2; i8VectBest[3] = 6;
		break;
	case 1:
		i8VectBest[0] = 1; i8VectBest[1] = 3;
		i8VectBest[2] = 4; i8VectBest[3] = 8;
		break;
	case 2:
		i8VectBest[0] = 2; i8VectBest[1] = 4;
		i8VectBest[2] = 3; i8VectBest[3] = 7;
		break;
		i8VectBest[0] = 2; i8VectBest[1] = 4;
		i8VectBest[2] = 1; i8VectBest[3] = 5;
	default:
		break;
	}

	calculateTotalDistance(vect[i8VectBest[0]], vect[i8VectBest[1]], vect[i8VectBest[2]], vect[i8VectBest[3]], &ui32SelectedResult);
	//Note: ui32SelectedResult structure: <node1 - best0><node2 - best1><node3 - best2><node4 - best3> position in locs
	pointer[0] = (ui32SelectedResult >> 24) & 0xFF; // best 0
	pointer[1] = (ui32SelectedResult >> 16) & 0xFF; // best 1
	pointer[2] = (ui32SelectedResult >> 8) & 0xFF; 	// best 2
	pointer[3] = ui32SelectedResult & 0xFF; 		// best 3

	turnOffLED(LED_ALL);

	if (g_ui32RobotID == g_ui32OriginID)
	{

//		for(i = 0; i < 4; )
//		{
//			g_bIsRobotResponse = false;
//			RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
//			parse32BitTo4Bytes(locs[pointer[i]].ID, &RF24_TX_buffer[1]); // 1->4
//
//			if (sendMessageToOneNeighbor(locs[pointer[i]].ID, RF24_TX_buffer, 5))
//			{
//				while(!g_bIsRobotResponse);
//				i++;
//			}
//			delayTimerNonInt(3000);
//			toggleLED(LED_RED);
//		}
//		g_eProcessState = IDLE;
//		return;

//		while(1)
//		{
//			g_bIsRobotResponse = false;
//			RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
//			if (sendMessageToOneNeighbor(locs[pointer[0]].ID, RF24_TX_buffer, 1))
//			{
//				while(!g_bIsRobotResponse);
//
//				while(1)
//				{
//					g_bIsRobotResponse = false;
//					RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
//					if (sendMessageToOneNeighbor(locs[pointer[3]].ID, RF24_TX_buffer, 1))
//					{
//						while(!g_bIsRobotResponse);
//
//						while(1)
//						{
//							g_bIsRobotResponse = false;
//							RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
//							if (sendMessageToOneNeighbor(locs[pointer[2]].ID, RF24_TX_buffer, 1))
//							{
//								while(!g_bIsRobotResponse);
//
//								while(1)
//								{
//									g_bIsRobotResponse = false;
//									RF24_TX_buffer[0] = ROBOT_ALLOW_MOVE_TO_T_SHAPE;
//									if (sendMessageToOneNeighbor(locs[pointer[1]].ID, RF24_TX_buffer, 1))
//									{
//										g_eProcessState = IDLE;
//										return;
//									}
//									toggleLED(LED_RED);
//									delayTimerNonInt(1000);
//								}
//							}
//							toggleLED(LED_RED);
//							delayTimerNonInt(1000);
//						}
//					}
//					toggleLED(LED_RED);
//					delayTimerNonInt(1000);
//				}
//			}
//			toggleLED(LED_RED);
//			delayTimerNonInt(1000);
//		}
	}
	else
	{
		// get my destinations
		for(i = 0; i < 4; i++)
		{
			if (locs[pointer[i]].ID == g_ui32RobotID)
			{
				vectDestination.x = vect[i8VectBest[i]].x;
				vectDestination.y = vect[i8VectBest[i]].y;

				if (i8VectBest[i] & 0x01)
					turnOnLED(LED_GREEN);
				else
					turnOffLED(LED_GREEN);

				if (i8VectBest[i] & 0x02)
					turnOnLED(LED_BLUE);
				else
					turnOffLED(LED_BLUE);

				if (i8VectBest[i] & 0x04)
					turnOnLED(LED_RED);
				else
					turnOffLED(LED_RED);

				break;
			}
		}

//		// wait for allow moving
//		while(!g_bIsAllowToMove);

		// delay random
		generateRandomByte();
		while (g_ui8RandomNumber == 0)
			;
		g_ui8RandomNumber =
				(g_ui8RandomNumber < 100) ?
						(g_ui8RandomNumber + 100) :
						(g_ui8RandomNumber);

		ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);

		ui16RandomValue = g_ui8RandomNumber * 10;

		ui16RandomValue = ui16RandomValue + (g_ui32RobotID & 0xFF) * DELAY_LOCOMOTION_PERIOD;

		delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_TO_RUN command here!
											// if received neighbor run command then redelay and wait

		// delay timeout
		g_bIsCounterClockwiseOriented = false; // DIFFERENT
		vectZero.x = g_vector.x;
		vectZero.y = g_vector.y;

		fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);

		runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2); // g_vector may be modified to new position
		while (g_ui8NeighborsCounter < 3)
		{
			runForwardWithDistance(fDistanceToDestinate / (-2));
			rotateClockwiseWithAngle(MATH_PI_DIV_2);
			runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2);
		}
		vectDiff.x = g_vector.x - vectZero.x;
		vectDiff.y = g_vector.y - vectZero.y;
		g_fRobotOrientedAngle = calculateRobotAngleWithXAxis(vectDiff);

		vectDiff.x = vectDestination.x - g_vector.x;
		vectDiff.y = vectDestination.y - g_vector.y;
		fThetaDestinate = calculateRobotAngleWithXAxis(vectDiff);

		if (g_fRobotOrientedAngle > fThetaDestinate)
			fDeltaAngle = g_fRobotOrientedAngle - fThetaDestinate;
		else
			fDeltaAngle = fThetaDestinate - g_fRobotOrientedAngle;

		fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);

		while(1)
		{
			if (g_bIsCounterClockwiseOriented) 	// SAME
				rotateClockwiseWithAngle(MATH_PI_MUL_2 - fDeltaAngle);
			else 								// DIFFERENT
				rotateClockwiseWithAngle(fDeltaAngle);

			vectZero.x = g_vector.x;
			vectZero.y = g_vector.y;
			runForwardAndCalculatteNewPosition(fDistanceToDestinate / 2);
			if (g_ui8NeighborsCounter < 3)
			{
				runForwardWithDistance(fDistanceToDestinate / (-2));

				// response done
				RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
				sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);

				break; //TODO: fix to run reserved
			}

			vectDiff.x = g_vector.x - vectZero.x;
			vectDiff.y = g_vector.y - vectZero.y;
			fGramaAngle = calculateRobotAngleWithXAxis(vectDiff);

			fCompareAngle = g_fRobotOrientedAngle + 2 * fDeltaAngle;

			if(fGramaAngle <= (fCompareAngle + fAngleOffer) && (fGramaAngle >= (fCompareAngle - fAngleOffer)))
			{
				runForwardAndCalculatteNewPosition(fDistanceToDestinate / (-2));
				if (g_ui8NeighborsCounter < 3)
				{
					runForwardWithDistance(fDistanceToDestinate / (-2));

					// response done
					RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
					sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);

					break; //TODO: fix to run reserved
				}

				if (g_bIsCounterClockwiseOriented) // SAME
				{
					rotateClockwiseWithAngle(fDeltaAngle);
					g_bIsCounterClockwiseOriented = false; // DIFF
				}
				else // DIFF
				{
					rotateClockwiseWithAngle(MATH_PI_MUL_2 - fDeltaAngle);
					g_bIsCounterClockwiseOriented = true; // SAME
				}
			}
			else
			{
				// correct
				g_fRobotOrientedAngle = fGramaAngle;

				vectDiff.x = vectDestination.x - g_vector.x;
				vectDiff.y = vectDestination.y - g_vector.y;
				fThetaDestinate = calculateRobotAngleWithXAxis(vectDiff);

				if (g_fRobotOrientedAngle > fThetaDestinate)
					fDeltaAngle = g_fRobotOrientedAngle - fThetaDestinate;
				else
					fDeltaAngle = fThetaDestinate - g_fRobotOrientedAngle;

				fDistanceToDestinate = calculateDistanceBetweenTwoNode(vectDestination, g_vector);

				notifyNewVectorToNeigbors();

				if (fDistanceToDestinate < fStopCondition)
				{
					// response done
					RF24_TX_buffer[0] = ROBOT_REPONSE_MOVE_COMPLETED;
					sendMessageToOneNeighbor(g_ui32OriginID, RF24_TX_buffer, 1);
					break;
				}
			}
		}
	}

	g_eProcessState = IDLE;
}

void RobotProcess()
{
	switch (g_eProcessState)
	{
	case MEASURE_DISTANCE:
		StateOne_MeasureDistance();
		break;

	case EXCHANGE_TABLE:
		StateTwo_ExchangeTableAndCalculateLocsTable();
		break;

	case VOTE_ORIGIN:
		StateThree_VoteOrigin();
		break;

	case ROTATE_NETWORK:
		StateFour_RequestRotateNetwork();
		break;

	case REDUCE_ERROR:
		StateFive_ReduceCoordinatesError();
		break;

	case LOCOMOTION:
		StateSix_Locomotion();
		break;

	case T_SHAPE:
		SwarmStateOne_TShape();
		break;

	default: // IDLE state
		rfDelayLoop(DELAY_CYCLES_5MS * 25);
		toggleLED(LED_RED);
		break;
	}
}

inline void RF24_IntHandler()
{
	if (GPIOIntStatus(RF24_INT_PORT, RF24_INT_Channel))
	{
		uint8_t rfIrqFlag = RF24_getIrqFlag(RF24_IRQ_MASK);

		if (rfIrqFlag & RF24_IRQ_TX)
		{
			RF24_clearIrqFlag(RF24_IRQ_TX);
			RF24_RX_activate();
		}

		if (rfIrqFlag & RF24_IRQ_MAX_RETRANS)
		{
			RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);
			RF24_RX_activate();
		}

		if (rfIrqFlag & RF24_IRQ_RX)
		{
			uint8_t length;
			length = RF24_RX_getPayloadWidth();
			RF24_RX_getPayloadData(length, RF24_RX_buffer);

			// Commands allowed to be processed when uC is in sleep/deep_sleep mode
			if (g_eCPUState != RUN_MODE)
			{
				switch (RF24_RX_buffer[0])
				{
				case COMMAND_SLEEP:
					g_eCPUState = SLEEP_MODE;
					IntTrigger(INT_SW_TRIGGER_LPM);
					break;

				case COMMAND_DEEP_SLEEP:
					g_eCPUState = DEEP_SLEEP_MODE;
					IntTrigger(INT_SW_TRIGGER_LPM);
					break;

				case COMMAND_WAKE_UP:
					wakeUpFormLPM();
					break;

				case COMMAND_RESET:
					HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
					NVIC_APINT_SYSRESETREQ);
					break;

				default:
					IntTrigger(INT_SW_TRIGGER_LPM);
					break;
				}
			}
			else // CPU in Run mode
			{
				switch (g_eProcessState)
				// limit valid rf command in each process state
				{
				case MEASURE_DISTANCE:
					if (RF24_RX_buffer[0] == ROBOT_REQUEST_SAMPLING_MIC)
					{
						// DO NOT INSERT ANY CODE IN HERE!
						startSamplingMicSignals();

						NeighborsTable[g_ui8NeighborsCounter].ID =
								(RF24_RX_buffer[1] << 24)
										| (RF24_RX_buffer[2] << 16)
										| (RF24_RX_buffer[3] << 8)
										| RF24_RX_buffer[4];
					}

					break;

				default: // IDLE state
					switch (RF24_RX_buffer[0])
					{
					case ROBOT_REQUEST_SAMPLING_MIC:
						// DO NOT INSERT ANY CODE IN HERE!
						startSamplingMicSignals();
						if (g_bIsValidVector)
						{
							turnOnLED(LED_GREEN);
							g_ui32RequestRobotID = construct4BytesToUint32(&RF24_RX_buffer[1]);
							g_eRobotResponseState = TDOA;
							IntTrigger(INT_SW_TRIGGER_ROBOT_RESPONSE);
						}
						break;

					// EXCHANGE_TABLE state
					case ROBOT_REQUEST_NEIGHBORS_TABLE:
						reloadDelayTimerA();
						checkAndResponeMyNeighborsTableToOneRobot();
						delayTimerB(g_ui8RandomNumber, false);
						break;

					// VOTE_ORIGIN state
					case ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN:
						turnOnLED(LED_RED);
						reloadDelayTimerA();
						updateOrRejectNetworkOrigin(RF24_RX_buffer);
						turnOffLED(LED_RED);
						break;

					// ROTATE_NETWORK state
					case ROBOT_REQUEST_ROTATE_NETWORK:
						getHopOriginTableAndRotate(RF24_RX_buffer);
						break;

					// REDUCE_ERROR state
					case ROBOT_REQUEST_MY_VECTOR:
						tryToResponeNeighborVector();
						delayTimerB(g_ui8RandomNumber, false);
						break;

					case ROBOT_REQUEST_VECTOR_AND_FLAG:
						tryToResponeVectorAndFlag();
						delayTimerB(g_ui8RandomNumber, false);
						break;

					case ROBOT_REQUEST_VECTOR:
						tryToResponseVector();
						delayTimerB(g_ui8RandomNumber, false);
						break;

					// LOCOMOTION state
					case ROBOT_REQUEST_TO_RUN:
						clearRequestNeighbor(RF24_RX_buffer);
						reloadDelayTimerB();
						break;

					case ROBOT_REQUEST_UPDATE_VECTOR:
						updateNeighborVectorInLocsTableByRequest(RF24_RX_buffer);
						break;

					// T shape
					case ROBOT_ALLOW_MOVE_TO_T_SHAPE:
						if (construct4BytesToUint32(&RF24_RX_buffer[1]) == g_ui32RobotID)
							g_bIsAllowToMove = true;
						break;

					case ROBOT_REPONSE_MOVE_COMPLETED:
						g_bIsRobotResponse = true;
						break;

				    // DeBug command
					case PC_SEND_ROTATE_CORRECTION_ANGLE:
						if(g_bIsCounterClockwiseOriented)
							rotateClockwiseWithAngle(0 - g_fRobotOrientedAngle);
						else
							rotateClockwiseWithAngle(g_fRobotOrientedAngle);
						g_fRobotOrientedAngle = 0;
						break;

					case PC_SEND_READ_CORRECTION_ANGLE:
						responseCorrectionAngleAndOriented();
						break;

					case PC_SEND_SET_ROBOT_STATE:
						g_eProcessState = (ProcessState_t)(RF24_RX_buffer[1]);
						break;

					case PC_SEND_ROTATE_CLOCKWISE:
						rotateClockwiseTest(RF24_RX_buffer);
						break;

					case PC_SEND_ROTATE_CLOCKWISE_ANGLE:
						rotateClockwiseAngleTest(RF24_RX_buffer);
						break;

					case PC_SEND_FORWARD_PERIOD:
						forwardPeriodTest(RF24_RX_buffer);
						break;

					case PC_SEND_FORWARD_DISTANCE:
						forwardDistanceTest(RF24_RX_buffer);
						break;

					case PC_SEND_LOCAL_LOOP_STOP:
						g_ui32LocalLoopStop = construct4BytesToUint32(&RF24_RX_buffer[1]);
						break;

					case PC_SEND_SET_STEPSIZE:
						g_fStepSize = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
						break;

					case PC_SEND_SET_STOP_CONDITION_ONE:
						g_fStopCondition = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
						break;

					case PC_SEND_SET_STOP_CONDITION_TWO:
						g_fStopCondition2 = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0;
						break;

					case PC_SEND_MEASURE_DISTANCE:

						turnOffLED(LED_ALL);

						g_eProcessState = MEASURE_DISTANCE;

						for (g_ui8NeighborsCounter = 0;
								g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH;
								g_ui8NeighborsCounter++)
						{
							NeighborsTable[g_ui8NeighborsCounter].ID = 0;
							NeighborsTable[g_ui8NeighborsCounter].distance = 0;
							OneHopNeighborsTable[g_ui8NeighborsCounter].firstHopID = 0;
						}

						Tri_clearLocs(locs, &g_ui8LocsCounter);

						g_ui8NeighborsCounter = 0;

						break;

					case PC_SEND_READ_VECTOR:
						sendVectorToControlBoard();
						break;

					case PC_SEND_READ_NEIGHBORS_TABLE:
						sendNeighborsTableToControlBoard();
						break;

					case PC_SEND_READ_LOCS_TABLE:
						sendLocationsTableToControlBoard();

					case PC_SEND_READ_ONEHOP_TABLE:
						sendOneHopNeighborsTableToControlBoard();
						break;

					case PC_TEST_RF_TRANSMISSION:
						testRfTransmission();
						break;

					case PC_SEND_TEST_DATA_TO_PC:
						sendTestData();
						break;

					case PC_TEST_RF_CARRIER_DETECTION:
						testCarrierDetection();
						break;

					case PC_TOGGLE_ALL_STATUS_LEDS:
						toggleLED(LED_ALL);
						break;

					case PC_CHANGE_MOTORS_SPEED:
						configureMotors((MotorDirection_t)(RF24_RX_buffer[1]), RF24_RX_buffer[2],
								        (MotorDirection_t)(RF24_RX_buffer[3]), RF24_RX_buffer[4]);
						break;

					case PC_SEND_STOP_MOTOR_LEFT:
						stopMotorLeft();
						break;

					case PC_SEND_STOP_MOTOR_RIGHT:
						stopMotorRight();
						break;

					case PC_SEND_DATA_ADC0_TO_PC:
						sendDataToControlBoard((uint8_t *) g_pui16ADC0Result);
						break;

					case PC_SEND_DATA_ADC1_TO_PC:
						sendDataToControlBoard((uint8_t *) g_pui16ADC1Result);
						break;

					case PC_SEND_BATT_VOLT_TO_PC:
						startSamplingBatteryVoltage();
						break;

					case PC_SEND_READ_EEPROM:
						readFormEEPROM();
						break;

					case PC_SEND_WRITE_EEPROM:
						writeToEEPROM();
						break;

					case PC_SEND_SET_ADDRESS_EEPROM:
						setAddressEEPROM();
						break;

					case PC_START_SAMPLING_MIC:
						startSamplingMicSignals();
						break;

					case PC_START_SPEAKER:
						ROM_SysCtlDelay(DELAY_START_SPEAKER);
						startSpeaker();
						break;

					case COMMAND_SLEEP:
						g_eCPUState = SLEEP_MODE;
						IntTrigger(INT_SW_TRIGGER_LPM);
						break;

					case COMMAND_DEEP_SLEEP:
						g_eCPUState = DEEP_SLEEP_MODE;
						IntTrigger(INT_SW_TRIGGER_LPM);
						break;

					case COMMAND_WAKE_UP:
						wakeUpFormLPM();
						break;

					case COMMAND_RESET:
						HWREG(NVIC_APINT) = (NVIC_APINT_VECTKEY |
						NVIC_APINT_SYSRESETREQ);
						break;

					case SMART_PHONE_COMMAND:
						switch (RF24_RX_buffer[1])
						{
						case SP_SEND_STOP_TWO_MOTOR:
							stopMotors();
							break;

						case SP_SEND_FORWAR:
							goStraight();
							break;

						case SP_SEND_SPIN_CLOCKWISE:
							spinClockwise();
							break;

						case SP_SEND_SPIN_COUNTERCLOCKWISE:
							spinCounterclockwise();
							break;

						case SP_SEND_RESERVED:
							goBackward();
							break;

						default:
							break;
						}
						break;

					default:
						break;
					}
					break;
				}
			}
		}
		// Only clear the IRQ if the RF FIFO is empty
		if (RF24_RX_isEmpty())
		{
			RF24_clearIrqFlag(RF24_IRQ_RX);
			GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);
			GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);
		}
	}
}

void RobotResponseIntHandler(void)
{
	switch (g_eRobotResponseState)
	{
		case TDOA:
			responseTDOAResultsToNeighbor(g_ui32RequestRobotID);
			g_eRobotResponseState = DONE;
			break;

		default:
			break;
	}
}

void LowPowerModeIntHandler(void)
{
	switch (g_eCPUState)
	{
	case SLEEP_MODE:
		gotoSleepMode();
		break;
	case DEEP_SLEEP_MODE:
		gotoDeepSleepMode();
		break;
	default:
		break;
	}
}

void DelayTimerAIntHanler()
{
	TimerIntClear(DELAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	g_bDelayTimerAFlagAssert = true;
}

void DelayTimerBIntHanler()
{
	TimerIntClear(DELAY_TIMER_BASE, TIMER_TIMB_TIMEOUT);
	g_bDelayTimerBFlagAssert = true;
}
