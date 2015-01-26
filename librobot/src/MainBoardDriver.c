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
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/eeprom.h"

//#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
//#include "libnrf24l01/inc/nRF24L01.h"
//#include "librobot/inc/MainBoardDriver.h"
//#include "librobot/inc/Trilateration.h"
//#include "librobot/inc/TDOA.h"

uint8_t RF24_RX_buffer[32] =
{ 0 };
uint8_t RF24_TX_buffer[32] =
{ 0 };

uint8_t g_pui8RandomBuffer[8];
uint8_t g_ui8RandomNumber = 0;

uint8_t g_ui8ReTransmitCounter; //NOTE: set this variable to 0 to disable software reTransmit


void parse32BitTo4Bytes(uint32_t value, uint8_t *buffer)
{
	*buffer = value >> 24;
	*(buffer + 1) = value >> 16;
	*(buffer + 2) = value >> 8;
	*(buffer + 3) = value;
}

uint32_t construct4BytesToUint32(uint8_t *buffer)
{
	uint32_t value = 0;
	value = *buffer;
	value = (value << 8) | *(buffer + 1);
	value = (value << 8) | *(buffer + 2);
	value = (value << 8) | *(buffer + 3);

	return value;
}

int32_t construct4BytesToInt32(uint8_t *buffer)
{
	int32_t value = 0;
	value = *buffer;
	value = (value << 8) | *(buffer + 1);
	value = (value << 8) | *(buffer + 2);
	value = (value << 8) | *(buffer + 3);

	return value;
}

void delayRandom(uint32_t parameterUnit)
{
	generateRandomByte();
	while (g_ui8RandomNumber == 0);
	ROM_SysCtlDelay(g_ui8RandomNumber * parameterUnit);
}


bool g_bDelayTimerAFlagAssert;
bool g_bDelayTimerBFlagAssert;

static uint32_t timerALastDelayPeriod;
static uint32_t timerBLastDelayPeriod;

void initTimerDelay()
{
	// Interrupt timers
	SysCtlPeripheralEnable(DELAY_TIMER_CLOCK);
	TimerClockSourceSet(DELAY_TIMER_BASE, TIMER_CLOCK_SYSTEM);
	TimerConfigure(DELAY_TIMER_BASE,
	TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);

	TimerIntEnable(DELAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(DELAY_TIMER_BASE, TIMER_TIMB_TIMEOUT);

	IntPrioritySet(INT_DELAY_TIMERA, PRIORITY_DELAY_TIMERA);
	IntPrioritySet(INT_DELAY_TIMERB, PRIORITY_DELAY_TIMERB);

	IntEnable(INT_DELAY_TIMERA);
	IntEnable(INT_DELAY_TIMERB);

	TimerIntClear(DELAY_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntClear(DELAY_TIMER_BASE, TIMER_TIMB_TIMEOUT);

	// Non-interrupt timer
	SysCtlPeripheralEnable(DELAY_TIMER_CLOCK_NON_INT);
	TimerClockSourceSet(DELAY_TIMER_BASE_NON_INT, TIMER_CLOCK_SYSTEM);
	TimerConfigure(DELAY_TIMER_BASE_NON_INT, TIMER_CFG_ONE_SHOT);

	TimerIntEnable(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);
}

void delayTimerA(uint32_t period, bool isSynchronous)
{
	timerALastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, timerALastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_A);

	g_bDelayTimerAFlagAssert = false;

	if (isSynchronous)
		while (!g_bDelayTimerAFlagAssert)
			;
}

void reloadDelayTimerA()
{
	TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, timerALastDelayPeriod);
}

void delayTimerB(uint32_t period, bool isSynchronous)
{
	timerBLastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_B);

	g_bDelayTimerBFlagAssert = false;

	if (isSynchronous)
		while (!g_bDelayTimerBFlagAssert)
			;
}

void reloadDelayTimerB()
{
	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);
}

void delayTimerNonInt(uint32_t period)
{
	uint32_t ui32Status;
	uint32_t delayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE_NON_INT, TIMER_A, delayPeriod);

	TimerEnable(DELAY_TIMER_BASE_NON_INT, TIMER_A);

	while(1)
	{
		ui32Status = TimerIntStatus(DELAY_TIMER_BASE_NON_INT, false);

		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;
	}

	TimerIntClear(DELAY_TIMER_BASE_NON_INT, TIMER_TIMA_TIMEOUT);
}


//----------------Robot Init functions-------------------

robotMeas_t NeighborsTable[NEIGHBOR_TABLE_LENGTH];
oneHopMeas_t OneHopNeighborsTable[ONEHOP_NEIGHBOR_TABLE_LENGTH];
location_t locs[LOCATIONS_TABLE_LENGTH];

uint32_t g_ui8LocsCounter;

uint32_t g_ui32OriginID;
uint32_t g_ui32RotationHopID;
uint8_t g_ui8Hopth;
uint8_t g_ui8OriginNumberOfNeighbors;

uint8_t g_ui8NeighborsCounter;

uint32_t g_ui32RobotID;
vector2_t g_vector;
bool g_bIsValidVector;
bool g_bIsCounterClockwiseOriented;
float g_fRobotOrientedAngle;

uint32_t g_ui32RequestRobotID;

bool g_bIsNetworkRotated;
bool g_bIsActiveCoordinatesFixing;
bool g_bIsGradientSearchStop;
uint32_t g_ui32LocalLoop;

ProcessState_t g_eProcessState;
RobotResponseState_t g_eRobotResponseState;

extern location_t locs[];
extern bool g_bIsNewTDOAResults;
extern float32_t g_f32PeakEnvelopeA;
extern float32_t g_f32PeakEnvelopeB;

extern float g_fStepSize;
extern float g_fStopCondition;
extern float g_fStopCondition2;

bool g_bBypassThisState;
uint8_t g_ui8ReBroadcastCounter;

float g_f32Intercept = 8.5619f;
float g_f32Slope = 3.046f;

extern uint32_t g_ui32LocalLoopStop; // Debug Only

void debugBreakpoint()
{
	g_ui32LocalLoopStop = 1;
	while(g_ui32LocalLoopStop != 0);
}


void initRobotProcess()
{
	uint32_t temp;

	//==============================================
	// Get Robot ID form EEPROM
	//==============================================
	EEPROMRead(&g_ui32RobotID, EEPROM_ADDR_ROBOT_ID, sizeof(&g_ui32RobotID));

	EEPROMRead(&temp, EEPROM_INTERCEPT, sizeof(&temp));
	g_f32Intercept = temp / 65536.0;

	EEPROMRead(&temp, EEPROM_SLOPE, sizeof(&temp));
	g_f32Slope = temp / 65536.0;

	if (g_f32Intercept == 0xFFFFFFFF || g_f32Slope == 0xFFFFFFFF)
	{
		while(1);
//		g_f32Intercept = 8.5619f;
//		g_f32Slope = 3.046f;
	}

	g_eProcessState = IDLE;
	g_vector.x = (g_ui32RobotID & 0xFF);
	g_vector.y = (g_ui32RobotID & 0xFF);
	g_bIsValidVector = false;
	g_bIsNetworkRotated = false;
	g_bIsActiveCoordinatesFixing = false;
	g_bIsGradientSearchStop = false;
	g_ui32LocalLoop = 0;

	g_fRobotOrientedAngle = 0;
	g_bIsCounterClockwiseOriented = 0;

	//==============================================
	// IMPORTANCE: Configure Software Interrupt
	//==============================================
	IntPrioritySet(INT_SW_TRIGGER_ROBOT_RESPONSE, PRIORITY_ROBOT_RESPONSE);

	IntEnable(INT_SW_TRIGGER_ROBOT_RESPONSE);
}

void addToNeighborTable(uint32_t neighborId, uint16_t distance)
{
	NeighborsTable[g_ui8NeighborsCounter].ID = neighborId;
	NeighborsTable[g_ui8NeighborsCounter].distance = distance;

	g_ui8NeighborsCounter++;

	//TODO: search the worth result and replace it by the current result if better
	g_ui8NeighborsCounter =
			(g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH) ?
					(g_ui8NeighborsCounter) :
					(NEIGHBOR_TABLE_LENGTH);
}

void responseTDOAResultsToNeighbor(uint32_t neighborId)
{
	float32_t f32_inputValue;
	float32_t f32_outputValue;
	uint16_t neighborDistance;
	uint16_t ui16DelayPeriod;

	int32_t i32TempAxix;

	while(!g_bIsNewTDOAResults);

	g_bIsNewTDOAResults = false;

	g_f32PeakEnvelopeA = (g_f32PeakEnvelopeA - g_f32Intercept) / g_f32Slope;
	g_f32PeakEnvelopeB = (g_f32PeakEnvelopeB - g_f32Intercept) / g_f32Slope;

	f32_inputValue = (((g_f32PeakEnvelopeA * g_f32PeakEnvelopeA
						+ g_f32PeakEnvelopeB
								* g_f32PeakEnvelopeB) / 2.0)
						- (DISTANCE_BETWEEN_TWO_MICS_SQR / 4.0))
						* 65536.0; // * 256^2

	f32_outputValue = vsqrtf(f32_inputValue);

	neighborDistance = (uint16_t) (f32_outputValue + 0.5);

	addToNeighborTable(neighborId, neighborDistance);

	generateRandomByte();
	while (g_ui8RandomNumber == 0);
	g_ui8RandomNumber = (g_ui8RandomNumber < 20) ? (g_ui8RandomNumber + 20) : (g_ui8RandomNumber);

	ui16DelayPeriod = g_ui8RandomNumber;
	delayTimerNonInt(ui16DelayPeriod);

	RF24_TX_buffer[0] = ROBOT_RESPONSE_TDOA_DISTANCE;
	parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4

	i32TempAxix = g_vector.x * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[5]); // 5->8

	i32TempAxix = g_vector.y * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[9]); // 9->12

	RF24_TX_buffer[13] = (uint8_t)(neighborDistance >> 8);
	RF24_TX_buffer[14] = (neighborDistance & 0xFF);

	sendMessageToOneNeighbor(neighborId, RF24_TX_buffer, 15);

	turnOffLED(LED_GREEN);
}

void storeNeighorVectorAndDistanceToTables(uint8_t RxData[])
{
	location_t neighbor;
	int32_t i32TempAxix;
	uint16_t neighborDistance;

	neighbor.ID = construct4BytesToUint32(&RxData[1]);

	i32TempAxix = construct4BytesToInt32(&RxData[5]);
	neighbor.vector.x = i32TempAxix / 65536.0;

	i32TempAxix = construct4BytesToInt32(&RxData[9]);
	neighbor.vector.y = i32TempAxix / 65536.0;

	neighborDistance = (RxData[13] << 8) | RxData[14];

	// Add to neighbor table
	NeighborsTable[g_ui8NeighborsCounter].ID = neighbor.ID;
	NeighborsTable[g_ui8NeighborsCounter].distance = neighborDistance;
	g_ui8NeighborsCounter++;

	//TODO: search the worth result and replace it by the current result if better
	g_ui8NeighborsCounter =
			(g_ui8NeighborsCounter < NEIGHBOR_TABLE_LENGTH) ?
					(g_ui8NeighborsCounter) : (NEIGHBOR_TABLE_LENGTH);

	// Add to locs table
	Tri_addLocation(neighbor.ID, neighbor.vector.x, neighbor.vector.y);
}

void checkAndResponeMyNeighborsTableToOneRobot()
{
	uint32_t neighborID = 0;
	uint32_t responseLength = 0;
	uint32_t tableSizeInByte = 0;
	uint32_t randomRfChannel = 0;

	neighborID = RF24_RX_buffer[1];
	neighborID = (neighborID << 8) + RF24_RX_buffer[2];
	neighborID = (neighborID << 8) + RF24_RX_buffer[3];
	neighborID = (neighborID << 8) + RF24_RX_buffer[4];

	randomRfChannel = RF24_RX_buffer[5];

	rfDelayLoop(DELAY_CYCLES_1MS5);

	RF24_setChannel(randomRfChannel);

	turnOffLED(LED_RED);

	int i;
	for (i = 0; i < NEIGHBOR_TABLE_LENGTH; i++)
	{
		if (NeighborsTable[i].ID == neighborID)
		{
			RF24_TX_buffer[0] = ROBOT_RESPONSE_HELLO_NEIGHBOR;

			RF24_TX_buffer[1] = g_ui32RobotID >> 24;
			RF24_TX_buffer[2] = g_ui32RobotID >> 16;
			RF24_TX_buffer[3] = g_ui32RobotID >> 8;
			RF24_TX_buffer[4] = g_ui32RobotID;

			tableSizeInByte = sizeof(robotMeas_t) * g_ui8NeighborsCounter;

			RF24_TX_buffer[5] = tableSizeInByte >> 24;
			RF24_TX_buffer[6] = tableSizeInByte >> 16;
			RF24_TX_buffer[7] = tableSizeInByte >> 8;
			RF24_TX_buffer[8] = tableSizeInByte;

			responseLength = 9;
			break;
		}
	}

	if (RF24_TX_buffer[0] != ROBOT_RESPONSE_HELLO_NEIGHBOR)
	{
		RF24_TX_buffer[0] = ROBOT_RESPONSE_NOT_YOUR_NEIGHBOR;
		responseLength = 1;
	}

	if (sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, responseLength))
	{
		//ROM_SysCtlDelay(10000); // delay 600us
		sendMessageToOneNeighbor(neighborID, (uint8_t*) NeighborsTable,
				tableSizeInByte);
	}

	RF24_RX_flush();
	RF24_setChannel(0);

	turnOnLED(LED_RED);
}

void getNeighborNeighborsTable()
{
	uint32_t neighborID = 0;
	uint32_t dataLength = 0;
	uint32_t length = 0;
	uint32_t writeTablePosition = 0;
	uint32_t pointer = 0;
	uint8_t i = 0;
	bool isReceivedData = false;

	disableRF24Interrupt();

	delayTimerB(DELAY_GET_TABLE_PERIOD, false);

	while (!g_bDelayTimerBFlagAssert)
	{
		reloadDelayTimerA();

		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			reloadDelayTimerB();

			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				if (!isReceivedData)
				{
					if (RF24_RX_buffer[0] == ROBOT_RESPONSE_HELLO_NEIGHBOR
							&& length == 9)
					{
						neighborID = RF24_RX_buffer[1];
						neighborID = (neighborID << 8) | RF24_RX_buffer[2];
						neighborID = (neighborID << 8) | RF24_RX_buffer[3];
						neighborID = (neighborID << 8) | RF24_RX_buffer[4];

						dataLength = RF24_RX_buffer[5];
						dataLength = (dataLength << 8) | RF24_RX_buffer[6];
						dataLength = (dataLength << 8) | RF24_RX_buffer[7];
						dataLength = (dataLength << 8) | RF24_RX_buffer[8];

						for (writeTablePosition = 0;
								writeTablePosition
										< ONEHOP_NEIGHBOR_TABLE_LENGTH;
								writeTablePosition++)
						{
							if (OneHopNeighborsTable[writeTablePosition].firstHopID
									== 0)
							{
								OneHopNeighborsTable[writeTablePosition].firstHopID =
										neighborID;
								pointer = 0;
								isReceivedData = true;
								break;
							}
						}
						if (writeTablePosition == ONEHOP_NEIGHBOR_TABLE_LENGTH)
						{
							// out of range, table haven't erased
							break;
						}
					}
					else // if (RF24_RX_buffer[0] == ROBOT_RESPONSE_NOT_YOUR_NEIGHBOR)
					{
						break;
					}
				}
				else
				{
					for (i = 0; i < length; i++)
					{
						*(((uint8_t*) OneHopNeighborsTable[writeTablePosition].neighbors)
								+ pointer) = RF24_RX_buffer[i];
						pointer++;
					}
					if (dataLength > length)
						dataLength = dataLength - length;
					else
					{
						break;
					}
				}
			}
		}
	}
	enableRF24Interrupt();
}

void sendVectorToControlBoard()
{
	int32_t i32TempAxix;
	i32TempAxix = g_vector.x * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[0]); // 0->3

	i32TempAxix = g_vector.y * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[4]); // 4->7

	sendDataToControlBoard(RF24_TX_buffer);
}

void sendNeighborsTableToControlBoard()
{
	uint8_t dataBuffer[60] = { 0 }; // 60 (sizeof(robotMeas_t) * 10)
	uint8_t length = sizeof(dataBuffer);

	uint8_t neighborPointer = 0;
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		parse32BitTo4Bytes(NeighborsTable[neighborPointer].ID, &dataBuffer[i]); // i->i+4
		i += 4;

		dataBuffer[i] = NeighborsTable[neighborPointer].distance >> 8;
		i++;

		dataBuffer[i] = NeighborsTable[neighborPointer].distance;

		neighborPointer++;
		if (neighborPointer > NEIGHBOR_TABLE_LENGTH)
			break;
	}
	sendDataToControlBoard(dataBuffer);
}

void sendLocationsTableToControlBoard()
{
	uint8_t dataBuffer[120] = { 0 }; // 120 (sizeof(location_t) * 10)
	uint8_t length = sizeof(dataBuffer);
	int32_t tempAxis;
	uint8_t neighborPointer = 0;
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		parse32BitTo4Bytes(locs[neighborPointer].ID, &dataBuffer[i]); // i->i+4
		i += 4;

		tempAxis = (int32_t)(locs[neighborPointer].vector.x * 65536 + 0.5);
		parse32BitTo4Bytes(tempAxis, &dataBuffer[i]); // i->i+4
		i += 4;

		tempAxis = (int32_t)(locs[neighborPointer].vector.y * 65536 + 0.5);
		parse32BitTo4Bytes(tempAxis, &dataBuffer[i]); // i->i+4
		i += 3;

		neighborPointer++;
		if (neighborPointer > LOCATIONS_TABLE_LENGTH)
			break;
	}
	sendDataToControlBoard(dataBuffer);
}

void sendOneHopNeighborsTableToControlBoard()
{
	uint8_t dataBuffer[640] = { 0 }; // 640 (sizeof(oneHopMeas_t) * 10)
	uint32_t length = sizeof(dataBuffer);

	uint8_t neighborPointer = 0;
	uint8_t oneHopNeighborPointer = 0;

	uint32_t i;
	for(i = 0; i < length; i++)
	{
		parse32BitTo4Bytes(OneHopNeighborsTable[neighborPointer].firstHopID, &dataBuffer[i]); // i->i+4
		i += 4;

		for(oneHopNeighborPointer = 0; oneHopNeighborPointer < NEIGHBOR_TABLE_LENGTH; oneHopNeighborPointer++)
		{
			parse32BitTo4Bytes(OneHopNeighborsTable[neighborPointer].neighbors[oneHopNeighborPointer].ID, &dataBuffer[i]); // i->i+4
			i += 4;

			dataBuffer[i] = OneHopNeighborsTable[neighborPointer].neighbors[oneHopNeighborPointer].distance >> 8;
			i++;

			dataBuffer[i] = OneHopNeighborsTable[neighborPointer].neighbors[oneHopNeighborPointer].distance;
			i++;
		}

		i--;

		neighborPointer++;
		if (neighborPointer > NEIGHBOR_TABLE_LENGTH)
			break;
	}
	sendDataToControlBoard(dataBuffer);
}

void rotateClockwiseTest(uint8_t RxData[])
{
	uint32_t ui32DelayPeriod = construct4BytesToUint32(&RxData[1]);

	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(REVERSE, i8Motor1Speed + 5, FORWARD, i8Motor2Speed + 5);
		delayTimerNonInt(ui32DelayPeriod);
		stopMotors();
	}
}

void rotateClockwiseAngleTest(uint8_t RxData[])
{
	int16_t angleInDegree = (RxData[1] << 8) | RxData[2];
	float angleInRadian = angleInDegree / 180.0 * MATH_PI;
	rotateClockwiseWithAngle(angleInRadian);
}

void forwardPeriodTest(uint8_t RxData[])
{
	uint32_t ui32DelayPeriod = construct4BytesToUint32(&RxData[1]);

	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(FORWARD, i8Motor1Speed + 5, FORWARD, i8Motor2Speed + 5);
		delayTimerNonInt(ui32DelayPeriod);
		stopMotors();
	}
}

void forwardDistanceTest(uint8_t RxData[])
{
	int32_t i32Distance = construct4BytesToInt32(&RxData[1]);

	float distance = i32Distance / 65536.0;

	runForwardWithDistance(distance);
}

void responseCorrectionAngleAndOriented()
{
	int32_t i32CorrectionAngle;

	i32CorrectionAngle = g_fRobotOrientedAngle * 65536 + 0.5;
	parse32BitTo4Bytes(i32CorrectionAngle, &RF24_TX_buffer[0]); // 0->3

	RF24_TX_buffer[4] = (g_bIsCounterClockwiseOriented) ? (0x01) : (0x00);

	sendDataToControlBoard(RF24_TX_buffer);
}

void updateOrRejectNetworkOrigin(uint8_t RxData[])
{
	//RxData:: <Update Network Origin COMMAND> <thisOriginID> <thisOriginNumberOfNeighbors> <thisHopth>
	uint32_t ui32OriginNodeID;
	uint8_t ui8OriginNumberOfNeighbors;
	uint8_t ui8Hopth;

	ui32OriginNodeID = (RxData[1] << 24) | (RxData[2] << 16) | (RxData[3] << 8) | RxData[4];

	if (ui32OriginNodeID == g_ui32OriginID)
		return;

	ui8OriginNumberOfNeighbors = RxData[5];
	ui8Hopth = RxData[6];

	if (isNeedRotateCoordinate(ui8OriginNumberOfNeighbors, ui32OriginNodeID))
	{
		g_ui8Hopth = ui8Hopth + 1;

		g_ui32OriginID = ui32OriginNodeID;

		g_ui8OriginNumberOfNeighbors = ui8OriginNumberOfNeighbors;
	}

	g_ui8ReBroadcastCounter = 0;

	g_bBypassThisState = false;
}

bool isNeedRotateCoordinate(uint8_t originNumberOfNeighbors, uint32_t originID)
{
	if (g_ui8OriginNumberOfNeighbors == originNumberOfNeighbors)
	{
		if (originID <  g_ui32OriginID)
			g_ui32OriginID = originID;

		return true;
	}
	else if (g_ui8OriginNumberOfNeighbors < originNumberOfNeighbors)
	{
		return true;
	}

	return false;
}

void getHopOriginTableAndRotate(uint8_t RxData[])
{
	uint8_t ui8RandomRfChannel;
	location_t oriLocs[LOCATIONS_TABLE_LENGTH];
	uint8_t oriLocsCounter;

	uint32_t dataLength;
	uint32_t length;
	uint32_t pointer;
	uint8_t i;
	uint32_t robotJ_ID;

	vector2_t RotationHopVector;

	float alphaJ;
	float alphaK;
	float betaJ;
	float betaI;
	float alphaJK;
	float betaJI;
	bool isNeedMirroring;
	float correctionAngle = 0;
	int8_t tempValue;

	g_ui32RotationHopID = (RxData[1] << 24) | (RxData[2] << 16) | (RxData[3] << 8) | RxData[4];

	dataLength = (RxData[5] << 24) | (RxData[6] << 16) | (RxData[7] << 8) | RxData[8];

	tempValue = (RxData[9] << 24) | (RxData[10] << 16) | (RxData[11] << 8) | RxData[12];
	RotationHopVector.x = (float) (tempValue / 65536.0);

	tempValue = (RxData[13] << 24) | (RxData[14] << 16) | (RxData[15] << 8) | RxData[16];
	RotationHopVector.y = (float) (tempValue / 65536.0);

	ui8RandomRfChannel = RxData[17];
	RF24_setChannel(ui8RandomRfChannel);

	oriLocsCounter = dataLength / sizeof(location_t);

	disableRF24Interrupt();

	RF24_clearIrqFlag(RF24_IRQ_RX);

	GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);

	pointer = 0;

	while (1)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				for (i = 0; i < length; i++)
				{
					*( ((uint8_t*)oriLocs) + pointer ) = RF24_RX_buffer[i];
					pointer++;
				}
				if (dataLength > length)
					dataLength = dataLength - length;
				else
					break;
			}
		}
	}

	RF24_RX_flush();
	RF24_setChannel(0);

	if(!g_bIsNetworkRotated)
	{
		if((isLocationTableContainID(g_ui32RobotID, oriLocs, oriLocsCounter) != -1)
				&& (isLocationTableContainID(g_ui32RotationHopID, locs, g_ui8LocsCounter) != -1))
		{
			robotJ_ID = 0;

			robotJ_ID = tryToGetCommonNeighborID(locs, g_ui8LocsCounter, oriLocs, oriLocsCounter);
			if (robotJ_ID != 0)
			{
				alphaJ = getAngleFromTable(robotJ_ID, oriLocs, oriLocsCounter);
				alphaK = getAngleFromTable(g_ui32RobotID, oriLocs, oriLocsCounter);

				alphaJK = alphaJ - alphaK;
				alphaJK = (alphaJK < 0) ? (360 + alphaJK) : (alphaJK);

				betaJ = getAngleFromTable(robotJ_ID, locs, g_ui8LocsCounter);
				betaI = getAngleFromTable(g_ui32RotationHopID, locs, g_ui8LocsCounter);

				betaJI = betaJ - betaI;
				betaJI = (betaJI < 0) ? (360 + betaJI) : (betaJI);

				if ((alphaJK < MATH_PI && betaJI > MATH_PI)
						|| (alphaJK > MATH_PI && betaJI < MATH_PI))
				{
					isNeedMirroring = false;
					correctionAngle = betaI - alphaK + MATH_PI;
				}

				if ((alphaJK < MATH_PI && betaJI < MATH_PI)
						|| (alphaJK > MATH_PI && betaJI > MATH_PI))
				{
					isNeedMirroring = true;
					correctionAngle = betaI + alphaK;
				}

				rotateLocationTable(correctionAngle, isNeedMirroring, locs, g_ui8LocsCounter);

				calculateRealVector(RotationHopVector, oriLocs, oriLocsCounter);

				g_bIsNetworkRotated = true;
			}
		}
	}
	else
	{
		// Do nothing!
	}

	enableRF24Interrupt();
}

void calculateRealVector(vector2_t vector, location_t table[], uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		if (table[i].ID == g_ui32RobotID)
			break;
	}
	g_vector.x = vector.x + table[i].vector.x;
	g_vector.y = vector.y + table[i].vector.y;
}

int32_t isLocationTableContainID(uint32_t id, location_t table[], uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		if (table[i].ID == id)
			return i;
	}
	return -1;
}

uint32_t tryToGetCommonNeighborID(location_t firstTable[], uint8_t firstTableLength, location_t secondTable[], uint8_t secondTableLength)
{
	uint8_t firstPointer;
	uint8_t secondPointer;

	uint32_t selectID;

	for(firstPointer = 0; firstPointer < firstTableLength; firstPointer++)
	{
		selectID = firstTable[firstPointer].ID;

		for(secondPointer = 0; secondPointer < secondTableLength; secondPointer++)
		{
			if (selectID == secondTable[secondPointer].ID
			    && selectID != g_ui32RobotID
				&& selectID != g_ui32RotationHopID)
			{
				return selectID;
			}
		}
	}
	return 0;
}

float getAngleFromTable(uint32_t id, location_t table[], uint8_t length)
{
	// WARNING! id must existed in table[]
	uint8_t i;
	float distance;

	for(i = 0; i < length; i++)
	{
		if(table[i].ID == id)
		{
			distance = vsqrtf(table[i].vector.x * table[i].vector.x
					+ table[i].vector.y * table[i].vector.y);

			return calASin(table[i].vector.y / distance);
		}
	}

	return 0;
}

void rotateLocationTable(float angle, bool mirrorXaxis, location_t table[], uint8_t length)
{
	uint8_t i;
	float x;
	float y;
	float angleOffset;

	angleOffset = MATH_PI_MUL_2 - angle;

	for(i = 0; i < length; i++)
	{
		x = table[i].vector.x;
		y = table[i].vector.y;

		table[i].vector.x = x * calCos(angleOffset) - y * calSin(angleOffset);
		table[i].vector.y = x * calSin(angleOffset) + y * calCos(angleOffset);

		if (mirrorXaxis)
			table[i].vector.x *= -1;
	}
}

void updateGradient(vector2_t *pvectGradienNew, bool enableRandomCalculation)
{
	int i;
	uint32_t ui32Distance;
	float fVectorNorm;
	float fTemplateParameter;
	vector2_t distanceVector;

	// Gradian Update
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32RobotID)
			continue;

		if (enableRandomCalculation)
		{
			generateRandomByte();
			while (g_ui8RandomNumber == 0);
			if (g_ui8RandomNumber & 0x01)
				continue;
		}

		ui32Distance = Tri_tryToGetDistance(OneHopNeighborsTable, locs[i].ID, g_ui32RobotID);
		if (ui32Distance > 0)
			ui32Distance = (ui32Distance + Tri_tryToGetNeighborsDistance(NeighborsTable, locs[i].ID)) / 2;
		else
			ui32Distance = Tri_tryToGetNeighborsDistance(NeighborsTable, locs[i].ID);

		distanceVector.x = g_vector.x - locs[i].vector.x;
		distanceVector.y = g_vector.y - locs[i].vector.y;

		if (distanceVector.x == 0 && distanceVector.y == 0)
		{
			distanceVector.x = 1;
			distanceVector.y = 1;
		}

		fVectorNorm = vsqrtf(distanceVector.x * distanceVector.x + distanceVector.y * distanceVector.y);

		fTemplateParameter = (fVectorNorm - (float)(ui32Distance / 256.0f)) / fVectorNorm;

		pvectGradienNew->x += distanceVector.x * fTemplateParameter;
		pvectGradienNew->y += distanceVector.y * fTemplateParameter;
	}
}

bool checkVarianceCondition(vector2_t vectNew, vector2_t vectOld, float fCondition)
{
	vector2_t vectVariance;

	vectVariance.x = vectNew.x - vectOld.x;
	vectVariance.y = vectNew.y - vectOld.y;

	vectVariance.x = (vectVariance.x < 0) ? (-vectVariance.x) : (vectVariance.x);
	vectVariance.y = (vectVariance.y < 0) ? (-vectVariance.y) : (vectVariance.y);

	return (vectVariance.x < fCondition) && (vectVariance.y < fCondition);
}

void updatePosition(vector2_t *pvectAverageCoordination, vector2_t *pvectEstimatePosNew, vector2_t *pvectEstimatePosOld, vector2_t *pvectGradienNew, vector2_t *pvectGradienOld, float fStepSize)
{
	pvectEstimatePosOld->x = pvectEstimatePosNew->x;
	pvectEstimatePosOld->y = pvectEstimatePosNew->y;

	pvectEstimatePosNew->x = pvectEstimatePosOld->x - fStepSize * pvectGradienNew->x;
	pvectEstimatePosNew->y = pvectEstimatePosOld->y - fStepSize * pvectGradienNew->y;

	pvectAverageCoordination->x = pvectEstimatePosNew->x;
	pvectAverageCoordination->y = pvectEstimatePosNew->y;

	pvectGradienOld->x = pvectGradienNew->x;
	pvectGradienOld->y = pvectGradienNew->y;

	pvectGradienNew->x = 0;
	pvectGradienNew->y = 0;
}

void updateLocsByOtherRobotCurrentPosition(bool isFirstInit)
{
	uint8_t i;
	uint8_t ui8RandomRfChannel;
	uint16_t ui16RandomValue;
	bool isSuccess;
	bool isNeighborActive;

	int8_t i8Position;
	vector2_t vectReceived;
	bool bIsNeighborGradientSearchStop;

	for(i = 0; i < g_ui8NeighborsCounter; i++)
	{
		if (NeighborsTable[i].ID == g_ui32RobotID)
		{
			continue;
		}
		if (NeighborsTable[i].ID == g_ui32OriginID)
		{
			i8Position = isLocationTableContainID(g_ui32OriginID, locs, g_ui8LocsCounter);
			if (i8Position != -1)
			{
				locs[i8Position].vector.x = 0;
				locs[i8Position].vector.y = 0;
			}
			continue;
		}

		g_ui8ReTransmitCounter = 1; // set this variable to 0 to disable software reTransmit, reTransmit times = (255 - g_ui8ReTransmitCounter)

		isSuccess = false;
		isNeighborActive = false;

		while (1) // wait for neighbor g_ui32LocalLoop < this.g_ui32LocalLoop && neighbor g_bIsActiveCoordinatesFixing == true
		{
			generateRandomByte();
			while (g_ui8RandomNumber == 0);

			ui8RandomRfChannel = (g_ui8RandomNumber % 125) + 1; // only allow channel range form 1 to 125

			g_ui8RandomNumber =
					(g_ui8RandomNumber < 100) ?
							(g_ui8RandomNumber + 100) :
							(g_ui8RandomNumber);

			ui16RandomValue = g_ui8RandomNumber * 10;

			delayTimerB(ui16RandomValue, true); // maybe Received ROBOT_REQUEST_VECTOR_AND_FLAG command here!

			while (!g_bDelayTimerBFlagAssert)
				; // this line make sure robot will re delay after response to another robot

			// delay timeout
			RF24_TX_buffer[0] = ROBOT_REQUEST_VECTOR_AND_FLAG;
			parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
			RF24_TX_buffer[5] = ui8RandomRfChannel;
			parse32BitTo4Bytes(g_ui32LocalLoop, &RF24_TX_buffer[6]); // 6->9

			// send request neighbor send there g_vector coordinates: <x>, <y>
			if (sendMessageToOneNeighbor(NeighborsTable[i].ID, RF24_TX_buffer, 10))
			{
				turnOffLED(LED_RED);

				RF24_setChannel(ui8RandomRfChannel);
				RF24_TX_flush();
				RF24_clearIrqFlag(RF24_IRQ_MASK);
				RF24_RX_activate();

				isSuccess = getNeighborVectorAndFlag(&vectReceived, &bIsNeighborGradientSearchStop, &isNeighborActive);

				RF24_setChannel(0);
				RF24_TX_flush();
				RF24_clearIrqFlag(RF24_IRQ_MASK);
				RF24_RX_activate();

				turnOnLED(LED_RED);

				if (isSuccess)
				{
					if (vectReceived.x == 0 || vectReceived.y == 0)
					{
						turnOffLED(LED_ALL);
						while(1)
						{
							rfDelayLoop(DELAY_CYCLES_5MS * 10);
							toggleLED(LED_ALL);
						}
					}
					break;
				}
			}
			else if (g_ui8ReTransmitCounter == 0)
				break;
		}

		if (isNeighborActive)
		{
			i8Position = isLocationTableContainID(NeighborsTable[i].ID, locs, g_ui8LocsCounter);

			if (i8Position == -1)
			{
				Tri_addLocation(NeighborsTable[i].ID, vectReceived.x, vectReceived.y);
			}
			else
			{
				locs[i8Position].vector.x = vectReceived.x;
				locs[i8Position].vector.y = vectReceived.y;
			}

			if (!isFirstInit)
			{
				g_bIsGradientSearchStop &= bIsNeighborGradientSearchStop;
			}
		}
	}
}

void synchronousLocsTableAndMyVector()
{
	uint8_t i;
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32RobotID)
		{
			locs[i].vector.x = g_vector.x;
			locs[i].vector.y = g_vector.y;
			return;
		}
	}
}

void tryToResponeNeighborVector()
{
	int32_t i32TempAxix;
	uint8_t i;

	uint32_t neighborID = 0;
	uint32_t responseLength = 0;
	uint32_t randomRfChannel = 0;

	neighborID = construct4BytesToUint32(&RF24_RX_buffer[1]);

	randomRfChannel = RF24_RX_buffer[5];

	rfDelayLoop(DELAY_CYCLES_1MS5);

	RF24_setChannel(randomRfChannel);

	turnOffLED(LED_RED);

	if (g_bIsNetworkRotated)
	{
		for(i = 0; i < g_ui8LocsCounter; i++)
		{
			if (locs[i].ID == neighborID)
			{
				RF24_TX_buffer[0] = ROBOT_RESPONSE_MY_VECTOR;

				i32TempAxix = (int32_t)(locs[i].vector.x * 65536 + 0.5);
				parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[1]); // 1->4

				i32TempAxix = (int32_t)(locs[i].vector.y * 65536 + 0.5);
				parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[5]); // 5->8

				responseLength = 9;
				break;
			}
		}

		if (RF24_TX_buffer[0] != ROBOT_RESPONSE_MY_VECTOR)
		{
			RF24_TX_buffer[0] = ROBOT_RESPONSE_MY_VECTOR_NOT_FOUND;
			responseLength = 1;
		}
	}
	else
	{
		RF24_TX_buffer[0] = ROBOT_RESPONSE_MY_VECTOR_PLEASE_WAIT;
		responseLength = 1;
	}

	sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, responseLength);

	RF24_RX_flush();
	RF24_setChannel(0);

	turnOnLED(LED_RED);
}

void tryToResponeVectorAndFlag()
{
	int32_t i32TempAxix;

	uint32_t neighborID = 0;
	uint32_t responseLength = 0;
	uint32_t randomRfChannel = 0;
	uint32_t neighborLoopCounter = 0;

	neighborID = construct4BytesToUint32(&RF24_RX_buffer[1]);

	randomRfChannel = RF24_RX_buffer[5];

	neighborLoopCounter = construct4BytesToUint32(&RF24_RX_buffer[6]);

	rfDelayLoop(DELAY_CYCLES_1MS5);

	RF24_setChannel(randomRfChannel);

	turnOffLED(LED_RED);

	if (g_ui32LocalLoop >= neighborLoopCounter && g_bIsActiveCoordinatesFixing)
	{
		RF24_TX_buffer[0] = ROBOT_RESPONSE_VECTOR_AND_FLAG;

		i32TempAxix = g_vector.x * 65536 + 0.5;
		parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[1]); // 1->4

		i32TempAxix = g_vector.y * 65536 + 0.5;
		parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[5]); // 5->8

		RF24_TX_buffer[9] = (g_bIsGradientSearchStop) ? (0x01): (0x00);

		responseLength = 10;
	}
	else
	{
		if (g_bIsActiveCoordinatesFixing)
			RF24_TX_buffer[0] = ROBOT_RESPONSE_PLEASE_WAIT;
		else
			RF24_TX_buffer[0] = ROBOT_RESPONSE_UNACTIVE;

		responseLength = 1;
	}

	sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, responseLength);

	RF24_RX_flush();
	RF24_setChannel(0);

	turnOnLED(LED_RED);
}

void tryToResponseVector()
{
	int32_t i32TempAxix;

	uint32_t neighborID = 0;
	uint32_t responseLength = 0;
	uint32_t randomRfChannel = 0;

	neighborID = construct4BytesToUint32(&RF24_RX_buffer[1]);

	randomRfChannel = RF24_RX_buffer[5];

	rfDelayLoop(DELAY_CYCLES_1MS5);

	RF24_setChannel(randomRfChannel);

	turnOffLED(LED_RED);

	RF24_TX_buffer[0] = ROBOT_RESPONSE_VECTOR;

	i32TempAxix = g_vector.x * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[1]); // 1->4

	i32TempAxix = g_vector.y * 65536 + 0.5;
	parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[5]); // 5->8

	responseLength = 9;

	sendMessageToOneNeighbor(neighborID, RF24_TX_buffer, responseLength);

	RF24_RX_flush();
	RF24_setChannel(0);

	turnOnLED(LED_RED);
}

bool getMyVector(uint8_t *pCounter)
{
	bool returnState = true;
	uint32_t length = 0;

	disableRF24Interrupt();

	delayTimerB(DELAY_GET_FLAG_PERIOD, false);

	while (!g_bDelayTimerBFlagAssert)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			reloadDelayTimerB();

			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				if (RF24_RX_buffer[0] == ROBOT_RESPONSE_MY_VECTOR && length == 9)
				{
					g_vector.x += (construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0f);
					g_vector.y += (construct4BytesToInt32(&RF24_RX_buffer[5]) / 65536.0f);

					*pCounter = *pCounter + 1;
					break;
				}
				else if (RF24_RX_buffer[0] == ROBOT_RESPONSE_MY_VECTOR_PLEASE_WAIT)
				{
					returnState = false;
					break;
				}
				else // if (RF24_RX_buffer[0] == ROBOT_RESPONSE_MY_VECTOR_NOT_FOUND)
					break;
			}
		}
	}

	enableRF24Interrupt();

	return returnState;
}

bool getNeighborVectorAndFlag(vector2_t *pVectReceived, bool* pIsNeighborGradientSearchStop, bool *pIsNeighborActive)
{
	bool returnState = true;
	uint32_t length = 0;

	disableRF24Interrupt();

	delayTimerB(DELAY_GET_VECTOR_PERIOD, false);

	while (!g_bDelayTimerBFlagAssert)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			reloadDelayTimerB();

			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				if (RF24_RX_buffer[0] == ROBOT_RESPONSE_VECTOR_AND_FLAG && length == 10)
				{
					pVectReceived->x = (construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0f);
					pVectReceived->y = (construct4BytesToInt32(&RF24_RX_buffer[5]) / 65536.0f);
					*pIsNeighborGradientSearchStop = (RF24_RX_buffer[9] == 0x01);
					returnState = true;
					*pIsNeighborActive = true;
					break;
				}
				else if (RF24_RX_buffer[0] == ROBOT_RESPONSE_PLEASE_WAIT)
				{
					returnState = false;
					*pIsNeighborActive = true;
					break;
				}
				else // if (RF24_RX_buffer[0] == ROBOT_RESPONSE_UNACTIVE)
				{
					returnState = true;
					*pIsNeighborActive = false;
					break;
				}
			}
		}
	}

	enableRF24Interrupt();

	return returnState;
}

bool getNeighborVector(uint32_t neighborID)
{
	bool returnState = true;
	uint32_t length = 0;
	int8_t i8Position;
	vector2_t tempVector;

	disableRF24Interrupt();

	delayTimerB(DELAY_GET_VECTOR_PERIOD, false);

	while (!g_bDelayTimerBFlagAssert)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			reloadDelayTimerB();

			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				if (RF24_RX_buffer[0] == ROBOT_RESPONSE_VECTOR && length == 9)
				{
					tempVector.x = construct4BytesToInt32(&RF24_RX_buffer[1]) / 65536.0f;
					tempVector.y = construct4BytesToInt32(&RF24_RX_buffer[5]) / 65536.0f;

					i8Position = isLocationTableContainID(neighborID, locs, g_ui8LocsCounter);
					if (i8Position != -1)
					{
						locs[i8Position].vector.x = tempVector.x;
						locs[i8Position].vector.y = tempVector.y;
					}
					else
					{
						Tri_addLocation(neighborID, tempVector.x, tempVector.y);
					}

					returnState = true;
					break;
				}
				else
				{
					returnState = false;
					break;
				}
			}
		}
	}

	enableRF24Interrupt();

	return returnState;
}

void clearRequestNeighbor(uint8_t RxData[])
{
	uint32_t neighborID = construct4BytesToUint32(&RxData[1]);

	deleteNeighborInInNeighborTable(NeighborsTable, neighborID);
	deleteNeighborInOneHopTable(OneHopNeighborsTable, neighborID);
	deleteNeighborInLocationsTable(locs, neighborID);
}

void updateNeighborVectorInLocsTableByRequest(uint8_t RxData[])
{
	uint32_t neigborId = construct4BytesToUint32(&RF24_RX_buffer[1]); // 1->4
	int32_t i32xAxis = construct4BytesToInt32(&RF24_RX_buffer[5]); // 5->8
	int32_t i32yAxis = construct4BytesToInt32(&RF24_RX_buffer[9]); // 9->12

	float xAxis = i32xAxis / 65536.0;
	float yAxis = i32yAxis / 65536.0;

	updateNeighborInLocationTable(neigborId, xAxis, yAxis);
}

void runForwardAndCalculatteNewPosition(float distance)
{
	uint8_t length;
	uint8_t i;

	vector2_t vectEstimatePosNew;
	vector2_t vectEstimatePosOld;
	vector2_t vectGradienNew;
	vector2_t vectGradienOld;

	distance = (distance > 5) ? (5) : distance;

	clearNeighborTable(NeighborsTable, &g_ui8NeighborsCounter);
	clearOneHopNeighborTable(OneHopNeighborsTable);

	for (i = 0; i < LOCATIONS_TABLE_LENGTH; i++)
	{
		locs[i].ID = 0;
		locs[i].vector.x = 0;
		locs[i].vector.y = 0;
	}
	g_ui8LocsCounter = 0;

	RF24_TX_buffer[0] = ROBOT_REQUEST_TO_RUN;
	parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
	broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);

	runForwardWithDistance(distance);

	SysCtlDelay(2500);

	// send request measure distance command
	RF24_TX_buffer[0] = ROBOT_REQUEST_SAMPLING_MIC;
	parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
	broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
	// WARING!!! DO NOT INSERT ANY CODE IN HERE!
	ROM_SysCtlDelay(DELAY_START_SPEAKER);
	// WARING!!! DO NOT INSERT ANY CODE IN HERE!
	startSpeaker();

	RF24_RX_activate();

	disableRF24Interrupt();

	RF24_clearIrqFlag(RF24_IRQ_MASK);

	g_ui8NeighborsCounter = 0;

	// start state timer wait for new neighbor response
	delayTimerA(DELAY_NEIGHBOR_RESPONSE_PERIOD, false); // may received ROBOT_RESPONSE_TDOA_DISTANCE command at here!

	while (!g_bDelayTimerAFlagAssert)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			toggleLED(LED_GREEN);

			reloadDelayTimerA();

			if (RF24_getIrqFlag(RF24_IRQ_RX))
			{
				length = RF24_RX_getPayloadWidth();

				RF24_RX_getPayloadData(length, RF24_RX_buffer);

				RF24_clearIrqFlag(RF24_IRQ_RX);

				if (RF24_RX_buffer[0] == ROBOT_RESPONSE_TDOA_DISTANCE
							&& length == 15)
				{
					storeNeighorVectorAndDistanceToTables(RF24_RX_buffer);
				}
			}
		}
	}

	turnOffLED(LED_GREEN);

	enableRF24Interrupt();

	if (g_ui8NeighborsCounter < 3)
		return;

	// active gradient descent to calculate new position

	vectEstimatePosNew.x = g_vector.x;
	vectEstimatePosNew.y = g_vector.y;

	vectGradienNew.x = 0;
	vectGradienNew.y = 0;

	g_bIsGradientSearchStop = false;

	while(!g_bIsGradientSearchStop)
	{
		toggleLED(LED_BLUE);

		updateGradient(&vectGradienNew, false);

		updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, g_fStepSize);
		synchronousLocsTableAndMyVector();

		g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition);
	}

	Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);

	turnOffLED(LED_BLUE);
}

int8_t calculateQuadrant(vector2_t vectSource, vector2_t vectDestinate)
{
	vector2_t vectDiff;
	vectDiff.x = vectDestinate.x - vectSource.x;
	vectDiff.y = vectDestinate.y - vectSource.y;

	if (vectDiff.x == 0 && vectDiff.y == 0)
		return 0;

	if (vectDiff.y == 0)
	{
		if (vectDiff.x > 0)
			return 1;
		else
			return 3;
	}

	if (vectDiff.x == 0)
	{
		if (vectDiff.y > 0)
			return 4;
		else
			return 2;
	}

	if (vectDiff.y > 0)
	{
		if (vectDiff.x > 0)
			return 1;
		else
			return 4;
	}
	else
	{
		if (vectDiff.x > 0)
			return 2;
		else
			return 3;
	}
}

float calculateRobotOrientation(vector2_t vectDiff)
{
	if (vectDiff.x == 0)
	{
		if (vectDiff.y > 0)
			return MATH_PI_DIV_2;
		else
			return MINUS_MATH_PI_DIV_2;
	}

	if (vectDiff.y == 0)
	{
		if (vectDiff.x > 0)
			return 0;
		else
			return MATH_PI;
	}

	float fraction = vectDiff.y / vectDiff.x;

	return calASin(fraction / vsqrtf(fraction * fraction + 1));
}

float calculateRobotAngleWithXAxis(vector2_t vectDiff)
{
	float fraction;
	float angle;

	if (vectDiff.x == 0)
	{
		if (vectDiff.y > 0)
			return MATH_PI_DIV_2; 		// OY 90
		else
			return MATH_PI_MUL_3_DIV_2; // -OY 270
	}

	if (vectDiff.y == 0)
	{
		if (vectDiff.x > 0)
			return 0; 		// OX 0
		else
			return MATH_PI;	// -OX 180
	}

	fraction = vectDiff.y / vectDiff.x;

	angle = calASin(fraction / vsqrtf(fraction * fraction + 1));

	if (vectDiff.x > 0) // Quadrant I & II
		angle = MATH_PI_MUL_2 + angle;
	else // Quadrant III & IV
		angle = MATH_PI + angle;

	while(angle > MATH_PI_MUL_2)
		angle -= MATH_PI_MUL_2;

	return angle;
}

float calculateTotalDistance(vector2_t v1, vector2_t v2, vector2_t v3, vector2_t v4, uint32_t *selectedResult)
{
	uint8_t selectedNode[4];
	float totalDistance = 0;

	int8_t i;
	int8_t j;
	float distanceMin, distanceScan;

	// Choose the First Node for v1
	j = 0;
	while(locs[j].ID == g_ui32OriginID)
		j++;
	selectedNode[0] = j;

	distanceMin = calculateDistanceBetweenTwoNode(v1, locs[selectedNode[0]].vector);
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32OriginID || i == selectedNode[0])
			continue;
		distanceScan = calculateDistanceBetweenTwoNode(v1, locs[i].vector);
		if (distanceScan < distanceMin)
		{
			distanceMin = distanceScan;
			selectedNode[0] = i;
		}
	}
	totalDistance += distanceMin;

	// Choose the Second Node for v2
	j = 0;
	while(locs[j].ID == g_ui32OriginID || j == selectedNode[0])
		j++;
	selectedNode[1] = j;
	distanceMin = calculateDistanceBetweenTwoNode(v2, locs[selectedNode[1]].vector);

	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32OriginID || i == selectedNode[0] || i == selectedNode[1])
			continue;
		distanceScan = calculateDistanceBetweenTwoNode(v2, locs[i].vector);
		if (distanceScan < distanceMin)
		{
			distanceMin = distanceScan;
			selectedNode[1] = i;
		}
	}
	totalDistance += distanceMin;

	// Choose the Third Node for v3
	j = 0;
	while(locs[j].ID == g_ui32OriginID || j == selectedNode[0] || j == selectedNode[1])
		j++;
	selectedNode[2] = j;

	distanceMin = calculateDistanceBetweenTwoNode(v3, locs[selectedNode[2]].vector);
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32OriginID || i == selectedNode[0] || i == selectedNode[1] || i == selectedNode[2])
			continue;
		distanceScan = calculateDistanceBetweenTwoNode(v3, locs[i].vector);
		if (distanceScan < distanceMin)
		{
			distanceMin = distanceScan;
			selectedNode[2] = i;
		}
	}
	totalDistance += distanceMin;

	// Last Node
	j = 0;
	while(locs[j].ID == g_ui32OriginID ||j == selectedNode[0] || j == selectedNode[1] || j == selectedNode[2])
		j++;
	selectedNode[3] = j;

	distanceMin = calculateDistanceBetweenTwoNode(v4, locs[selectedNode[3]].vector);
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == g_ui32OriginID || i == selectedNode[0] || i == selectedNode[1] || i == selectedNode[2] || i == selectedNode[3])
			continue;
		distanceScan = calculateDistanceBetweenTwoNode(v4, locs[i].vector);
		if (distanceScan < distanceMin)
		{
			distanceMin = distanceScan;
			selectedNode[3] = i;
		}
	}
	totalDistance += distanceMin;

	*selectedResult = construct4BytesToUint32(selectedNode);

	return totalDistance;
}

float calculateDistanceBetweenTwoNode(vector2_t v1, vector2_t v2)
{
	vector2_t diff;
	diff.x = v1.x - v2.x;
	diff.y = v1.y - v2.y;

	return vsqrtf((diff.x * diff.x) + (diff.y * diff.y));
}

void notifyNewVectorToNeigbors()
{
	uint8_t i;
	int32_t i32TempAxix;

	for(i = 0; i < g_ui8NeighborsCounter; i++)
	{
		RF24_TX_buffer[0] = ROBOT_REQUEST_UPDATE_VECTOR;
		parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4

		i32TempAxix = g_vector.x * 65536 + 0.5;
		parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[5]); // 5->8

		i32TempAxix = g_vector.y * 65536 + 0.5;
		parse32BitTo4Bytes(i32TempAxix, &RF24_TX_buffer[9]); // 9->12

		sendMessageToOneNeighbor(NeighborsTable[g_ui8NeighborsCounter].ID, RF24_TX_buffer, 13);
	}
}

void clearNeighborTable(robotMeas_t neighborTable[], uint8_t *counter)
{
	deleteTable(neighborTable);
	*counter = 0;
}

void clearOneHopNeighborTable(oneHopMeas_t table[])
{
	uint8_t i;
	for(i = 0; i < ONEHOP_NEIGHBOR_TABLE_LENGTH; i++)
	{
		table[i].firstHopID = 0;
		deleteTable(table[i].neighbors);
	}
}

void deleteNeighborInInNeighborTable(robotMeas_t table[], uint32_t id)
{
	uint8_t i;
	for(i = 0; i < NEIGHBOR_TABLE_LENGTH; i++)
	{
		if (table[i].ID == id)
		{
			while(i < NEIGHBOR_TABLE_LENGTH - 1)
			{
				table[i].ID = table[i + 1].ID;
				table[i].distance = table[i + 1].distance;
				i++;
			}
			table[i].ID = 0;
			table[i].distance = 0;
			break;
		}
	}
}

void deleteNeighborInOneHopTable(oneHopMeas_t table[], uint32_t id)
{
	uint8_t i;
	for(i = 0; i < ONEHOP_NEIGHBOR_TABLE_LENGTH; i++)
	{
		if (table[i].firstHopID == id)
		{
			while(i < ONEHOP_NEIGHBOR_TABLE_LENGTH - 1)
			{
				table[i].firstHopID = table[i + 1].firstHopID;
				*(table[i].neighbors) = *(table[i + 1].neighbors);
				i++;
			}
			table[i].firstHopID = 0;
			deleteTable(table[i].neighbors);
			break;
		}
	}
}

void deleteNeighborInLocationsTable(location_t table[], uint32_t id)
{
	uint8_t i;
	for(i = 0; i < LOCATIONS_TABLE_LENGTH; i++)
	{
		if (table[i].ID == id)
		{
			while(i < LOCATIONS_TABLE_LENGTH - 1)
			{
				table[i].ID = table[i + 1].ID;
				table[i].vector.x = table[i + 1].vector.x;
				table[i].vector.y = table[i + 1].vector.x;
				i++;
			}

			table[i].ID = 0;
			table[i].vector.x = 0;
			table[i].vector.y = 0;
			break;
		}
	}
}

void updateNeighborInLocationTable(uint32_t neigborId, float xAxis, float yAxis)
{
	uint8_t i;
	for(i = 0; i < g_ui8LocsCounter; i++)
	{
		if (locs[i].ID == neigborId)
		{
			locs[i].vector.x = xAxis;
			locs[i].vector.y = yAxis;
			return;
		}
	}
	Tri_addLocation(neigborId, xAxis, yAxis);
}

void deleteTable(robotMeas_t neighborTable[])
{
	uint8_t i;
	for(i = 0; i < NEIGHBOR_TABLE_LENGTH; i++)
	{
		neighborTable[i].ID = 0;
		neighborTable[i].distance = 0;
	}
}


void rotateClockwiseWithAngle(float angleInRadian)
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;
	uint16_t ui16DelayPeriod;

	MotorDirection_t motorLeftDirection;
	MotorDirection_t motorRightDirection;
	MotorDirection_t motorTempDirection;

	uint32_t pui32Read[1];

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = (*pui32Read & 0xFF);
	i8Motor2Speed = (((*pui32Read) >> 8) & 0xFF);
	ui16DelayPeriod = ((*pui32Read) >> 16) & 0xFFFF;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	if (angleInRadian < 0)
	{
		motorLeftDirection = FORWARD;
		motorRightDirection = REVERSE;

		angleInRadian = 0 - angleInRadian;
	}
	else
	{
		motorLeftDirection = REVERSE;
		motorRightDirection = FORWARD;
	}

	while(angleInRadian > MATH_PI_MUL_2)
		angleInRadian -= MATH_PI_MUL_2;

	if (angleInRadian > MATH_PI)
	{
		angleInRadian = MATH_PI_MUL_2 - angleInRadian;
		motorTempDirection = motorLeftDirection;
		motorLeftDirection = motorRightDirection;
		motorRightDirection = motorTempDirection;
	}

	configureMotors(motorLeftDirection , i8Motor1Speed + 5, motorRightDirection, i8Motor2Speed + 5);
	delayTimerNonInt((uint32_t)((ui16DelayPeriod / MATH_PI_DIV_2) * angleInRadian + 0.5));
	stopMotors();
}

void runForwardWithDistance(float distance)
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;
	uint16_t ui16DelayPeriod;

	MotorDirection_t motorLeftDirection;
	MotorDirection_t motorRightDirection;

	uint32_t pui32Read[1];

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = (*pui32Read & 0xFF);
	i8Motor2Speed = (((*pui32Read) >> 8) & 0xFF);
	ui16DelayPeriod = ((*pui32Read) >> 16) & 0xFFFF;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	if (distance < 0)
	{
		distance = 0 - distance;
		motorLeftDirection = REVERSE;
		motorRightDirection = REVERSE;
	}
	else
	{
		motorLeftDirection = FORWARD;
		motorRightDirection = FORWARD;
	}

	configureMotors(motorLeftDirection , i8Motor1Speed + 5, motorRightDirection, i8Motor2Speed + 5);
	delayTimerNonInt((uint32_t)((ui16DelayPeriod / 8.0) * distance + 0.5));
	stopMotors();
}



