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

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"
#include "librobot/inc/MainBoardDriver.h"
#include "librobot/inc/Trilateration.h"

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


//----------------Math functions-------------------
float vsqrtf(float op1)
{
	if (op1 <= 0.f)
		return 0.f;

	float result;
	__ASM
	volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
	return (result);
}

float calSin(float x)
{

	float tempX;
	float angleX;
	uint32_t angleIndex;
	int8_t resultSigned;
	uint8_t selectResult;
	uint32_t resultIndex;
	uint16_t pui16ReadBuffer[2] =
	{ 0, 0 };

	x *= _180_DIV_PI;

	tempX = (x > 0) ? (x) : (360 + x);

	// angleX = (tempX > 360) ? (tempX - 360.0) : (tempX);
	angleX = tempX;
	while(angleX > 360)
		angleX -= 360.0;

	angleIndex = (int) (angleX * 2 + 0.5);

	if (angleIndex < 180)
	{
		resultSigned = 1;
		resultIndex = angleIndex;
	}
	else if (angleIndex < 360)
	{
		resultSigned = 1;
		resultIndex = 360 - angleIndex;
	}
	else if (angleIndex < 540)
	{
		resultSigned = -1;
		resultIndex = angleIndex - 360;
	}
	else
	{
		resultSigned = -1;
		resultIndex = 720 - angleIndex;
	}

	selectResult = resultIndex & 0x01;

	resultIndex &= 0xFFFFFFFE;
	resultIndex <<= 1;
	resultIndex += EPPROM_SINE_TABLE_ADDRESS;

	EEPROMRead((uint32_t*) pui16ReadBuffer, resultIndex,
			sizeof(pui16ReadBuffer));

	return (((resultSigned * pui16ReadBuffer[selectResult])) / 32768.0);
}

float calCos(float x)
{
	return calSin(x + MATH_PI_DIV_2);
}

float calASin(float x)
{
	int8_t resultSigned;
	uint8_t selectResult;
	uint32_t resultIndex;
	uint16_t pui16ReadBuffer[2] =
	{ 0, 0 };

	if (x > 0)
	{
		resultSigned = 1;
		resultIndex = (int) ((x * 180) + 0.25);
	}
	else
	{
		resultSigned = -1;
		resultIndex = (int) (((-x) * 180) + 0.25);
	}

	selectResult = resultIndex & 0x01;

	resultIndex &= 0xFFFFFFFE;
	resultIndex <<= 1;
	resultIndex += EPPROM_ARC_SINE_TABLE_ADDRESS;

	EEPROMRead((uint32_t*) pui16ReadBuffer, resultIndex,
			sizeof(pui16ReadBuffer));

	return ((resultSigned * pui16ReadBuffer[selectResult]) / 32768.0);
}

float calACos(float x)
{
	//return (MATH_PI_DIV_2_MUL_32768 - calASin(x));
	return (MATH_PI_DIV_2 - calASin(x));
//	float ASin = calASin(x);
//	float result = MATH_PI_DIV_2 - ASin;
//
//	return (result);
}

float cosinesRuleForTriangles(float a, float b, float c)
{
	return (((a * a) + (b * b) - (c * c)) / (2 * a * b));
}

bool isTriangle(float a, float b, float c)
{
	if (((a + b) > c) && ((b + c) > a) && ((a + c) > b))
		return true;
	return false;
}

bool isValidTriangle(uint32_t a, uint32_t b, uint32_t c)
{
	float cosA;
	float cosB;
	float cosC;

	float fa = a / 256.0f;
	float fb = b / 256.0f;
	float fc = c / 256.0f;

	if (!isTriangle(fa, fb, fc))
		return false;

	cosA = cosinesRuleForTriangles(fb, fc, fa);
	if (cosA > COSINE_ANGLE_MIN)
	 return false;

	cosB = cosinesRuleForTriangles(fa, fc, fb);
	if (cosB > COSINE_ANGLE_MIN)
	 return false;

	cosC = cosinesRuleForTriangles(fa, fb, fc);
	if (cosC > COSINE_ANGLE_MIN)
	 return false;

	return true;
}

//-----------------------------------Math functions


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

bool g_bIsNetworkRotated;
bool g_bIsActiveCoordinatesFixing;
bool g_bIsGradientSearchStop;
uint32_t g_ui32LocalLoop;

ProcessStateEnum g_eProcessState;

extern location_t locs[];

bool g_bBypassThisState;
uint8_t g_ui8ReBroadcastCounter;

float g_f32Intercept = 8.5619f;
float g_f32Slope = 3.046f;

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
		g_f32Intercept = 8.5619f;
		g_f32Slope = 3.046f;
	}

	g_eProcessState = IDLE;
	g_vector.x = (g_ui32RobotID & 0xFF);
	g_vector.y = (g_ui32RobotID & 0xFF);
	g_bIsValidVector = false;
	g_bIsNetworkRotated = false;
	g_bIsActiveCoordinatesFixing = false;
	g_bIsGradientSearchStop = false;
	g_ui32LocalLoop = 0;
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

			//tableSizeInByte = sizeof(NeighborsTable); //TODO: optimize size
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

void sendVectorToControlBoard(const vector2_t vector)
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

	spinClockwise();
	delayTimerB(ui32DelayPeriod, true);
	stopMotors();
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
	float correctionAngle;
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

//-----------------------------------Robot Int functions

//-----------------------LED functions-------------------------
inline void initLED()
{
	SysCtlPeripheralEnable(LED_CLOCK_PORT);
	GPIOPinTypeGPIOOutput(LED_PORT_BASE, LED_RED | LED_GREEN | LED_BLUE);
	turnOffLED(LED_RED | LED_GREEN | LED_BLUE);
}

inline void turnOnLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = LEDpin;
}

inline void turnOffLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) = 0x00;
}

inline void toggleLED(uint8_t LEDpin)
{
	ASSERT(_GPIOBaseValid(LED_PORT_BASE));
	HWREG(LED_PORT_BASE + (GPIO_O_DATA + (LEDpin << 2))) ^= 0xFF;
}

void signalUnhandleError()
{
	int i;
	for (i = 0; i < 2; i++)
	{
		GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);
		SysCtlDelay(2000000);

		GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);
		SysCtlDelay(2000000);

		GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_BLUE);
		SysCtlDelay(2000000);
	}
}
//------------------------------------------------LED functions

//-----------------------Motor functions-----------------------
static uint32_t ui32PWMPeriod;

inline void initMotor()
{
	// Initilize PWM generator module 0
	SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(MOTOR_PWM_CLOCK);

	uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	ui32PWMPeriod = (pwmClock / MOTOR_PWM_FREQUENCY);

	// Configure GPIO pin to control Motor driver IC
	SysCtlPeripheralEnable(MOTOR_SLEEP_PIN_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeGPIOOutput(MOTOR_SLEEP_PIN_BASE, MOTOR_SLEEP_PIN);

	// Left motor (1) pin configure
	SysCtlPeripheralEnable(LEFT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);

	GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// left - M0PWM4
	PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2, 0);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	// left - M0PWM5
	PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, 0);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);

	// Right motor (2) pin configure
	SysCtlPeripheralEnable(RIGHT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);

	GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	// right - M0PWM2
	PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2, 0);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	// right - M0PWM3
	PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, 0);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
}

inline void enableMOTOR()
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x01;
}

inline void disableMOTOR()
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x00;
}

void setMotorLeftDirectionAndSpeed(uint8_t direction, uint8_t speed)
{
	if (direction == FORWARD)
	{
		// left - E5
		GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN1);

		// left - M0PWM4
		GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG2);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2,
				(speed * ui32PWMPeriod) / 100);

		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);
		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (direction == REVERSE)
	{
		// left - M0PWM5
		GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG1);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				(speed * ui32PWMPeriod) / 100);

		// left - E4
		GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2, LEFT_MOTOR_IN2);

		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	}
}

void setMotorRightDirectionAndSpeed(uint8_t direction, uint8_t speed)
{
	if (direction == FORWARD)
	{
		// right - B5
		GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN1);

		// right - M0PWM2
		GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG2);
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2,
				(speed * ui32PWMPeriod) / 100);

		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (direction == REVERSE)
	{
		// right - M0PWM3
		GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG1);
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				(speed * ui32PWMPeriod) / 100);

		// right - B4
		GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2, RIGHT_MOTOR_IN2);

		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	}
}

void configureMotors(uint8_t leftDir, uint8_t leftDutyCycle, uint8_t rightDir, uint8_t rightDutyCycle)
{
	disableMOTOR();

	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorLeftDirectionAndSpeed(leftDir, leftDutyCycle);
	setMotorRightDirectionAndSpeed(rightDir, rightDutyCycle);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	enableMOTOR();
}

void stopMotorLeft()
{
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);

}

void stopMotorRight()
{
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);

}

void stopMotors()
{
	stopMotorLeft();
	stopMotorRight();
	disableMOTOR();
}

//----------------------------------------------Motor functions

//--------------------------------Ananlog functions-----------------------------------
unsigned char countAdcDMAsStopped = 0;

static uint32_t g_ui32uDMAErrCount = 0;

//*****************************************************************************
// The buffers for ADCs
//*****************************************************************************
uint16_t g_pui16ADC0Result[NUMBER_OF_SAMPLE];
uint16_t g_pui16ADC1Result[NUMBER_OF_SAMPLE];

uint16_t g_ui16BatteryVoltage;

#ifdef gcc
static uint8_t ui8ControlTable[1024] __attribute__ ((aligned (1024)));
#else
#pragma DATA_ALIGN(ui8ControlTable, 1024)
static uint8_t ui8ControlTable[1024];
#endif

inline void initPeripheralsForAnalogFunction(void)
{

	/*
	 * Configure GPIO to analog type
	 *  ground adjacent pin to eliminate noise
	 */
	SysCtlPeripheralEnable(ADC_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeADC(ADC_PORT, ADC0_IN);
	GPIOPinTypeADC(ADC_PORT, ADC1_IN);

	GPIOPinTypeGPIOOutput(ADC_PORT, ADC_ADJACENT_PINS);
	GPIOPinWrite(ADC_PORT, ADC_ADJACENT_PINS, 0);

	SysCtlPeripheralEnable(BATTERY_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeADC(BATTERY_PORT, BATTERY_IN);

	/*
	 * Initialize ADC0
	 *  Sequence Type 2 use for sampling battery level
	 * 	Sequence Type 3 use for sampling microphone 1
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCHardwareOversampleConfigure(ADC0_BASE, ADC_AVERAGING_FACTOR);

	ADCSequenceConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, BATTERY_MEASURENMENT_PRIORITY);
	ADCSequenceStepConfigure(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, 0,
	BATTERY_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
	//ADCIntClear(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);

	ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC0_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC0_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize ADC1
	 *  Sequence type 0 use for generate random number
	 *	Sequence Type 3 use for sampling microphone 2
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	ADCHardwareOversampleConfigure(ADC1_BASE, ADC_AVERAGING_FACTOR);

	ADCSequenceConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
	ADC_TRIGGER_PROCESSOR, RANDOM_GEN_PRIORITY);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			0, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			1, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			2, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			3, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			4, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			5, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			6, RANDOM_GEN_CHANNEL);
	ADCSequenceStepConfigure(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
			7, RANDOM_GEN_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);

	ADCSequenceConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, ADC_TRIGGER_TIMER,
	DISTANCE_SENSING_PRIORITY);
	ADCSequenceStepConfigure(ADC1_BASE, ADC_SEQUENCE_TYPE, 0,
	ADC1_CHANNEL | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);
	ADCSequenceDMAEnable(ADC1_BASE, ADC_SEQUENCE_TYPE);

	/*
	 * Initialize uDMA
	 *  Channel 16 handle ADC0 Sequence type 2 - battery level
	 *	Channel 17 handle ADC0 Sequence type 3 - microphone 1
	 *	Channel 24 handle ADC1 Sequence type 0 - random number generator
	 *  Channel 27 handle ADC1 Sequence type 3 - microphone 2
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();
	uDMAControlBaseSet(ui8ControlTable);

	uDMAChannelAssign(DMA_BATT_CHANNEL);
	uDMAChannelAttributeDisable(BATT_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_1);
	uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
			&g_ui16BatteryVoltage, 1);
	uDMAChannelEnable(BATT_DMA_CHANNEL);

	uDMAChannelAssign(DMA_ADC0_CHANNEL);
	uDMAChannelAttributeDisable(ADC0_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS), g_pui16ADC0Result,
			NUMBER_OF_SAMPLE);
	uDMAChannelEnable(ADC0_DMA_CHANNEL);

	uDMAChannelAssign(DMA_RANDOM_GEN_CHANNEL);
	uDMAChannelAttributeDisable(RANDOM_GEN_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_8);
	uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_MODE_BASIC,
			(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
			g_pui8RandomBuffer, 8);
	uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

	uDMAChannelAssign(DMA_ADC1_CHANNEL);
	uDMAChannelAttributeDisable(ADC1_DMA_CHANNEL,
	UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
	uDMAChannelControlSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
	UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
	uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
			(void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS), g_pui16ADC1Result,
			NUMBER_OF_SAMPLE);
	uDMAChannelEnable(ADC1_DMA_CHANNEL);

	/*
	 * Configure Interrupts
	 */
	IntPrioritySet(ADC0_INT, PRIORITY_DMA_MIC1);
	IntPrioritySet(ADC1_INT, PRIORITY_DMA_MIC2);
	IntPrioritySet(ADC_BATT_INT, PRIORITY_DMA_BATT);
	IntPrioritySet(RANDOM_GEN_INT, PRIORITY_DMA_RANDOM_GEN);

	IntEnable(ADC0_INT);
	IntEnable(ADC1_INT);
	IntEnable(ADC_BATT_INT);
	IntEnable(RANDOM_GEN_INT);

	IntEnable(INT_UDMAERR);

	/*
	 * Configure Timer to trigger ADC0 Seq3 and ADC1 Seq3
	 */
	SysCtlPeripheralEnable(ADC_TIMER_CLOCK);
	TimerDisable(ADC_TIMER, TIMER_A);
	TimerClockSourceSet(ADC_TIMER, TIMER_CLOCK_SYSTEM);
	TimerConfigure(ADC_TIMER, TIMER_CFG_PERIODIC);
	TimerLoadSet(ADC_TIMER, TIMER_A, (SysCtlClockGet() / SAMPLE_FREQUENCY));
	TimerControlTrigger(ADC_TIMER, TIMER_A, true);
}

inline void startSamplingMicSignals()
{
	disableMOTOR();
	countAdcDMAsStopped = 0;
	ROM_SysCtlDelay(DELAY_SAMPING_MIC);
	TimerEnable(ADC_TIMER, TIMER_A);
}
inline void startSamplingBatteryVoltage()
{
	disableMOTOR();
	ADCProcessorTrigger(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE);
}
inline void generateRandomByte()
{
	g_ui8RandomNumber = 0;
	ADCProcessorTrigger(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE);
}

float generateRandomRange(float min, float max)
{
	if (max <= min)
		return 0;

	float fResolution = 255.0f / (max - min);

	generateRandomByte();
	while (g_ui8RandomNumber == 0);

	return ((float)(g_ui8RandomNumber / fResolution) + min);
}

void ADC0IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC0_BASE, ADC_SEQUENCE_TYPE, true);
	ADCIntClear(ADC0_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		uDMAChannelTransferSet(ADC0_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC0_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC0Result,
				NUMBER_OF_SAMPLE);
		uDMAChannelEnable(ADC0_DMA_CHANNEL);

		countAdcDMAsStopped++;
		if (countAdcDMAsStopped == 2)
			enableMOTOR();
	}
}
void ADC1IntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC1_BASE, ADC_SEQUENCE_TYPE, true);
	ADCIntClear(ADC1_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		TimerDisable(ADC_TIMER, TIMER_A);
		// Setup for a future request
		uDMAChannelTransferSet(ADC1_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC1_BASE + ADC_SEQUENCE_ADDRESS),
				g_pui16ADC1Result,
				NUMBER_OF_SAMPLE);
		uDMAChannelEnable(ADC1_DMA_CHANNEL);

		countAdcDMAsStopped++;
		if (countAdcDMAsStopped == 2)
			enableMOTOR();
	}
}
void BatterySequenceIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC_BATT_BASE, ADC_BATT_SEQUENCE_TYPE, true);
	ADCIntClear(ADC_BATT_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		uDMAChannelTransferSet(BATT_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC, (void *) (ADC_BATT_BASE + ADC_BATT_SEQUENCE_ADDRESS),
				&g_ui16BatteryVoltage, 1);
		uDMAChannelEnable(BATT_DMA_CHANNEL);
		sendDataToControlBoard((uint8_t *) &g_ui16BatteryVoltage);
		enableMOTOR();
	}
}
void RandomGeneratorIntHandler(void)
{
	uint32_t ui32Status;
	uint32_t ui32Mode;

	ui32Status = ADCIntStatus(ADC_RANDOM_GEN_BASE, ADC_RANDOM_GEN_SEQUENCE_TYPE,
	true);
	ADCIntClear(ADC_RANDOM_GEN_BASE, ui32Status);

	ui32Mode = uDMAChannelModeGet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT);
	if (ui32Mode == UDMA_MODE_STOP)
	{
		// Setup for a future request
		uDMAChannelTransferSet(RANDOM_GEN_DMA_CHANNEL | UDMA_PRI_SELECT,
		UDMA_MODE_BASIC,
				(void *) (ADC_RANDOM_GEN_BASE + RANDOM_GEN_SEQUENCE_ADDRESS),
				g_pui8RandomBuffer, 8);
		uDMAChannelEnable(RANDOM_GEN_DMA_CHANNEL);

		int i;
		for (i = 0; i < 8; i++)
		{
			g_ui8RandomNumber |= ((g_pui8RandomBuffer[i] & 0x01) << i);
		}

		if (g_ui8RandomNumber == 0 || g_ui8RandomNumber == 0xFF)
		{
			g_ui8RandomNumber = 0;
			ADCProcessorTrigger(ADC_RANDOM_GEN_BASE,
			ADC_RANDOM_GEN_SEQUENCE_TYPE);
		}
	}
}
void uDMAErrorHandler(void)
{
	uint32_t ui32Status;

	ui32Status = uDMAErrorStatusGet();

	if (ui32Status)
	{
		uDMAErrorStatusClear();
		g_ui32uDMAErrCount++;
	}
}

//-------------------------------------------------------------------Ananlog functions


//----------------Speaker functions-------------------
inline void initSpeaker()
{
	SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(SPEAKER_PWM_CLOCK_BASE);

//  SysCtlPeripheralEnable(SPEAKER_PORT_CLOCK);
	SysCtlDelay(2);

	if ((SPEAKER_PORT_BASE == GPIO_PORTF_BASE) && (SPEAKER_PIN == GPIO_PIN_0))
	{
		// unlock the GPIO commit control register to modify PF0 configuration because it may be configured to be a NMI input.
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	}

	uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	uint32_t pwmPeriod = (pwmClock / SPEAKER_PWM_FREQUENCY);

	GPIOPinConfigure(SPEAKER_PWM_CONFIG);
	GPIODirModeSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_DIR_MODE_HW);
	GPIOPadConfigSet(SPEAKER_PORT_BASE, SPEAKER_PIN, GPIO_STRENGTH_8MA,
	GPIO_PIN_TYPE_STD);

	PWMGenConfigure(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN, pwmPeriod);
	PWMPulseWidthSet(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT, pwmPeriod / 2);

	SysCtlPeripheralEnable(SPEAKER_TIMER_CLOCK);
	TimerDisable(SPEAKER_TIMER_BASE, TIMER_A);
	TimerConfigure(SPEAKER_TIMER_BASE, TIMER_CFG_ONE_SHOT);
	TimerLoadSet(SPEAKER_TIMER_BASE, TIMER_A,
			(SysCtlClockGet() / SPEAKER_TIMER_FREQUENCY));
	IntMasterEnable();
	TimerIntEnable(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(SPEAKER_INT);
}

void startSpeaker()
{
	PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, true);
	PWMGenEnable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	TimerEnable(SPEAKER_TIMER_BASE, TIMER_A);
}

void SpeakerTimerIntHandler(void)
{
	TimerIntClear(SPEAKER_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	PWMGenDisable(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN);
	PWMSyncTimeBase(SPEAKER_PWM_BASE, SPEAKER_PWM_GEN_BIT);
	PWMOutputState(SPEAKER_PWM_BASE, SPEAKER_PWM_OUT_BIT, false);
}
//-----------------------------------Speaker functions

//----------------------RF24 Functions------------------------
#define RF24_CONTOLBOARD_ADDR_BYTE2		0xC1
#define RF24_CONTOLBOARD_ADDR_BYTE1		0xAC
#define RF24_CONTOLBOARD_ADDR_BYTE0		0x02

inline void initRfModule()
{
	RF24_InitTypeDef initRf24;
	initRf24.AddressWidth = RF24_ADRESS_WIDTH_3;
	initRf24.Channel = RF24_CHANNEL_0;
	initRf24.CrcBytes = RF24_CRC_2BYTES;
	initRf24.CrcState = RF24_CRC_EN;
	initRf24.RetransmitCount = RF24_RETRANS_COUNT15;
	initRf24.RetransmitDelay = RF24_RETRANS_DELAY_4000u;
	initRf24.Speed = RF24_SPEED_1MBPS;
	initRf24.Power = RF24_POWER_0DBM;
	initRf24.Features = RF24_FEATURE_EN_DYNAMIC_PAYLOAD
			| RF24_FEATURE_EN_NO_ACK_COMMAND;
	initRf24.InterruptEnable = true;
	initRf24.LNAGainEnable = true;
	RF24_init(&initRf24);

	// Set payload 3 pipes dynamic
	RF24_PIPE_setPacketSize(RF24_PIPE0, RF24_PACKET_SIZE_DYNAMIC);
	RF24_PIPE_setPacketSize(RF24_PIPE1, RF24_PACKET_SIZE_DYNAMIC);
	RF24_PIPE_setPacketSize(RF24_PIPE2, RF24_PACKET_SIZE_DYNAMIC);

	// Open 3 pipes with Enhanced ShockBurst enabled for receiving Auto-ACKs
	RF24_PIPE_open(RF24_PIPE0, true); // Robot ID
	RF24_PIPE_open(RF24_PIPE1, true); // Global Boardcast (RX command form ControlBoard)
	RF24_PIPE_open(RF24_PIPE2, true); // Local Boardcast (RX message form another robots)

	uint8_t addr[3];

	if (g_ui32RobotID != 0xFFFFFFFF)
	{
		addr[2] = g_ui32RobotID >> 16;
		addr[1] = g_ui32RobotID >> 8;
		addr[0] = g_ui32RobotID;
		RF24_RX_setAddress(RF24_PIPE0, addr);
	}

	addr[2] = RF24_GLOBAL_BOARDCAST_BYTE2;
	addr[1] = RF24_GLOBAL_BOARDCAST_BYTE1;
	addr[0] = RF24_GLOBAL_BOARDCAST_BYTE0;
	RF24_RX_setAddress(RF24_PIPE1, addr);

	addr[2] = RF24_LOCAL_BOARDCAST_BYTE2;
	addr[1] = RF24_LOCAL_BOARDCAST_BYTE1;
	addr[0] = RF24_LOCAL_BOARDCAST_BYTE0;
	RF24_RX_setAddress(RF24_PIPE2, addr);

	RF24_RX_activate();

//	uint32_t g_ui32AddressTX;
//	uint32_t g_ui32AddressPipe[6];
//
//	clearRfCSN();
//    SPI_sendAndGetData((RF24_REG_TX_ADDR & RF24_REG_MASK) | RF24_COMMAND_R_REGISTER);
//	g_ui32AddressTX  = SPI_sendAndGetData(RF24_COMMAND_NOP) & 0x0000FF;
//	g_ui32AddressTX |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 8;
//	g_ui32AddressTX |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 16;
//	setRfCSN();
//
//	clearRfCSN();
//    SPI_sendAndGetData((RF24_REG_RX_ADDR_P0 & RF24_REG_MASK) | RF24_COMMAND_R_REGISTER);
//	g_ui32AddressPipe[0]  = SPI_sendAndGetData(RF24_COMMAND_NOP) & 0x0000FF;
//	g_ui32AddressPipe[0] |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 8;
//	g_ui32AddressPipe[0] |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 16;
//	setRfCSN();
//
//	clearRfCSN();
//    SPI_sendAndGetData((RF24_REG_RX_ADDR_P1 & RF24_REG_MASK) | RF24_COMMAND_R_REGISTER);
//	g_ui32AddressPipe[1]  = SPI_sendAndGetData(RF24_COMMAND_NOP) & 0x0000FF;
//	g_ui32AddressPipe[1] |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 8;
//	g_ui32AddressPipe[1] |= SPI_sendAndGetData(RF24_COMMAND_NOP) << 16;
//	setRfCSN();
//
//	g_ui32AddressPipe[2] = (g_ui32AddressPipe[1] & 0xFFFF00) | RF24_readRegister(RF24_REG_RX_ADDR_P2);
//
//	g_ui32AddressPipe[3] = (g_ui32AddressPipe[1] & 0xFFFF00) | RF24_readRegister(RF24_REG_RX_ADDR_P3);
//
//	g_ui32AddressPipe[4] = (g_ui32AddressPipe[1] & 0xFFFF00) | RF24_readRegister(RF24_REG_RX_ADDR_P4);
//
//	g_ui32AddressPipe[5] = (g_ui32AddressPipe[1] & 0xFFFF00) | RF24_readRegister(RF24_REG_RX_ADDR_P5);
}

void sendDataToControlBoard(uint8_t * data)
{
	GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);

	uint32_t length;
	length = RF24_RX_buffer[1];

	length = (length << 8) + RF24_RX_buffer[2];
	length = (length << 8) + RF24_RX_buffer[3];
	length = (length << 8) + RF24_RX_buffer[4];

	uint32_t pointer = 0;
	uint32_t i;

	uint8_t addr[3];

	addr[2] = RF24_CONTOLBOARD_ADDR_BYTE2;
	addr[1] = RF24_CONTOLBOARD_ADDR_BYTE1;
	addr[0] = RF24_CONTOLBOARD_ADDR_BYTE0;
	RF24_RX_setAddress(RF24_PIPE0, addr);
	RF24_TX_setAddress(addr);

	RF24_RX_flush();
	RF24_clearIrqFlag(RF24_IRQ_RX);
	RF24_TX_activate();
	RF24_RETRANS_setCount(RF24_RETRANS_COUNT15);
	RF24_RETRANS_setDelay(RF24_RETRANS_DELAY_4000u);

	disableRF24Interrupt();

	while (1)
	{
		rfDelayLoop(DELAY_CYCLES_1MS5);

		if (data != RF24_TX_buffer)
		{
			for (i = 0; (i < length) && (i < 32); i++)
			{
				RF24_TX_buffer[i] = *(data + pointer);
				pointer++;
			}
		}
		else
		{
			i = length;
		}

		RF24_TX_writePayloadAck(i, (uint8_t*) RF24_TX_buffer);

		RF24_TX_pulseTransmit();

		while (1)
		{
			if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
			{
				if (RF24_getIrqFlag(RF24_IRQ_TX))
					break;
				if (RF24_getIrqFlag(RF24_IRQ_MAX_RETRANS))
				{
					RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);
					addr[2] = g_ui32RobotID >> 16;
					addr[1] = g_ui32RobotID >> 8;
					addr[0] = g_ui32RobotID;
					RF24_RX_setAddress(RF24_PIPE0, addr);
					RF24_RX_activate();

					enableRF24Interrupt();

					return;
				}
			}
		}
		RF24_clearIrqFlag(RF24_IRQ_TX);

		if (length > 32)
			length -= 32;
		else
		{
			addr[2] = g_ui32RobotID >> 16;
			addr[1] = g_ui32RobotID >> 8;
			addr[0] = g_ui32RobotID;
			RF24_RX_setAddress(RF24_PIPE0, addr);
			RF24_RX_activate();
			GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);

			enableRF24Interrupt();

			return;
		}
	}
}

bool sendMessageToOneNeighbor(uint32_t neighborID, uint8_t * messageBuffer,
		uint32_t length)
{
	uint8_t addr[3];

	addr[2] = neighborID >> 16;
	addr[1] = neighborID >> 8;
	addr[0] = neighborID;
	RF24_RX_setAddress(RF24_PIPE0, addr);
	RF24_TX_setAddress(addr);

	uint32_t pointer = 0;
	uint32_t i;

	RF24_RX_flush();
	RF24_clearIrqFlag(RF24_IRQ_RX);
	RF24_TX_activate();
	RF24_RETRANS_setCount(RF24_RETRANS_COUNT15);
	RF24_RETRANS_setDelay(RF24_RETRANS_DELAY_2000u);

	disableRF24Interrupt();

	RF24_clearIrqFlag(RF24_IRQ_RX);
	GPIOIntClear(RF24_INT_PORT, RF24_INT_Channel);

	while (1)
	{
		rfDelayLoop(DELAY_CYCLES_1MS5);

		if (messageBuffer != RF24_TX_buffer)
		{
			for (i = 0; (i < length) && (i < 32); i++)
			{
				RF24_TX_buffer[i] = *(messageBuffer + pointer);
				pointer++;
			}
		}
		else
		{
			i = length;
		}

		RF24_TX_writePayloadAck(i, RF24_TX_buffer);

		RF24_TX_pulseTransmit();

		while (1)
		{
			if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
			{
				if (RF24_getIrqFlag(RF24_IRQ_TX))
					break;
				if (RF24_getIrqFlag(RF24_IRQ_MAX_RETRANS))
				{
					RF24_clearIrqFlag(RF24_IRQ_MAX_RETRANS);

					if (g_ui8ReTransmitCounter != 0) // software trigger reTransmit
					{
						g_ui8ReTransmitCounter++;
					}

					addr[2] = g_ui32RobotID >> 16;
					addr[1] = g_ui32RobotID >> 8;
					addr[0] = g_ui32RobotID;
					RF24_RX_setAddress(RF24_PIPE0, addr);
					RF24_RX_activate();

					enableRF24Interrupt();

					return false;
				}
				else
				{
					RF24_clearIrqFlag(RF24_IRQ_MASK);
				}
			}
		}
		RF24_clearIrqFlag(RF24_IRQ_TX);

		if (length > 32)
			length -= 32;
		else
		{
			addr[2] = g_ui32RobotID >> 16;
			addr[1] = g_ui32RobotID >> 8;
			addr[0] = g_ui32RobotID;
			RF24_RX_setAddress(RF24_PIPE0, addr);
			RF24_RX_activate();

			enableRF24Interrupt();

			return true;
		}
	}
}

void broadcastLocalNeighbor(uint8_t* pData, uint8_t ui8Length)
{
	// WARNING!: must call RF24_RX_activate() to switch back RX mode after this function servered

	uint8_t addr[3];

	RF24_TX_activate();

	addr[2] = RF24_LOCAL_BOARDCAST_BYTE2;
	addr[1] = RF24_LOCAL_BOARDCAST_BYTE1;
	addr[0] = RF24_LOCAL_BOARDCAST_BYTE0;
	RF24_TX_setAddress(addr);

	RF24_TX_writePayloadNoAck(ui8Length, pData);

	disableRF24Interrupt();

	RF24_TX_pulseTransmit();

	while (1)
	{
		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
		{
			if (RF24_getIrqFlag(RF24_IRQ_TX))
				break;
		}
	}

	RF24_clearIrqFlag(RF24_IRQ_TX);

	enableRF24Interrupt();
}

inline void testCarrierDetection()
{
	rfDelayLoop(DELAY_CYCLES_5MS);
	while (RF24_RX_carrierDetection())
	{
		GPIOPinWrite(LED_PORT_BASE, LED_BLUE, LED_BLUE);
		rfDelayLoop(DELAY_CYCLES_5MS * 25);
		GPIOPinWrite(LED_PORT_BASE, LED_BLUE, 0);
		rfDelayLoop(DELAY_CYCLES_5MS * 25);
	}
}

inline void testRfTransmission()
{
	GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_GREEN);

	uint32_t length;
	uint8_t dataLength;
	length = RF24_RX_buffer[1];
	length = length << 8;
	length = (length << 8) | RF24_RX_buffer[2];
	length = (length << 8) | RF24_RX_buffer[3];
	length = (length << 8) | RF24_RX_buffer[4];

	uint8_t value = 0;
	uint32_t i;

	while (1)
	{
		RF24_clearIrqFlag(RF24_IRQ_RX);

		while (RF24_getIrqFlag(RF24_IRQ_RX) == 0)
			;

		dataLength = RF24_RX_getPayloadWidth();
		RF24_RX_getPayloadData(dataLength, RF24_RX_buffer);

		for (i = 0; (i < length) && (i < 32); i++)
		{
			if (RF24_RX_buffer[i] != value)
			{
				GPIOPinWrite(LED_PORT_BASE, LED_RED, LED_RED);
				return;
			}
			value++;
		}

		if (length > 32)
			length -= 32;
		else
		{
			GPIOPinWrite(LED_PORT_BASE, LED_ALL, LED_RED);
			return;
		}

	}
}

inline void sendTestData()
{
	uint32_t i;
	uint16_t testData[NUMBER_OF_SAMPLE];
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		testData[i] = i;
	}
	sendDataToControlBoard((uint8_t *) testData);
}
//----------------------------------------------RF24 Functions

//----------------Low Power Mode functions-------------------
CpuStateEnum g_eCPUState = RUN_MODE;

inline void initLowPowerMode()
{
	//
	// Wake up condition form Sleep/Deep Sleep mode:
	// Any interrupt events will force CPU back to Run Mode.
	//

	//==========================
	// SLEEP MODE Configuration
	//==========================
	// In Sleep mode, the clock frequency of the active peripherals is unchanged.

	//
	// Enable Peripherals in Sleep Mode.
	//
	SysCtlPeripheralSleepEnable(RF24_INT_PORT_CLOCK); // IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
	SysCtlPeripheralSleepEnable(DELAY_TIMER_CLOCK);

	//
	// Set LDO to 1.15V in Sleep.
	// Available options: 0.9V, 0.95V, 1V, 1.05V, 1.1V, 1.15V, 1.2V
	//
	SysCtlLDOSleepSet(SYSCTL_LDO_1_15V);

	//
	// Set SRAM to Standby when in Sleep Mode.
	//
	SysCtlSleepPowerSet(SYSCTL_SRAM_STANDBY);

	//==============================
	// DEEP SLEEP MODE Configuration
	//==============================
	//
	// Set the clocking for Deep-Sleep.
	// Power down the PIOSC & MOSC to save power and run from the
	// internal 30kHz osc.
	//
	SysCtlDeepSleepClockConfigSet(1, (SYSCTL_DSLP_OSC_INT30 |
	SYSCTL_DSLP_PIOSC_PD | SYSCTL_DSLP_MOSC_PD));

	//
	// Enable Peripherals in Deep-Sleep Mode.
	//
	SysCtlPeripheralDeepSleepEnable(RF24_INT_PORT_CLOCK);// IMPORTANCE: allow IRQ pin of RF module wake CPU up when new byte has received.
	SysCtlPeripheralDeepSleepEnable(DELAY_TIMER_CLOCK);

	//
	// Set LDO to 0.9V in Deep-Sleep.
	// Available options: 0.9V, 0.95V, 1V, 1.05V, 1.1V, 1.15V, 1.2V
	//
	SysCtlLDODeepSleepSet(SYSCTL_LDO_0_90V);

	//
	// Set Flash & SRAM to Low Power in Deep-Sleep Mode.
	//
	SysCtlDeepSleepPowerSet(SYSCTL_FLASH_LOW_POWER | SYSCTL_SRAM_LOW_POWER);

	//==============================================
	// IMPORTANCE: Enable Auto Clock Gating Control.
	//==============================================
	SysCtlPeripheralClockGating(true);

	//==============================================
	// IMPORTANCE: Configure Software Interrupt
	//==============================================
	IntPrioritySet(INT_SW_TRIGGER_LPM, PRIORITY_LOW_POWER_MODE);

	IntEnable(INT_SW_TRIGGER_LPM);

	g_eCPUState = RUN_MODE;
}

inline void gotoSleepMode()
{
	//
	// NOTE: Switch clock to PIOSC and power down the MOSC before going into Sleep.
	// This will be the Run mode's clock configuration after wake up form Sleep mode.
	// So that reconfigure system clock should be considered if required.
	//
	SysCtlClockSet(SYSCTL_OSC_INT | SYSCTL_USE_OSC | SYSCTL_MAIN_OSC_DIS);

	disableMOTOR();

	turnOffLED(LED_ALL);
	turnOnLED(LED_GREEN);

	SysCtlSleep();

	turnOffLED(LED_GREEN);
}

inline void gotoDeepSleepMode()
{
	disableMOTOR();

	turnOffLED(LED_ALL);

	SysCtlDeepSleep();
}

inline void wakeUpFormLPM()
{
	SysCtlClockSet(
	SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	g_eCPUState = RUN_MODE;
	SysCtlDelay(100000);
}

//----------------------------------Low Power Mode Functions

//----------------EEPROM functions-------------------
uint32_t g_ui32EEPROMAdderss;

void initEEPROM()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while (EEPROMInit() != EEPROM_INIT_OK)
		;
	EEPROMIntDisable(EEPROM_INT_PROGRAM);
	g_ui32EEPROMAdderss = 0;
}

void writeToEEPROM()
{
	uint32_t ui32WriteAddress;
	uint32_t ui32WriteWord;

	ui32WriteAddress = RF24_RX_buffer[1] << 24;
	ui32WriteAddress = RF24_RX_buffer[2] << 16;
	ui32WriteAddress = RF24_RX_buffer[3] << 8;
	ui32WriteAddress |= RF24_RX_buffer[4];

	ui32WriteAddress <<= 2;

	ui32WriteWord = RF24_RX_buffer[5] << 24;
	ui32WriteWord |= RF24_RX_buffer[6] << 16;
	ui32WriteWord |= RF24_RX_buffer[7] << 8;
	ui32WriteWord |= RF24_RX_buffer[8];

	EEPROMProgramNonBlocking(ui32WriteWord, ui32WriteAddress);

	if (EEPROMStatusGet() == 0)
	{
		EEPROMIntClear(EEPROM_INT_PROGRAM);
	}
	else
	{
		// EEPROM operation error occur...
	}
}

void readFormEEPROM()
{
	uint8_t pui8ReadBuffer[4];
	uint32_t pui32Read[1];

	EEPROMRead(pui32Read, g_ui32EEPROMAdderss, sizeof(pui32Read));

	pui8ReadBuffer[0] = *pui32Read;
	pui8ReadBuffer[1] = (*pui32Read) >> 8;
	pui8ReadBuffer[2] = (*pui32Read) >> 16;
	pui8ReadBuffer[3] = (*pui32Read) >> 24;

	sendDataToControlBoard(pui8ReadBuffer);
}

void setAddressEEPROM()
{
	turnOnLED(LED_GREEN);

	g_ui32EEPROMAdderss = RF24_RX_buffer[1] << 24;
	g_ui32EEPROMAdderss = RF24_RX_buffer[2] << 16;
	g_ui32EEPROMAdderss = RF24_RX_buffer[3] << 8;
	g_ui32EEPROMAdderss |= RF24_RX_buffer[4];
	g_ui32EEPROMAdderss <<= 2;

	turnOffLED(LED_GREEN);
}
//-----------------------------------EEPROM functions

//----------------Smart-phone Control functions-------------------
bool tryToGetMotorsParameterInEEPROM(int8_t *pi8Motor1Speed, int8_t *pi8Motor2Speed)
{
	uint32_t pui32Read[1];

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	*pi8Motor1Speed = *pui32Read;
	*pi8Motor2Speed = (*pui32Read) >> 8;

	if(*pi8Motor1Speed < 0 || *pi8Motor1Speed > 99 || *pi8Motor2Speed < 0 || *pi8Motor2Speed > 99)
		return false;

	return true;
}

void goStraight()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(FORWARD, i8Motor1Speed, FORWARD, i8Motor2Speed);
	}
}

void goBackward()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(REVERSE, i8Motor1Speed, REVERSE, i8Motor2Speed);
	}
}

void spinClockwise()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(REVERSE, i8Motor1Speed, FORWARD, i8Motor2Speed);
	}
}

void spinCounterclockwise()
{
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	if (tryToGetMotorsParameterInEEPROM(&i8Motor1Speed, &i8Motor2Speed))
	{
		configureMotors(FORWARD, i8Motor1Speed, REVERSE, i8Motor2Speed);
	}
}

//-----------------------------------Smart-phone Control functions

