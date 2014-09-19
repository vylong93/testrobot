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

uint8_t RF24_RX_buffer[32] =
{ 0 };
uint8_t RF24_TX_buffer[32] =
{ 0 };

uint8_t g_pui8RandomBuffer[8];
uint8_t g_ui8RandomNumber = 0;

uint8_t g_ui8ReTransmitCounter; // set this variable to 0 to disable software reTransmit

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
	g_bDelayTimerAFlagAssert = false;

	timerALastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, timerALastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_A);

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
	g_bDelayTimerBFlagAssert = false;

	timerBLastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_B);

	if (isSynchronous)
		while (!g_bDelayTimerBFlagAssert)
			;
}

void reloadDelayTimerB()
{
	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);
}


//----------------Math functions-------------------
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

	angleX = (tempX > 360) ? (tempX - 360.0) : (tempX);

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

bool isValidTriangle(uint32_t a, uint32_t b, uint32_t c)
{
	float cosA;
	float cosB;
	float cosC;

	// Long Dang, Sep 16, 2014 ======================
//	float fa = ((a / 256.0) - INTERCEPT) / SLOPE;
//	float fb = ((b / 256.0) - INTERCEPT) / SLOPE;
//	float fc = ((c / 256.0) - INTERCEPT) / SLOPE;

	float fa = a / 256.0f;
	float fb = b / 256.0f;
	float fc = c / 256.0f;
	//====================== Long Dang, Sep 16, 2014

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

oneHopMeas_t OneHopNeighborsTable[ONEHOP_NEIGHBOR_TABLE_LENGTH];
robotMeas_t NeighborsTable[NEIGHBOR_TABLE_LENGTH];
location_t locs[LOCATIONS_TABLE_LENGTH];

uint32_t g_ui8LocsCounter = 0;
uint8_t g_ui8ReadTablePosition;
uint8_t g_ui8ReadOneHopTablePosition;
uint8_t g_ui8NeighborsCounter;

uint32_t g_ui32RobotID;

ProcessStateEnum g_eProcessState;

extern location_t locs[];

bool g_bBypassThisState;

void initRobotProcess()
{
	//==============================================
	// Get Robot ID form EEPROM
	//==============================================
	EEPROMRead(&g_ui32RobotID, EEPROM_ADDR_ROBOT_ID, sizeof(&g_ui32RobotID));

	g_eProcessState = IDLE;
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

	for (g_ui8ReadTablePosition = 0;
			g_ui8ReadTablePosition < NEIGHBOR_TABLE_LENGTH;
			g_ui8ReadTablePosition++)
	{
		if (NeighborsTable[g_ui8ReadTablePosition].ID == neighborID)
		{
			RF24_TX_buffer[0] = ROBOT_RESPONSE_HELLO_NEIGHBOR;

			RF24_TX_buffer[1] = g_ui32RobotID >> 24;
			RF24_TX_buffer[2] = g_ui32RobotID >> 16;
			RF24_TX_buffer[3] = g_ui32RobotID >> 8;
			RF24_TX_buffer[4] = g_ui32RobotID;

			tableSizeInByte = sizeof(NeighborsTable);

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

void sendNeighborsTableToControlBoard()
{
	RF24_TX_buffer[0] = NeighborsTable[g_ui8ReadTablePosition].ID >> 24;
	RF24_TX_buffer[1] = NeighborsTable[g_ui8ReadTablePosition].ID >> 16;
	RF24_TX_buffer[2] = NeighborsTable[g_ui8ReadTablePosition].ID >> 8;
	RF24_TX_buffer[3] = NeighborsTable[g_ui8ReadTablePosition].ID;

	RF24_TX_buffer[4] = NeighborsTable[g_ui8ReadTablePosition].distance >> 8;
	RF24_TX_buffer[5] = NeighborsTable[g_ui8ReadTablePosition].distance;

	sendDataToControlBoard(RF24_TX_buffer);
}

void sendLocationsTableToControlBoard()
{
	int32_t temp;

	RF24_TX_buffer[0] = locs[g_ui8ReadTablePosition].ID >> 24;
	RF24_TX_buffer[1] = locs[g_ui8ReadTablePosition].ID >> 16;
	RF24_TX_buffer[2] = locs[g_ui8ReadTablePosition].ID >> 8;
	RF24_TX_buffer[3] = locs[g_ui8ReadTablePosition].ID;

	temp = locs[g_ui8ReadTablePosition].vector.x * 32768;
	RF24_TX_buffer[4] = temp >> 24;
	RF24_TX_buffer[5] = temp >> 16;
	RF24_TX_buffer[6] = temp >> 8;
	RF24_TX_buffer[7] = temp;

	temp = locs[g_ui8ReadTablePosition].vector.y * 32768;
	RF24_TX_buffer[8] = temp >> 24;
	RF24_TX_buffer[9] = temp >> 16;
	RF24_TX_buffer[10] = temp >> 8;
	RF24_TX_buffer[11] = temp;

	sendDataToControlBoard(RF24_TX_buffer);
}

void sendOneHopNeighborsTableToControlBoard()
{
	RF24_TX_buffer[0] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].firstHopID >> 24;
	RF24_TX_buffer[1] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].firstHopID >> 16;
	RF24_TX_buffer[2] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].firstHopID >> 8;
	RF24_TX_buffer[3] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].firstHopID;

	RF24_TX_buffer[4] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].ID
					>> 24;
	RF24_TX_buffer[5] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].ID
					>> 16;
	RF24_TX_buffer[6] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].ID
					>> 8;
	RF24_TX_buffer[7] =
			OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].ID;

	RF24_TX_buffer[8] = OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].distance >> 8;
	RF24_TX_buffer[9] = OneHopNeighborsTable[g_ui8ReadOneHopTablePosition].neighbors[g_ui8ReadTablePosition].distance;

	sendDataToControlBoard(RF24_TX_buffer);
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
	SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(MOTOR_PWM_CLOCK);

	uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	ui32PWMPeriod = (pwmClock / MOTOR_PWM_FREQUENCY);

	SysCtlPeripheralEnable(MOTOR_SLEEP_PIN_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeGPIOOutput(MOTOR_SLEEP_PIN_BASE, MOTOR_SLEEP_PIN);

	SysCtlPeripheralEnable(LEFT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
	GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG);
	GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);

	SysCtlPeripheralEnable(RIGHT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
	GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG);
	GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);

	PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, 0);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);

	PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN); // M0PWM3
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

inline void setMotorDirection(uint32_t motorPortBase, uint8_t direction)
{
	if (direction == FORWARD)
		GPIOPinWrite(motorPortBase, LEFT_MOTOR_IN2, 0);
	else if (direction == REVERSE)
		GPIOPinWrite(motorPortBase, LEFT_MOTOR_IN2, LEFT_MOTOR_IN2);
}

inline void randomMotorMode()
{
//	while(bMotorRandomMode)
//	{
//		disableMOTOR();
//
//		PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
//		PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
//
//		generateRandomByte();
//		while(g_ui8RandomNumber != 0);
//
//		setMotorDirection(LEFT_MOTOR_PORT_BASE, g_ui8RandomNumber & 0x01);
//		setMotorDirection(RIGHT_MOTOR_PORT_BASE, (g_ui8RandomNumber >> 1) & 0x01);
//
//		setMotorSpeed(LEFT_MOTOR_PWM_OUT1, ((g_ui8RandomNumber << 3) & 0xE0) + 40);
//		setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, (g_ui8RandomNumber & 0xE0) + 40);
//
//		PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
//		PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
//
//		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
//		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
//
//		enableMOTOR();
//
//		toggleLED(LED_GREEN);
//
//		delayTimerB(g_ui8RandomNumber + 2000, true);

	setSpinSpeed(65, 1, 3000);
	setSpinSpeed(65, 0, 3000);
	setSpinSpeed(65, 1, 3000);
	setSpinSpeed(65, 0, 3000);
//	}
}

void setSpinSpeed(uint8_t speed, uint8_t direction, uint32_t delay)
{
	disableMOTOR();

	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	if (direction & 0x01)	// REVERSE 1
	{
		setMotorDirection(LEFT_MOTOR_PORT_BASE, direction);
		setMotorSpeed(LEFT_MOTOR_PWM_OUT1, 100 - speed);

		setMotorDirection(RIGHT_MOTOR_PORT_BASE, direction ^ 1);
		setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, speed);
	}
	else	//FORWARD 0
	{
		setMotorDirection(LEFT_MOTOR_PORT_BASE, direction); // direction = 0
		setMotorSpeed(LEFT_MOTOR_PWM_OUT1, speed);

		setMotorDirection(RIGHT_MOTOR_PORT_BASE, direction ^ 1); // direction = 1
		setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, 100 - speed);
	}

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
	enableMOTOR();

	ROM_SysCtlDelay((SysCtlClockGet() / 1000) * (delay / 3));

	disableMOTOR();
}

inline void setMotorSpeed(uint32_t motorPortOut, uint8_t speed)
{
	PWMPulseWidthSet(MOTOR_PWM_BASE, motorPortOut,
			(speed * ui32PWMPeriod) / 100);
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

	turnOffLED(LED_ALL);
	turnOnLED(LED_GREEN);

	SysCtlSleep();

	turnOffLED(LED_GREEN);
}

inline void gotoDeepSleepMode()
{
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
void goStraight()
{
	uint32_t pui32Read[1];
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = *pui32Read;
	i8Motor2Speed = (*pui32Read) >> 8;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	disableMOTOR();
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
	setMotorSpeed(LEFT_MOTOR_PWM_OUT1, i8Motor1Speed);
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
	setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, i8Motor2Speed);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
	true);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
	true);
	enableMOTOR();
}

void spinClockwise()
{
	uint32_t pui32Read[1];
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = *pui32Read;
	i8Motor2Speed = (*pui32Read) >> 8;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	disableMOTOR();
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);
	setMotorSpeed(LEFT_MOTOR_PWM_OUT1, 100 - i8Motor1Speed);
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
	setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, i8Motor2Speed);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
	true);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
	true);
	enableMOTOR();
}

void spinCounterclockwise()
{
	uint32_t pui32Read[1];
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = *pui32Read;
	i8Motor2Speed = (*pui32Read) >> 8;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	disableMOTOR();
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
	setMotorSpeed(LEFT_MOTOR_PWM_OUT1, i8Motor1Speed);
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);
	setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, 100 - i8Motor2Speed);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
	true);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
	true);
	enableMOTOR();
}

void goBackward()
{
	uint32_t pui32Read[1];
	int8_t i8Motor1Speed;
	int8_t i8Motor2Speed;

	EEPROMRead(pui32Read, EEPROM_ADDR_MOTOR_OFFSET, sizeof(pui32Read));

	i8Motor1Speed = *pui32Read;
	i8Motor2Speed = (*pui32Read) >> 8;

	if(i8Motor1Speed < 0 || i8Motor1Speed > 99 || i8Motor2Speed < 0 || i8Motor2Speed > 99)
		return;

	disableMOTOR();
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);
	setMotorSpeed(LEFT_MOTOR_PWM_OUT1, 100 - i8Motor1Speed);
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);
	setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, 100 - i8Motor2Speed);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
	true);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
	true);
	enableMOTOR();
}

//-----------------------------------Smart-phone Control functions

