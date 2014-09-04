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
#include "CustomTivaDrivers.h"
#include "MainBoardDriver.h"

volatile bool g_bDelayTimerAFlag;
volatile bool g_bDelayTimerBFlag;

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
	g_bDelayTimerAFlag = false;

	timerALastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, timerALastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_A);

	if (isSynchronous)
		while (!g_bDelayTimerAFlag)
			;
}

void reloadDelayTimerA()
{
	TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, timerALastDelayPeriod);
}

void delayTimerB(uint32_t period, bool isSynchronous)
{
	g_bDelayTimerBFlag = false;

	timerBLastDelayPeriod = SysCtlClockGet() / 1000 * period;

	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);

	TimerEnable(DELAY_TIMER_BASE, TIMER_B);

	if (isSynchronous)
		while (!g_bDelayTimerBFlag)
			;
}

void reloadDelayTimerB()
{
	TimerLoadSet(DELAY_TIMER_BASE, TIMER_B, timerBLastDelayPeriod);
}

//----------------Math functions-------------------
int32_t calSin(float x)
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

	return (int32_t) (resultSigned * (int32_t) pui16ReadBuffer[selectResult]);
}

int32_t calCos(float x)
{
	return calSin(x + MATH_PI_DIV_2);
}

int32_t calASin(float x)
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

	return (int32_t) (resultSigned * (int32_t) pui16ReadBuffer[selectResult]);
}

int32_t calACos(float x)
{
	return (MATH_PI_DIV_2_MUL_32768 - calASin(x));
}
//-----------------------------------Math functions

//----------------Robot Init functions-------------------
extern uint32_t g_ui32RobotID;
extern RobotMeasStruct Neighbors[];
extern OneHopMeasStruct DisctanceTable[];
extern uint8_t g_ui8ReadTablePosition;

 void initRobotProcess()
{
	//==============================================
	// Get Robot ID form EEPROM
	//==============================================
	EEPROMRead(&g_ui32RobotID, EEPROM_ADDR_ROBOT_ID, sizeof(&g_ui32RobotID));

}

 void sendNeighborsTableToControlBoard() {
	 uint8_t buffer[8];
	 uint32_t tempDistance = Neighbors[g_ui8ReadTablePosition].distance * 32768;

	 buffer[0] = Neighbors[g_ui8ReadTablePosition].ID >> 24;
	 buffer[1] = Neighbors[g_ui8ReadTablePosition].ID >> 16;
	 buffer[2] = Neighbors[g_ui8ReadTablePosition].ID >> 8;
	 buffer[3] = Neighbors[g_ui8ReadTablePosition].ID;

	 buffer[4] = tempDistance >> 24;
	 buffer[5] = tempDistance >> 16;
	 buffer[6] = tempDistance >> 8;
	 buffer[7] = tempDistance;

	 sendDataToControlBoard(buffer);
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
	while (1)
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

inline void testAllMotorModes()
{
	uint8_t motorDutyCycles;

	enableMOTOR();
	turnOnLED(LED_RED);

	//=========================Test LEFT Motor=========================
	setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// speed up
	turnOnLED(LED_GREEN);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_GREEN);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);

	// speed up
	turnOnLED(LED_BLUE);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_BLUE);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}
	//==================================================Test LEFT Motor

	//=========================Test RIGHT Motor=========================
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	// speed up
	turnOnLED(LED_GREEN);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_GREEN);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);

	// speed up
	turnOnLED(LED_BLUE);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_BLUE);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}
	//==================================================Test RIGHT Motor

	//=========================Test BOTH Motors=========================
	setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
	setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// speed up
	turnOnLED(LED_GREEN);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_GREEN);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	setMotorDirection(RIGHT_MOTOR_PORT_BASE, REVERSE);
	setMotorDirection(LEFT_MOTOR_PORT_BASE, REVERSE);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// speed up
	turnOnLED(LED_BLUE);
	motorDutyCycles = 100;
	while (motorDutyCycles > 0)
	{
		motorDutyCycles--;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}

	// slow down
	turnOffLED(LED_BLUE);
	motorDutyCycles = 0;
	while (motorDutyCycles < 100)
	{
		motorDutyCycles++;
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				motorDutyCycles * ui32PWMPeriod / 100);
		SysCtlDelay(500000);
	}
	//==================================================Test BOTH Motors

	disableMOTOR();
	turnOffLED(LED_RED);
}

inline void setMotorSpeed(uint32_t motorPortOut, uint8_t speed)
{
	PWMPulseWidthSet(MOTOR_PWM_BASE, motorPortOut,
			(speed * ui32PWMPeriod) / 100);
}
//----------------------------------------------Motor functions

//--------------------------------Ananlog functions-----------------------------------
static unsigned char countAdcDMAsStopped = 0;

static uint32_t g_ui32uDMAErrCount = 0;

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

//----------------------------------------------TDOA functions----------------------------------------------
// Initialize two filter, input and output buffer pointers
float32_t OutputMicA[NUMBER_OF_SAMPLE] = { 0 };
float32_t OutputMicB[NUMBER_OF_SAMPLE] = { 0 };

float32_t SamplesMicA[NUMBER_OF_SAMPLE] = { 0 };
float32_t SamplesMicB[NUMBER_OF_SAMPLE] = { 0 };

arm_fir_instance_f32 FilterA;
arm_fir_instance_f32 FilterB;
static float32_t pStateA[BLOCK_SIZE + FILTER_ORDER - 1] =
{ 0 };
static float32_t pStateB[BLOCK_SIZE + FILTER_ORDER - 1] =
{ 0 };

extern float32_t g_f32PeakEnvelopeA;
extern float32_t g_f32MaxEnvelopeA;
extern float32_t g_f32PeakEnvelopeB;
extern float32_t g_f32MaxEnvelopeB;

void initFilters(float32_t* FilterCoeffs)
{
	// Call FIR init function to initialize the instance structure.
	arm_fir_init_f32(&FilterA, FILTER_ORDER, FilterCoeffs, pStateA, BLOCK_SIZE);
	arm_fir_init_f32(&FilterB, FILTER_ORDER, FilterCoeffs, pStateB, BLOCK_SIZE);
}
void runAlgorithmTDOA()
{
//	float32_t OutputMicA[NUMBER_OF_SAMPLE] = { 0 };
//	float32_t OutputMicB[NUMBER_OF_SAMPLE] = { 0 };
//
//	float32_t SamplesMicA[NUMBER_OF_SAMPLE] = { 0 };
//	float32_t SamplesMicB[NUMBER_OF_SAMPLE] = { 0 };

	int i;

	while (countAdcDMAsStopped != 2)
		;

	// Convert g_ui32ADC0/1Result to SamplesMicA/B
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		SamplesMicA[i] = g_pui16ADC0Result[i];
		SamplesMicB[i] = g_pui16ADC1Result[i];
	}

	// Filter signals
	for (i = 0; i < NUM_BLOCKS; i++)
	{
		arm_fir_f32(&FilterA, SamplesMicA + (i * BLOCK_SIZE),
				OutputMicA + (i * BLOCK_SIZE),
				BLOCK_SIZE);
		arm_fir_f32(&FilterB, SamplesMicB + (i * BLOCK_SIZE),
				OutputMicB + (i * BLOCK_SIZE),
				BLOCK_SIZE);
	}

	// Drop some invalid samples at the begin output buffer
	for (i = 0; i < NUMBER_OF_SAMPLE; i++)
	{
		OutputMicA[i] = OutputMicA[i + START_SAMPLES_POSTITION];
		OutputMicB[i] = OutputMicB[i + START_SAMPLES_POSTITION];
	}

	getDistances(OutputMicA, &g_f32PeakEnvelopeA, &g_f32MaxEnvelopeA);
	getDistances(OutputMicB, &g_f32PeakEnvelopeB, &g_f32MaxEnvelopeB);
}

float32_t getDistances(float32_t *myData, float32_t *peakEnvelope,
		float32_t *maxEnvelope)
{
	float32_t step = +0.125f;
	float32_t localPeaksPosition[3] =
	{ 0 };
	float32_t localMaxValue[3] =
	{ 0 };

	find3LocalPeaks(myData, localPeaksPosition);
	if (localPeaksPosition[0] != 0 && localPeaksPosition[1] != 0
			&& localPeaksPosition[2] != 0)
	{
		float32_t PositionsArray[3] =
		{ 0 };
		float32_t ValuesArray[3] =
		{ 0 };
		int i;
		for (i = 0; i < 3; i++)
		{
			PositionsArray[0] = localPeaksPosition[i] - 1;
			PositionsArray[1] = localPeaksPosition[i];
			PositionsArray[2] = localPeaksPosition[i] + 1;
			ValuesArray[0] = *(myData + (uint32_t) PositionsArray[0]);
			ValuesArray[1] = *(myData + (uint32_t) PositionsArray[1]);
			ValuesArray[2] = *(myData + (uint32_t) PositionsArray[2]);
			localMaxValue[i] = *(myData + (uint32_t) localPeaksPosition[i]);
			interPeak(PositionsArray, ValuesArray, localPeaksPosition[i],
					localMaxValue[i], step, &localPeaksPosition[i],
					&localMaxValue[i]);
		}
		interPeak(localPeaksPosition, localMaxValue, localPeaksPosition[1],
				localMaxValue[1], step, peakEnvelope, maxEnvelope);
	}
	else
		return 0;       //signal error

	// return (0.379 * (*peakEnvelope + START_SAMPLES_POSTITION) - 8.734676667);
	return 1;
}
void find3LocalPeaks(float32_t *myData, float32_t* LocalPeaksStoragePointer)
{
	uint32_t SamplePosition = 0;
	uint32_t maxSamplePosition = 0;
	int i;
	for (i = START_SAMPLES_POSTITION; i < NUM_DATAS; i++)
	{
		if (*(myData + i) > *(myData + maxSamplePosition))
		{
			maxSamplePosition = i;
		}
	}
	LocalPeaksStoragePointer[1] = maxSamplePosition;
	SamplePosition = reachBottom(myData, LocalPeaksStoragePointer[1], -1);
	if (SamplePosition != 0)
	{
		LocalPeaksStoragePointer[0] = reachPeak(myData, SamplePosition, -1);
	}
	SamplePosition = reachBottom(myData, LocalPeaksStoragePointer[1], 1);
	if (SamplePosition != 0)
	{
		LocalPeaksStoragePointer[2] = reachPeak(myData, SamplePosition, 1);
	}
}
uint32_t reachBottom(float32_t *myData, uint32_t const PeakPosition,
		uint32_t const PointerIncreaseNumber)
{
	uint32_t SamplePosition = PeakPosition;
	while (SamplePosition > 1 && SamplePosition < NUM_DATAS)
	{
		if (*(myData + SamplePosition)
				< *(myData + SamplePosition + PointerIncreaseNumber))
		{
			return SamplePosition;
		}
		else
		{
			SamplePosition += PointerIncreaseNumber;
		}
	}
	SamplePosition = 0;
	return 0;
}
uint32_t reachPeak(float32_t *myData, uint32_t const PeakPosition,
		uint32_t const PointerIncreaseNumber)
{
	uint32_t SamplePosition = PeakPosition;
	while (SamplePosition > 1 && SamplePosition < NUM_DATAS)
	{
		if (*(myData + SamplePosition)
				> *(myData + SamplePosition + PointerIncreaseNumber))
		{
			return SamplePosition;
		}
		else
		{
			SamplePosition += PointerIncreaseNumber;
		}
	}
	SamplePosition = 0;
	return 0;
}
void interPeak(float32_t* PositionsArray, float32_t* ValuesArray,
		float32_t UserPosition, float32_t UserMaxValue, float32_t const step,
		float32_t* ReturnPosition, float32_t* ReturnValue)
{
	float32_t realLocalPeak = UserPosition;
	float32_t realLocalMax = UserMaxValue;
	float32_t samplePosition = realLocalPeak - step;
	float32_t interpolateValue = larange(PositionsArray, ValuesArray,
			samplePosition);
	float32_t PointerDirection = 0;
	if (interpolateValue > UserMaxValue)
	{
		PointerDirection = -1;
		realLocalPeak = samplePosition;
		realLocalMax = interpolateValue;
	}
	else
	{
		PointerDirection = 1;
	}
	int flag = 1;
	while (flag)
	{
		samplePosition = realLocalPeak + step * PointerDirection;
		interpolateValue = larange(PositionsArray, ValuesArray, samplePosition);
		if (interpolateValue >= realLocalMax)
		{
			realLocalMax = interpolateValue;
			realLocalPeak = samplePosition;
		}
		else
		{
			*ReturnPosition = realLocalPeak;
			*ReturnValue = realLocalMax;
			flag = 0;
		}
	}
}
float32_t larange(float32_t *PositionsArray, float32_t *ValuesArray,
		float32_t interpolatePoint)
{
	float32_t result = 0;
	int i, j;
	float32_t temp;

	for (j = 0; j < 3; j++)
	{
		temp = 1;
		for (i = 0; i < 3; i++)
		{
			if (i != j)
				temp = (interpolatePoint - (*(PositionsArray + i))) * temp
						/ ((*(PositionsArray + j)) - (*(PositionsArray + i)));
		}
		result = result + (*(ValuesArray + j) * temp);
	}

	return result;
}
//--------------------------------------------------------------------------------------------TDOA functions

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
extern uint8_t RF24_RX_buffer[32];

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

	uint8_t buffer[32];
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
	while (1)
	{
		rfDelayLoop(DELAY_CYCLES_1MS5);

		for (i = 0; (i < length) && (i < 32); i++)
		{
			buffer[i] = *(data + pointer);
			pointer++;
		}

		RF24_TX_writePayloadAck(i, &buffer[0]);

		disableRF24Interrupt();

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
extern uint32_t g_ui32EEPROMAdderss;

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
