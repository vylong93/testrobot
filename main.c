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
#include "CustomTivaDrivers.h"
#include "MainBoardDriver.h"

//*****************************************************************************
// Robot ID
//*****************************************************************************
uint32_t g_ui32RobotID;

//*****************************************************************************
// The buffer to receive data from the RF module
//*****************************************************************************
volatile uint8_t RF24_RX_buffer[32] =
{ 0 };
volatile uint8_t RF24_TX_buffer[32] =
{ 0 };

//*****************************************************************************
// Motors Speed
//*****************************************************************************
volatile uint8_t ui8LeftMotorDutyCycles;
volatile uint8_t ui8RightMotorDutyCycles;

//*****************************************************************************
// The buffers for ADCs
//*****************************************************************************
uint16_t g_pui16ADC0Result[NUMBER_OF_SAMPLE];
uint16_t g_pui16ADC1Result[NUMBER_OF_SAMPLE];

uint8_t g_pui8RandomBuffer[8];
uint8_t g_ui8RandomNumber;

uint16_t g_ui16BatteryVoltage;

uint32_t g_ui32EEPROMAdderss;

float32_t g_f32PeakEnvelopeA = 0;
float32_t g_f32MaxEnvelopeA = 0;
float32_t g_f32PeakEnvelopeB = 0;
float32_t g_f32MaxEnvelopeB = 0;

float32_t* FilterCoeffs;

typedef enum
{
	IDLE, MEASURE_DISTANCE, EXCHANGE_TABLE, LOCALIZATION
} ProcessStateEnum;

ProcessStateEnum g_eProcessState = IDLE;

typedef struct tagRobotMeas
{
	float32_t distance;
	uint32_t ID;
} RobotMeasStruct;

typedef struct tagOneHopMeas
{
	uint32_t firstHopID;
	RobotMeasStruct neighbors[10];
} OneHopMeasStruct;

OneHopMeasStruct DisctanceTable[10];
RobotMeasStruct Neighbors[10];
uint8_t neighborsCounter = 0;

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
	// External Crystal 16MHz source for PLL (400MHz/2)/4 = 50MHz sysclk
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

	initFilters(FilterCoeffs);

	while (1)
	{
		rfDelayLoop(DELAY_CYCLES_5MS * 25);
		toggleLED(LED_RED);
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
			else
			{
				switch (RF24_RX_buffer[0])
				{
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
					GPIOPinToggle(LED_PORT_BASE, LED_ALL);
					break;

				case PC_TEST_ALL_MOTOR_MODES:
					// testAllMotorModes();
					//TODO: modified to set random direction with random speed
					break;

				case PC_CHANGE_MOTORS_SPEED:
					disableMOTOR();
					PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
					PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

					setMotorDirection(LEFT_MOTOR_PORT_BASE, RF24_RX_buffer[1]);
					setMotorSpeed(LEFT_MOTOR_PWM_OUT1, RF24_RX_buffer[2]);
					setMotorDirection(RIGHT_MOTOR_PORT_BASE, RF24_RX_buffer[3]);
					setMotorSpeed(RIGHT_MOTOR_PWM_OUT1, RF24_RX_buffer[4]);

					PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
					PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

					PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
					true);
					PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
					true);
					enableMOTOR();
					break;

				case PC_SEND_STOP_MOTOR_LEFT:
					PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
					setMotorDirection(LEFT_MOTOR_PORT_BASE, FORWARD);
					PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT,
					false);
					break;

				case PC_SEND_STOP_MOTOR_RIGHT:
					PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
					setMotorDirection(RIGHT_MOTOR_PORT_BASE, FORWARD);
					PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT,
					false);
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

				case PC_SEND_MEASURE_DISTANCE:

					turnOffLED(LED_ALL);

					g_eProcessState = MEASURE_DISTANCE;

					delayTimerA(5000, false); // Asychronous delay 5 second

					IntTrigger(INT_SW_TRIGGER_PROCESS);

					break;

				case ROBOT_REQUEST_SAMPLING_MIC:
					// DO NOT INSERT ANY CODE IN HERE!
					startSamplingMicSignals();

//					turnOnLED(LED_GREEN);
//
//					reloadDelayTimerA();
//
//					runAlgorithmTDOA();

					//TODO: check threshold g_f32MaxEnvelopeA and g_f32MaxEnvelopeB make sure is valid neighbor

					Neighbors[neighborsCounter].ID = (RF24_RX_buffer[1] << 24)
							| (RF24_RX_buffer[2] << 16)
							| (RF24_RX_buffer[3] << 8) | RF24_RX_buffer[4];

//					Neighbors[neighborsCounter].distance = (g_f32PeakEnvelopeA
//							+ g_f32PeakEnvelopeB) / 2;
//
//					neighborsCounter++;
//					neighborsCounter = (neighborsCounter < 10) ? (neighborsCounter) : (0);
//
//					turnOffLED(LED_GREEN);

					break;

				case ROBOT_CONFIRM_MEASURE_DISCTANCE:
					//TODO:
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

				default:
					signalUnhandleError();
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

void RobotProcessIntHandler(void)
{
	bool hasSentCommand = false;
	uint16_t ui16RandomValue;

	switch (g_eProcessState)
	{
	case MEASURE_DISTANCE:
		do
		{
			if (!hasSentCommand)
			{

				turnOnLED(LED_BLUE);

				generateRandomByte();

				while (g_ui8RandomNumber == 0)
					;
				g_ui8RandomNumber =
						(g_ui8RandomNumber < 100) ?
								(g_ui8RandomNumber + 100) : (g_ui8RandomNumber);
				ui16RandomValue = (g_ui32RobotID << 8) | g_ui8RandomNumber;

				delayTimerB(ui16RandomValue, false);

				while (!g_bDelayTimerBFlag)
				{
					if (Neighbors[neighborsCounter].ID != 0)
					{
						turnOnLED(LED_GREEN);

						reloadDelayTimerA();

						runAlgorithmTDOA();

						//TODO: check threshold g_f32MaxEnvelopeA and g_f32MaxEnvelopeB make sure is valid neighbor

						Neighbors[neighborsCounter].distance =
								(g_f32PeakEnvelopeA + g_f32PeakEnvelopeB) / 2;

						neighborsCounter++;
						neighborsCounter =
								(neighborsCounter < 10) ?
										(neighborsCounter) : (0);

						turnOffLED(LED_GREEN);
					}
				}

				turnOffLED(LED_BLUE);

				reloadDelayTimerA();

				// delay timeout - send request

				RF24_TX_buffer[0] = ROBOT_REQUEST_SAMPLING_MIC;
				RF24_TX_buffer[1] = g_ui32RobotID >> 24;
				RF24_TX_buffer[2] = g_ui32RobotID >> 16;
				RF24_TX_buffer[3] = g_ui32RobotID >> 8;
				RF24_TX_buffer[4] = g_ui32RobotID;
				broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
				// DO NOT INSERT ANY CODE IN HERE!
				ROM_SysCtlDelay(DELAY_START_SPEAKER);
				startSpeaker();

				RF24_RX_activate();

				hasSentCommand = true;

				//		//set TimerB 2s timeout, reset Timer everytime received RF data
				//		delayTimerB(2000, true);
				//
				//		// send confirm if timeout
				//		RF24_TX_buffer[0] = ROBOT_CONFIRM_MEASURE_DISCTANCE;
				//		RF24_TX_buffer[1] = g_ui32RobotID >> 24;
				//		RF24_TX_buffer[2] = g_ui32RobotID >> 16;
				//		RF24_TX_buffer[3] = g_ui32RobotID >> 8;
				//		RF24_TX_buffer[4] = g_ui32RobotID;
				//		broadcastLocalNeighbor(RF24_TX_buffer, 5);
			}
		} while (!g_bDelayTimerAFlag);

		//TODO: exchange table - State 2 start here
		turnOnLED(LED_ALL);

		break;

	case EXCHANGE_TABLE:
		break;

	case LOCALIZATION:
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
	g_bDelayTimerAFlag = true;
}

void DelayTimerBIntHanler()
{
	TimerIntClear(DELAY_TIMER_BASE, TIMER_TIMB_TIMEOUT);
	g_bDelayTimerBFlag = true;
}
