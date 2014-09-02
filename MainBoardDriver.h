#ifndef MAIN_BOARD_DRIVERS_H
#define MAIN_BOARD_DRIVERS_H

#include "arm_math.h"

#define DELAY_START_SPEAKER	100000
#define DELAY_SAMPING_MIC	 90000

/*
 * Priority level: 0x00 = 0x20 :: 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0
 */

//#define PRIORITY_RF24_IRQ			0x00 // CANNOT change this priority in here!
#define PRIORITY_DMA_BATT			0x20
#define PRIORITY_DMA_RANDOM_GEN		0x20
#define PRIORITY_DMA_MIC1			0x40
#define PRIORITY_DMA_MIC2			0x40
#define PRIORITY_DELAY_TIMERA		0x60
#define PRIORITY_DELAY_TIMERB		0x60
#define PRIORITY_ROBOT_PROCESS		0x80
#define PRIORITY_LOW_POWER_MODE		0xE0

#define INT_SW_TRIGGER_LPM			INT_I2C1
#define INT_SW_TRIGGER_PROCESS		INT_I2C2

//----------------Robot Init functions-------------------
#define EEPROM_ADDR_ROBOT_ID			0x0040

void initRobotProcess();
//-----------------------------------Robot Int functions


//----------------Math functions-------------------
#define MATH_PI 				3.141592654
#define MATH_PI_DIV_2			1.570796327
#define MATH_PI_DIV_2_MUL_32768	51471.85404
#define _180_DIV_PI				57.29577951
#define EPPROM_SINE_TABLE_ADDRESS       0x0080  // Block 2
#define EPPROM_ARC_SINE_TABLE_ADDRESS   0x0200  // Block 5

int32_t calSin(float x);
int32_t calCos(float x);
int32_t calASin(float x);
int32_t calACos(float x);
//-----------------------------------Math functions


//----------------Delay Timer functions-------------------
#define DELAY_TIMER_CLOCK	SYSCTL_PERIPH_WTIMER0
#define DELAY_TIMER_BASE	WTIMER0_BASE
#define INT_DELAY_TIMERA	INT_WTIMER0A
#define INT_DELAY_TIMERB	INT_WTIMER0B

extern volatile bool g_bDelayTimerAFlag;
extern volatile bool g_bDelayTimerBFlag;

void initTimerDelay();
void delayTimerA(uint32_t period, bool isSynchronous);
void delayTimerB(uint32_t period, bool isSynchronous);
void reloadDelayTimerA();
void reloadDelayTimerB();
//-----------------------------------Delay Timer functions


//-----------------------LED functions-------------------------
#define LED_CLOCK_PORT          SYSCTL_PERIPH_GPIOF
#define LED_PORT_BASE           GPIO_PORTF_BASE
#define LED_RED                 GPIO_PIN_1
#define LED_BLUE                GPIO_PIN_2
#define LED_GREEN               GPIO_PIN_3
#define LED_ALL                 (LED_RED | LED_GREEN | LED_BLUE)

inline void initLED();
inline void turnOnLED(uint8_t LEDpin);
inline void turnOffLED(uint8_t LEDpin);
inline void toggleLED(uint8_t LEDpin);

void signalUnhandleError();
//------------------------------------------------LED functions


//--------------------------PWM Defs---------------------------
#define PWM_CLOCK_SELECT        SYSCTL_PWMDIV_16
#define PWM_CLOCK_PRESCALE      16 // Must match with PWM_CLOCK_SELECT
//-----------------------------------------------------PWM Defs


//----------------------Speaker Functions------------------------
#define SPEAKER_PORT_BASE               GPIO_PORTF_BASE
#define SPEAKER_PORT_CLOCK              SYSCTL_PERIPH_GPIOF
#define SPEAKER_PIN                     GPIO_PIN_0
#define SPEAKER_PWM_CLOCK_BASE          SYSCTL_PERIPH_PWM1
#define SPEAKER_PWM_FREQUENCY           8000                    // tan so phat
#define SPEAKER_PWM_BASE                PWM1_BASE
#define SPEAKER_PWM_CONFIG              GPIO_PF0_M1PWM4
#define SPEAKER_PWM_GEN                 PWM_GEN_2
#define SPEAKER_PWM_GEN_BIT				PWM_GEN_2_BIT
#define SPEAKER_PWM_OUT                 PWM_OUT_4
#define SPEAKER_PWM_OUT_BIT             PWM_OUT_4_BIT
#define SPEAKER_TIMER_CLOCK             SYSCTL_PERIPH_TIMER1
#define SPEAKER_TIMER_BASE              TIMER1_BASE
#define SPEAKER_TIMER_FREQUENCY         4000                   // thoi gian phat
#define SPEAKER_INT                     INT_TIMER1A

inline void initSpeaker();
inline void startSpeaker();
//-----------------------------------------------Speaker functions


//-----------------------Motor functions-----------------------
#define MOTOR_PWM_CLOCK         SYSCTL_PERIPH_PWM0
#define MOTOR_PWM_FREQUENCY     1000
#define MOTOR_PWM_BASE          PWM0_BASE

#define MOTOR_SLEEP_PIN_CLOCK   SYSCTL_PERIPH_GPIOD
#define MOTOR_SLEEP_PIN_BASE    GPIO_PORTD_BASE
#define MOTOR_SLEEP_PIN         GPIO_PIN_0

#define LEFT_MOTOR_PORT_CLOCK   	SYSCTL_PERIPH_GPIOE
#define LEFT_MOTOR_PORT_BASE    	GPIO_PORTE_BASE
#define LEFT_MOTOR_IN1          	GPIO_PIN_5
#define LEFT_MOTOR_IN2          	GPIO_PIN_4
#define LEFT_MOTOR_PWM_CONFIG   	GPIO_PE5_M0PWM5
#define LEFT_MOTOR_PWM_GEN      	PWM_GEN_2
#define LEFT_MOTOR_PWM_OUT1     	PWM_OUT_5
#define LEFT_MOTOR_PWM_OUT2     	PWM_OUT_4
#define LEFT_MOTOR_PWM_OUT1_BIT 	PWM_OUT_5_BIT
#define LEFT_MOTOR_PWM_OUT2_BIT 	PWM_OUT_4_BIT

#define RIGHT_MOTOR_PORT_CLOCK          SYSCTL_PERIPH_GPIOB
#define RIGHT_MOTOR_PORT_BASE           GPIO_PORTB_BASE
#define RIGHT_MOTOR_IN1                 GPIO_PIN_5
#define RIGHT_MOTOR_IN2                 GPIO_PIN_4
#define RIGHT_MOTOR_PWM_CONFIG          GPIO_PB5_M0PWM3
#define RIGHT_MOTOR_PWM_GEN             PWM_GEN_1
#define RIGHT_MOTOR_PWM_OUT1            PWM_OUT_3
#define RIGHT_MOTOR_PWM_OUT2            PWM_OUT_2
#define RIGHT_MOTOR_PWM_OUT1_BIT        PWM_OUT_3_BIT
#define RIGHT_MOTOR_PWM_OUT2_BIT        PWM_OUT_2_BIT

#define FORWARD         0
#define REVERSE         1

inline void initMotor();
inline void enableMOTOR();
inline void disableMOTOR();
inline void setMotorDirection(uint32_t motor, uint8_t direction);
inline void testAllMotorModes();
inline void setMotorSpeed(uint32_t motorPortOut, uint8_t speed);
//----------------------------------------------Motor functions


//--------------------------------Ananlog functions-----------------------------------
#define NUMBER_OF_SAMPLE    300
#define SAMPLE_FREQUENCY    100000

#define ADC_PORT_CLOCK          	SYSCTL_PERIPH_GPIOE
#define ADC_PORT                	GPIO_PORTE_BASE
#define ADC0_IN                 	GPIO_PIN_1
#define ADC1_IN                 	GPIO_PIN_3
#define ADC_ADJACENT_PINS       	GPIO_PIN_2 | GPIO_PIN_0
#define ADC0_CHANNEL            	ADC_CTL_CH2
#define ADC1_CHANNEL            	ADC_CTL_CH0
#define ADC0_INT                	INT_ADC0SS3
#define ADC1_INT                	INT_ADC1SS3
#define ADC_TIMER_CLOCK         	SYSCTL_PERIPH_TIMER0
#define ADC_TIMER               	TIMER0_BASE
#define ADC_AVERAGING_FACTOR    	8
#define ADC_SEQUENCE_TYPE       	3
#define ADC_SEQUENCE_ADDRESS    	ADC_O_SSFIFO3
#define ADC0_DMA_CHANNEL       		UDMA_CHANNEL_ADC3
#define ADC1_DMA_CHANNEL        	UDMA_SEC_CHANNEL_ADC13
#define DMA_ADC0_CHANNEL        	UDMA_CH17_ADC0_3
#define DMA_ADC1_CHANNEL       		UDMA_CH27_ADC1_3
#define DISTANCE_SENSING_PRIORITY	0

#define BATTERY_PORT_CLOCK				SYSCTL_PERIPH_GPIOD
#define BATTERY_PORT					GPIO_PORTD_BASE
#define BATTERY_IN						GPIO_PIN_1
#define BATTERY_CHANNEL 				ADC_CTL_CH6
#define BATTERY_MEASURENMENT_PRIORITY	1
#define ADC_BATT_BASE					ADC0_BASE
#define ADC_BATT_SEQUENCE_TYPE     	 	2	// IMPORTANCE note: make sure this sequence type NOT equal ADC_SEQUENCE_TYPE
#define ADC_BATT_SEQUENCE_ADDRESS    	ADC_O_SSFIFO2
#define	ADC_BATT_INT	 				INT_ADC0SS2
#define BATT_DMA_CHANNEL       			UDMA_CHANNEL_ADC2
#define DMA_BATT_CHANNEL        		UDMA_CH16_ADC0_2

#define ADC_RANDOM_GEN_BASE			    ADC1_BASE
#define ADC_RANDOM_GEN_SEQUENCE_TYPE	0
#define RANDOM_GEN_PRIORITY				1
#define RANDOM_GEN_CHANNEL				ADC1_CHANNEL
#define RANDOM_GEN_SEQUENCE_ADDRESS		ADC_O_SSFIFO0
#define RANDOM_GEN_DMA_CHANNEL			UDMA_SEC_CHANNEL_ADC10
#define DMA_RANDOM_GEN_CHANNEL			UDMA_CH24_ADC1_0
#define RANDOM_GEN_INT                	INT_ADC1SS0

extern uint16_t g_ui16BatteryVoltage;
extern uint16_t g_pui16ADC0Result[];
extern uint16_t g_pui16ADC1Result[];
extern uint8_t g_pui8RandomBuffer[];
extern uint8_t g_ui8RandomNumber;

inline void initPeripheralsForAnalogFunction(void);
inline void startSamplingMicSignals();
inline void startSamplingBatteryVoltage();
inline void generateRandomByte();
//-------------------------------------------------------------------Ananlog functions


//----------------------------------------------TDOA functions----------------------------------------------
#define FILTER_ORDER                    34
#define BLOCK_SIZE                      10
#define START_SAMPLES_POSTITION         40
#define NUM_BLOCKS                      (NUMBER_OF_SAMPLE / BLOCK_SIZE)
#define NUM_DATAS                       (NUMBER_OF_SAMPLE - START_SAMPLES_POSTITION)

inline void initFilters(float32_t* FilterCoeffs);
void runAlgorithmTDOA();

float32_t getDistances(float32_t *myData, float32_t *peakEnvelope, float32_t *maxEnvelope);
void find3LocalPeaks(float32_t *myData, float32_t *LocalPeaksStoragePointer);
uint32_t reachBottom(float32_t *myData, uint32_t const PeakPosition,
                uint32_t const PointerIncreaseNumber);
uint32_t reachPeak(float32_t *myData, uint32_t const PeakPosition,
                uint32_t const PointerIncreaseNumber);
void interPeak(float32_t* PositionsArray, float32_t* ValuesArray, float32_t UserPosition,
                float32_t UserMaxValue, float32_t const step, float32_t* ReturnPosition,
                float32_t* ReturnValue);
float32_t larange(float32_t *PositionsArray, float32_t *ValuesArray, float32_t interpolatePoint);
//--------------------------------------------------------------------------------------------TDOA functions


//----------------------RF24 Functions------------------------
#define PC_TEST_RF_TRANSMISSION         0xC0
#define PC_TOGGLE_ALL_STATUS_LEDS       0xC1
#define PC_START_SAMPLING_MIC           0xC2
#define PC_TEST_ALL_MOTOR_MODES         0xC3
#define PC_CHANGE_MOTORS_SPEED          0xC4
#define PC_TEST_RF_CARRIER_DETECTION    0xC5
#define PC_SEND_TEST_DATA_TO_PC         0xC6
#define PC_START_SPEAKER                0xC8

#define PC_SEND_DATA_ADC0_TO_PC         0xA0
#define PC_SEND_DATA_ADC1_TO_PC         0xA1
#define PC_SEND_BATT_VOLT_TO_PC			0xA3

#define PC_SEND_STOP_MOTOR_LEFT			0xA4
#define PC_SEND_STOP_MOTOR_RIGHT		0xA5

#define PC_SEND_MEASURE_DISTANCE		0xB0

#define PC_SEND_READ_EEPROM             0xE0
#define PC_SEND_WRITE_EEPROM            0xE1
#define PC_SEND_SET_ADDRESS_EEPROM      0xE2

#define ROBOT_REQUEST_SAMPLING_MIC		0xD0
#define ROBOT_CONFIRM_MEASURE_DISCTANCE	0xD1

#define COMMAND_RESET			0x01
#define COMMAND_SLEEP			0x02
#define	COMMAND_DEEP_SLEEP		0x03
#define	COMMAND_WAKE_UP			0x04

inline void initRfModule();
inline void testRfTransmission();
void sendDataToControlBoard(uint8_t * data);
inline void testCarrierDetection();
inline void sendTestData();
void broadcastLocalNeighbor(uint8_t* pData, uint8_t ui8Length);

//----------------------------------------------RF24 Functions


//----------------------Low Power Mode Functions------------------------
typedef enum
{
  RUN_MODE,
  SLEEP_MODE,
  DEEP_SLEEP_MODE,
} CpuStateEnum;

extern CpuStateEnum  g_eCPUState;	// Low Power Mode State

//-----------------------------------------------------------------------------
//  void initLowPowerMode()
//
//  DESCRIPTION:	This function configure clock system when enter Deep Sleep mode
//					and enable some necessary peripherals keep operate in Sleep/Deep Sleep mode.
//					Flash and SRAM power consumption on each state also modified.					
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
inline void initLowPowerMode();

//-----------------------------------------------------------------------------
//  void gotoSleepMode()
//
//  DESCRIPTION: 	Switch clock to PIOSC and power down the MOSC before going into Sleep. 
//					This will be the Run mode's clock configuration after wake up form Sleep mode. 
// 					So that reconfigure system clock should be considered if required.
//					After all, enter Sleep Mode.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
inline void gotoSleepMode();

//-----------------------------------------------------------------------------
//  void gotoDeepSleepMode()
//
//  DESCRIPTION:	This function forces CPU enter Deep Sleep Mode.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
inline void gotoDeepSleepMode();

//-----------------------------------------------------------------------------
//  void wakeUpFormLPM()
//
//  DESCRIPTION:	Reconfigure System clock before return normal operation.
//
//  ARGUMENTS: None
//
//  RETURN VALUE: None
//
//-----------------------------------------------------------------------------
inline void wakeUpFormLPM();

//----------------------------------------------Low Power Mode Functions


//----------------------EEPROM Functions------------------------
void initEEPROM();
void writeToEEPROM();
void readFormEEPROM();
void setAddressEEPROM();
//----------------------------------------------EEPROM Functions


#endif /* MAIN_BOARD_DRIVERS_H */
