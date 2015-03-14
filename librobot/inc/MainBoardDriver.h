#ifndef MAIN_BOARD_DRIVERS_H
#define MAIN_BOARD_DRIVERS_H

#include "libnrf24l01/inc/nRF24L01.h"

#include "arm_math.h"

#define DISTANCE_BETWEEN_TWO_MICS		5.0
#define DISTANCE_BETWEEN_TWO_MICS_SQR	25.0

#define DELAY_START_SPEAKER	1000
#define DELAY_SAMPING_MIC	1

// These following definition use for 32-bit Timer delay, 1 stand for 1ms
// so the range must between 1ms to 85s (85000ms)
#define DELAY_MEASURE_DISTANCE_STATE 500 	// move to exchange table state timeout period
#define DELAY_EXCHANGE_TABLE_STATE	 2000	// move to next state timeout period
#define DELAY_GET_TABLE_PERIOD	 	 1000	// wait for neighbor check his table and send to me
#define DELAY_ROTATE_NETWORK		 4000
#define DELAY_REBROADCAST			 2000
#define DELAY_GET_FLAG_PERIOD		 1000
#define DELAY_GET_VECTOR_PERIOD		 1000
#define DELAY_LOCOMOTION_PERIOD	     1250
#define DELAY_T_SHAPE_PERIOD	     5000
#define DELAY_NEIGHBOR_RESPONSE_PERIOD	500 // must be < DELAY_LOCOMOTION_PERIOD

#define NEIGHBOR_TABLE_LENGTH 10
#define ONEHOP_NEIGHBOR_TABLE_LENGTH NEIGHBOR_TABLE_LENGTH
#define LOCATIONS_TABLE_LENGTH	NEIGHBOR_TABLE_LENGTH

typedef struct tagRobotMeas
{
	uint32_t ID;
	uint16_t distance; // <8.8>
} robotMeas_t;

typedef struct tagOneHopMeas
{
	uint32_t firstHopID;
	robotMeas_t neighbors[NEIGHBOR_TABLE_LENGTH];
} oneHopMeas_t;

typedef struct tagVector {
	float x;
	float y;
} vector2_t;

typedef struct tagLocation {
	uint32_t ID;
	vector2_t vector;
} location_t;


/*
 * Priority level: 0x00 = 0x20 :: 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0
 */

//#define PRIORITY_RF24_IRQ			0x00 // CANNOT change this priority in here!
#define PRIORITY_DMA_BATT			0x00

#define PRIORITY_DMA_RANDOM_GEN		0x40
#define PRIORITY_DMA_MIC1			0x40
#define PRIORITY_DMA_MIC2			0x40

#define PRIORITY_MOTOR_TIMERA       0x60
#define PRIORITY_MOTOR_TIMERB       0x60

#define PRIORITY_DELAY_TIMERA		0x80
#define PRIORITY_DELAY_TIMERB		0x80

#define PRIORITY_ROBOT_RESPONSE		0xA0

#define PRIORITY_LOW_POWER_MODE		0xE0

#define INT_SW_TRIGGER_LPM				INT_I2C1
#define INT_SW_TRIGGER_ROBOT_RESPONSE	INT_I2C2

void clearNeighborTable(robotMeas_t neighborTable[], uint8_t *counter);
void clearOneHopNeighborTable(oneHopMeas_t table[]);
void deleteNeighborInInNeighborTable(robotMeas_t table[], uint32_t id);
void deleteNeighborInOneHopTable(oneHopMeas_t table[], uint32_t id);
void deleteNeighborInLocationsTable(location_t table[], uint32_t id);
void updateNeighborInLocationTable(uint32_t neigborId, float xAxis, float yAxis);
void deleteTable(robotMeas_t neighborTable[]);

void parse32BitTo4Bytes(uint32_t value, uint8_t *buffer);
uint32_t construct4BytesToUint32(uint8_t *buffer);
int32_t construct4BytesToInt32(uint8_t *buffer);
void delayRandom(uint32_t parameterUnit);


//----------------Robot Init functions-------------------
#define EEPROM_ADDR_ROBOT_ID			0x0040
#define EEPROM_ADDR_MOTOR_OFFSET		0x0044	// EEPROM_ADDR_ROBOT_ID + 4

#define EEPROM_INTERCEPT				0x0048
#define EEPROM_SLOPE					0x004C

#define REBROADCAST_TIMES	5

typedef enum
{
	IDLE = 0, MEASURE_DISTANCE = 1, EXCHANGE_TABLE = 2, VOTE_ORIGIN = 3, ROTATE_NETWORK = 4, REDUCE_ERROR = 5, LOCOMOTION = 6, T_SHAPE = 7,
} ProcessState_t;

typedef enum
{
	DONE = 0, TDOA = 1,
} RobotResponseState_t;

void debugBreakpoint();

void initRobotProcess();
void addToNeighborTable(uint32_t neighborId, uint16_t distance);
void responseTDOAResultsToNeighbor(uint32_t neighborId);
void storeNeighorVectorAndDistanceToTables(uint8_t RxData[]);
void checkAndResponeMyNeighborsTableToOneRobot();
void sendVectorToControlBoard();
void sendNeighborsTableToControlBoard();
void sendLocationsTableToControlBoard();
void sendOneHopNeighborsTableToControlBoard();
void rotateClockwiseTest(uint8_t RxData[]);
void rotateClockwiseAngleTest(uint8_t RxData[]);
void forwardPeriodTest(uint8_t RxData[]);
void forwardDistanceTest(uint8_t RxData[]);
void responseCorrectionAngleAndOriented();
void getNeighborNeighborsTable();
void updateOrRejectNetworkOrigin(uint8_t RxData[]);
bool isNeedRotateCoordinate(uint8_t originNumberOfNeighbors, uint32_t originID);
void getHopOriginTableAndRotate(uint8_t RxData[]);
int32_t isLocationTableContainID(uint32_t id, location_t table[], uint8_t length);
uint32_t tryToGetCommonNeighborID(location_t firstTable[], uint8_t firstTableLength, location_t secondTable[], uint8_t secondTableLength);
float getAngleFromTable(uint32_t id, location_t table[], uint8_t length);
void rotateLocationTable(float angle, bool mirror, location_t table[], uint8_t length);
void calculateRealVector(vector2_t vector, location_t table[], uint8_t length);
void updateGradient(vector2_t *pVectGradienNew, bool enableRandomCal);
void updatePosition(vector2_t *pvectAverageCoordination, vector2_t *pvectEstimatePosNew, vector2_t *pvectEstimatePosOld, vector2_t *pvectGradienNew, vector2_t *pectGradienOld, float fStepSize);
bool checkVarianceCondition(vector2_t vectNew, vector2_t vectOld, float fCondition);
void updateLocsByOtherRobotCurrentPosition(bool isFirstInit);
void synchronousLocsTableAndMyVector();
void tryToResponeNeighborVector();
void tryToResponeVectorAndFlag();
void tryToResponseVector();
bool getMyVector(uint8_t *pCounter);
bool getNeighborVectorAndFlag(vector2_t *pvectReceived, bool* pIsNeighborGradientSearchStop, bool *pIsNeighborActive);
bool getNeighborVector(uint32_t neighborID);
void clearRequestNeighbor(uint8_t RxData[]);
void updateNeighborVectorInLocsTableByRequest(uint8_t RxData[]);
void runForwardAndCalculatteNewPosition(float distance);
int8_t calculateQuadrant(vector2_t vectSource, vector2_t vectDestinate);
float calculateRobotOrientation(vector2_t vectDiff);
float calculateRobotAngleWithXAxis(vector2_t vectDiff);
float calculateTotalDistance(vector2_t v1, vector2_t v2, vector2_t v3, vector2_t v4, uint32_t *selectedResult);
float calculateDistanceBetweenTwoNode(vector2_t v1, vector2_t v2);
void notifyNewVectorToNeigbors();

//-----------------------------------Robot Int functions


//----------------Math functions-------------------
#define MATH_PI_MUL_2			6.283185307
#define MATH_PI_MUL_3_DIV_2	    4.71238898
#define MATH_PI 				3.141592654
#define MATH_PI_DIV_2			1.570796327
#define MINUS_MATH_PI_DIV_2		-1.570796327
#define MATH_PI_DIV_2_MUL_32768	51471.85404
#define _180_DIV_PI				57.29577951
#define EPPROM_SINE_TABLE_ADDRESS       0x0080  // Block 2
#define EPPROM_ARC_SINE_TABLE_ADDRESS   0x0200  // Block 5

//#define ANGLE_MIN_IN_RAD	0.15	// ~ 8.594366927 degree
//#define COSINE_ANGLE_MIN	0.9887710779 // cos(ANGLE_MIN_IN_RAD)

#define ANGLE_MIN_IN_RAD	0.1047197551 // ~ 6 degree
#define COSINE_ANGLE_MIN	0.9945218954 // cos(ANGLE_MIN_IN_RAD)

//#define ANGLE_MIN_IN_RAD	0.1745329252 // ~ 10 degree
//#define COSINE_ANGLE_MIN	0.9999953604 // cos(ANGLE_MIN_IN_RAD)

float vsqrtf(float op1);
float calSin(float x);
float calCos(float x);
float calASin(float x);
float calACos(float x);
float cosinesRuleForTriangles(float a, float b, float c);
bool isTriangle(float a, float b, float c);
bool isValidTriangle(uint32_t a, uint32_t b, uint32_t c);
//-----------------------------------Math functions


//----------------Delay Timer functions-------------------

// Interrupt Timers
#define DELAY_TIMER_CLOCK	SYSCTL_PERIPH_WTIMER0
#define DELAY_TIMER_BASE	WTIMER0_BASE
#define INT_DELAY_TIMERA	INT_WTIMER0A
#define INT_DELAY_TIMERB	INT_WTIMER0B

// Non-Interrtup Timer
#define DELAY_TIMER_CLOCK_NON_INT	SYSCTL_PERIPH_TIMER2
#define DELAY_TIMER_BASE_NON_INT	TIMER2_BASE

void initTimerDelay();
void delayTimerA(uint32_t period, bool isSynchronous);
void delayTimerB(uint32_t period, bool isSynchronous);
void reloadDelayTimerA();
void reloadDelayTimerB();
void delayTimerNonInt(uint32_t period);
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
#define MIN_MOTOR_DUTYCYCLE		1
#define MAX_MOTOR_DUTYCYCLE		90

#define MOTOR_PWM_CLOCK         SYSCTL_PERIPH_PWM0
#define MOTOR_PWM_FREQUENCY     15000
#define MOTOR_PWM_BASE          PWM0_BASE

#define MOTOR_SLEEP_PIN_CLOCK   SYSCTL_PERIPH_GPIOD
#define MOTOR_SLEEP_PIN_BASE    GPIO_PORTD_BASE
#define MOTOR_SLEEP_PIN         GPIO_PIN_0

// LEFT Motor's pin
#define LEFT_MOTOR_PORT_CLOCK   	SYSCTL_PERIPH_GPIOE
#define LEFT_MOTOR_PORT_BASE    	GPIO_PORTE_BASE

#define LEFT_MOTOR_PWM_GEN      	PWM_GEN_2

#define LEFT_MOTOR_IN1          	GPIO_PIN_5
#define LEFT_MOTOR_PWM_CONFIG1   	GPIO_PE5_M0PWM5
#define LEFT_MOTOR_PWM_OUT1     	PWM_OUT_5
#define LEFT_MOTOR_PWM_OUT1_BIT 	PWM_OUT_5_BIT

#define LEFT_MOTOR_IN2          	GPIO_PIN_4
#define LEFT_MOTOR_PWM_CONFIG2   	GPIO_PE4_M0PWM4
#define LEFT_MOTOR_PWM_OUT2     	PWM_OUT_4
#define LEFT_MOTOR_PWM_OUT2_BIT 	PWM_OUT_4_BIT

// RIGHT Motor's pin
#define RIGHT_MOTOR_PORT_CLOCK          SYSCTL_PERIPH_GPIOB
#define RIGHT_MOTOR_PORT_BASE           GPIO_PORTB_BASE

#define RIGHT_MOTOR_PWM_GEN             PWM_GEN_1

#define RIGHT_MOTOR_IN1                 GPIO_PIN_5
#define RIGHT_MOTOR_PWM_CONFIG1         GPIO_PB5_M0PWM3
#define RIGHT_MOTOR_PWM_OUT1            PWM_OUT_3
#define RIGHT_MOTOR_PWM_OUT1_BIT        PWM_OUT_3_BIT

#define RIGHT_MOTOR_IN2                 GPIO_PIN_4
#define RIGHT_MOTOR_PWM_CONFIG2         GPIO_PB4_M0PWM2
#define RIGHT_MOTOR_PWM_OUT2            PWM_OUT_2
#define RIGHT_MOTOR_PWM_OUT2_BIT        PWM_OUT_2_BIT

//#define FORWARD         0
//#define REVERSE         1

typedef enum
{
  FORWARD = 0,
  REVERSE = 1,
} MotorDirection_t;

inline void initMotor();
inline void enableMOTOR();
inline void disableMOTOR();

void setMotorLeftDirectionAndSpeed(uint8_t direction, uint8_t speed);
void setMotorRightDirectionAndSpeed(uint8_t direction, uint8_t speed);

void configureMotors(uint8_t left_Direction, uint8_t left_dutyCycles, uint8_t right_Direction, uint8_t right_dutyCycles);

void stopMotorLeft();
void stopMotorRight();
void stopMotors();

void rotateClockwiseWithAngle(float angle);
void runForwardWithDistance(float distance);

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

inline void initPeripheralsForAnalogFunction(void);
inline void startSamplingMicSignals();
inline void startSamplingBatteryVoltage();
inline void generateRandomByte();
float generateRandomRange(float min, float max);

//-------------------------------------------------------------------Ananlog functions


//----------------------RF24 Functions------------------------
#define COMMAND_RESET					0x01
#define COMMAND_SLEEP					0x02
#define	COMMAND_DEEP_SLEEP				0x03
#define	COMMAND_WAKE_UP					0x04

#define PC_SEND_DATA_ADC0_TO_PC         0xA0
#define PC_SEND_DATA_ADC1_TO_PC         0xA1

#define PC_SEND_BATT_VOLT_TO_PC			0xA3
#define PC_SEND_STOP_MOTOR_LEFT			0xA4
#define PC_SEND_STOP_MOTOR_RIGHT		0xA5
#define PC_SEND_READ_NEIGHBORS_TABLE	0xA6
#define PC_SEND_READ_ONEHOP_TABLE		0xA7
#define PC_SEND_READ_LOCS_TABLE			0xA8

#define PC_SEND_MEASURE_DISTANCE		0xB0
#define PC_SEND_READ_VECTOR				0xB1
#define PC_SEND_LOCAL_LOOP_STOP			0xB2
#define PC_SEND_SET_STEPSIZE			0xB3
#define PC_SEND_SET_STOP_CONDITION_ONE	0xB4
#define PC_SEND_SET_STOP_CONDITION_TWO	0xB5
#define PC_SEND_ROTATE_CLOCKWISE		0xB6
#define PC_SEND_ROTATE_CLOCKWISE_ANGLE	0xB7
#define PC_SEND_FORWARD_PERIOD			0xB8
#define PC_SEND_FORWARD_DISTANCE		0xB9
#define PC_SEND_SET_ROBOT_STATE			0xBA
#define PC_SEND_ROTATE_CORRECTION_ANGLE	0xBB
#define PC_SEND_READ_CORRECTION_ANGLE	0xBC

#define PC_SEND_ROTATE_CORRECTION_ANGLE_DIFF	0xBD
#define PC_SEND_ROTATE_CORRECTION_ANGLE_SAME	0xBE

#define PC_TEST_RF_TRANSMISSION         0xC0
#define PC_TOGGLE_ALL_STATUS_LEDS       0xC1
#define PC_START_SAMPLING_MIC           0xC2
#define PC_CHANGE_MOTORS_SPEED          0xC4
#define PC_TEST_RF_CARRIER_DETECTION    0xC5
#define PC_SEND_TEST_DATA_TO_PC         0xC6
#define PC_START_SPEAKER                0xC8

#define ROBOT_REQUEST_SAMPLING_MIC				0xD0
#define ROBOT_REQUEST_NEIGHBORS_TABLE 			0xD1
#define ROBOT_RESPONSE_HELLO_NEIGHBOR			0xD2
#define ROBOT_RESPONSE_NOT_YOUR_NEIGHBOR 		0xD3
#define ROBOT_REQUEST_UPDATE_NETWORK_ORIGIN		0xD4
#define ROBOT_REQUEST_ROTATE_NETWORK			0xD5
#define ROBOT_REQUEST_MY_VECTOR					0xD6
#define ROBOT_RESPONSE_MY_VECTOR_PLEASE_WAIT	0xD7
#define ROBOT_RESPONSE_MY_VECTOR				0xD8
#define ROBOT_RESPONSE_MY_VECTOR_NOT_FOUND 		0xD9
#define ROBOT_REQUEST_VECTOR_AND_FLAG			0xDA
#define ROBOT_RESPONSE_VECTOR_AND_FLAG			0xDB
#define ROBOT_RESPONSE_PLEASE_WAIT				0xDC
#define ROBOT_RESPONSE_UNACTIVE					0xDD
#define ROBOT_REQUEST_VECTOR					0xDE
#define ROBOT_RESPONSE_VECTOR					0xDF

#define ROBOT_REQUEST_TO_RUN				0x90
#define ROBOT_RESPONSE_TDOA_DISTANCE		0x92
#define ROBOT_REQUEST_UPDATE_VECTOR			0x93
#define ROBOT_ALLOW_MOVE_TO_T_SHAPE			0x94
#define ROBOT_REPONSE_MOVE_COMPLETED		0x95

#define PC_SEND_READ_EEPROM             0xE0
#define PC_SEND_WRITE_EEPROM            0xE1
#define PC_SEND_SET_ADDRESS_EEPROM      0xE2

// Format <SMART_PHONE_COMMAND><SP_SEND_...>
#define SMART_PHONE_COMMAND				0xF0
#define SP_SEND_STOP_TWO_MOTOR			0xF1
#define SP_SEND_FORWAR					0xF2
#define SP_SEND_SPIN_CLOCKWISE			0xF3
#define SP_SEND_SPIN_COUNTERCLOCKWISE	0xF4
#define SP_SEND_RESERVED				0xF5

inline void initRfModule();
inline void testRfTransmission();
void sendDataToControlBoard(uint8_t * data);
bool sendMessageToOneNeighbor(uint32_t neighborID, uint8_t * messageBuffer, uint32_t length);
inline void testCarrierDetection();
inline void sendTestData();
void broadcastLocalNeighbor(uint8_t* pData, uint8_t ui8Length);

//----------------------------------------------RF24 Functions


//----------------------Low Power Mode Functions------------------------
typedef enum
{
  RUN_MODE,
  SLEEP_MODE,
  DEEP_SLEEP_MODE
} CpuState_t;

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


//----------------Smart-phone Control functions-------------------
bool tryToGetMotorsParameterInEEPROM(int8_t *pi8Motor1Speed, int8_t *pi8Motor2Speed);
void goStraight();
void spinClockwise();
void spinCounterclockwise();
void goBackward();
//-----------------------------------Smart-phone Control functions



#endif /* MAIN_BOARD_DRIVERS_H */
