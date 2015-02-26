#ifndef MAIN_BOARD_DRIVERS_H
#define MAIN_BOARD_DRIVERS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "libnrf24l01/inc/nRF24L01.h"

#include "arm_math.h"

#define DISTANCE_BETWEEN_TWO_MICS		5.0
#define DISTANCE_BETWEEN_TWO_MICS_SQR	25.0

#define DELAY_START_SPEAKER	1000


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


void clearNeighborTable(robotMeas_t neighborTable[], uint8_t *counter);
void clearOneHopNeighborTable(oneHopMeas_t table[]);
void deleteNeighborInInNeighborTable(robotMeas_t table[], uint32_t id);
void deleteNeighborInOneHopTable(oneHopMeas_t table[], uint32_t id);
void deleteNeighborInLocationsTable(location_t table[], uint32_t id);
void updateNeighborInLocationTable(uint32_t neigborId, float xAxis, float yAxis);
void deleteTable(robotMeas_t neighborTable[]);

void parse32BitTo4Bytes(uint32_t value, uint8_t *buffer);
//uint32_t construct4BytesToUint32(uint8_t *buffer);
//int32_t construct4BytesToInt32(uint8_t *buffer);
void delayRandom(uint32_t parameterUnit);



void rotateClockwiseWithAngle(float angle);
void runForwardWithDistance(float distance);

//----------------Robot Init functions-------------------

#define REBROADCAST_TIMES	5


typedef enum
{
	DONE = 0, TDOA = 1,
} RobotResponseState_t;

void debugBreakpoint();

//void initRobotProcess();
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


//--------------------------PWM Defs---------------------------
#define PWM_CLOCK_SELECT        SYSCTL_PWMDIV_16
#define PWM_CLOCK_PRESCALE      16 // Must match with PWM_CLOCK_SELECT
//-----------------------------------------------------PWM Defs


#ifdef __cplusplus
}
#endif

#endif /* MAIN_BOARD_DRIVERS_H */
