#include <stdint.h>
#include <stdbool.h>

#include "libcustom/inc/custom_clock.h"
#include "libcustom/inc/custom_led.h"
#include "libcustom/inc/custom_delay.h"
#include "libcustom/inc/custom_ir.h"

#include "libcustom/inc/custom_uart_debug.h"

#include "librobot/inc/robot_lpm.h"
#include "librobot/inc/robot_task_timer.h"
#include "librobot/inc/robot_speaker.h"
#include "librobot/inc/robot_analog.h"
#include "librobot/inc/robot_motor.h"
#include "librobot/inc/robot_eeprom.h"
#include "librobot/inc/robot_communication.h"
#include "librobot/inc/robot_process.h"

#include "libstorage/inc/robot_data.h"

#include "libalgorithm/inc/Trilateration.h"
#include "libalgorithm/inc/GradientMap.h"

#include "libmath/inc/custom_math.h"
#include <math.h>

#define HAVE_IMU

#ifdef HAVE_IMU
#include "libcustom/inc/custom_i2c.h"
#include "librobot/inc/robot_imu.h"
#endif

extern "C" {
	void MCU_RF_IRQ_handler(void);
	void FaultISRHandler(void);
}

bool g_bIsRfFlagAsserted;

void initSystem(void);

int main(void)
{
	initSystem();

	initRobotProcess();

	turnOffLED(LED_ALL);

#ifdef REGION_COMMENT
//	NeighborsTable_clear();
//	OneHopNeighborsTable_clear();
//	RobotLocationsTable_clear();
//	initDataOfRobot1();
//	RobotLocationsTable_add(Network_getSelfAddress(), 0, 0);
//
//	uint32_t t0, t1;
//
//	uint32_t ClockSpeed = ROM_SysCtlClockGet();
//	ROM_TimerLoadSet(TASK_TIMER_BASE, TIMER_A, ClockSpeed);
//	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
//	ROM_TimerEnable(TASK_TIMER_BASE, TIMER_A);
//
//	t0 = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//	Tri_tryToCalculateRobotLocationsTable(0xBEAD01);
//	t1 = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//
//	ROM_TimerDisable(TASK_TIMER_BASE, TIMER_A);
//	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
//	test();
#endif

#ifdef HAVE_IMU
//	turnOnLED(LED_GREEN);
//	DEBUG_PRINT("delay 30s for DMP output stablized...\n");
//	delay_ms(30000);
//	turnOffLED(LED_GREEN);
//	turnOnLED(LED_RED);
//	DEBUG_PRINT("IMU DMP: Ready\n");
#endif

#ifdef REGION_COMMENT
//	// System Variables
//	Motor_t m1_Left;
//	Motor_t m2_Right;
//	uint8_t ui8NextPWM;
//
//	// Memmory Storages
//	float fYawAngleInRad = 0;
//	float e_old = 0;
//	float E = 0;
//
//	// Controller Variables
//	Vector3<float> vect3YawPitchRoll(0, 0, 0);
//	float fOriginR = 0;
//	float fReferenceYawAngleInRad = 0;
//	float e = 0;
//	float e_dot = 0;
//	float u = 0;
//
//	// Initialize
//	// get reference for PID Controller
////	fReferenceYawAngleInRad = IMU_getYawAngle();
////	fOriginR = fReferenceYawAngleInRad;
//
//	// Setup motor
//	m1_Left.eDirection = FORWARD;
//	m2_Right.eDirection = REVERSE;
//	m1_Left.ui8Speed = u;
//	m2_Right.ui8Speed = u;
//
//	while(1)
//	{
//		stopMotors();
//		fReferenceYawAngleInRad = IMU_getYawAngle();
//		fOriginR = fReferenceYawAngleInRad;
//
//		while(!bIsRunPID);
//		e = 0;
//		e_old = 0;
//		e_dot = 0;
//		E = 0;
//		fReferenceYawAngleInRad = fOriginR + r;
//		while(bIsRunPID)
//		{
//			/* PID controll robot: moving forward in straight line */
//			// read current yaw angle
//			fYawAngleInRad = IMU_getYawAngle();
//
//			// calculate the error
//			e = fYawAngleInRad - fReferenceYawAngleInRad;
//
//			// Because e is an angle so need extra-handle
//			e = atan2f(sinf(e), cosf(e));
//
//			// calculate the error dynamic
//			e_dot = e - e_old;
//
//			// integral the error
//			E = E + e;
//
//			// compute the control input
//			u = kP*e + kI*E + kD*e_dot;
//
//			// save 'e' for the next loop
//			e_old = e;
//
//
//			/* PWM control signal */
//			if(u < 0)
//			{
//				u = 0 - u;
//				m1_Left.eDirection = REVERSE;
//				m2_Right.eDirection = FORWARD;
//			}
//			else
//			{
//				m1_Left.eDirection = FORWARD;
//				m2_Right.eDirection = REVERSE;
//			}
//			ui8NextPWM = u * 100;
//
//			if(ui8NextPWM < MOTOR_SPEED_MINIMUM)
//				ui8NextPWM = 0;
//
//			if(ui8NextPWM > MOTOR_SPEED_MAXIMUM)
//				ui8NextPWM = MOTOR_SPEED_MAXIMUM;
//
//			m1_Left.ui8Speed = ui8NextPWM;
//			m2_Right.ui8Speed = ui8NextPWM;
//
//			configureMotors(m1_Left, m2_Right);
//
//			DEBUG_PRINTS3("u = %d, %d %d \n", (int32_t)(u * 63356 + 0.5f),
//										  (int32_t)(m1_Left.eDirection),
//										  (int32_t)(m1_Left.ui8Speed));
//
//			delay_ms(10);
//		}
//	}
#endif

	while(true)
	{
		switch (getRobotState())
		{
			case ROBOT_STATE_MEASURE_DISTANCE:
				DEBUG_PRINT("goto State ONE: Measure Distance\n");
				StateOne_MeasureDistance();
			break;

			case ROBOT_STATE_EXCHANGE_TABLE:
				DEBUG_PRINT("goto State TWO: Exchange Table\n");
				StateTwo_ExchangeTable();
			break;

			case ROBOT_STATE_VOTE_ORIGIN:
				DEBUG_PRINT("goto State THREE: Vote Origin\n");
				StateThree_VoteTheOrigin();
			break;

			case ROBOT_STATE_ROTATE_COORDINATES:
				DEBUG_PRINT("goto State FOUR: Rotate Coordinates\n");
				StateFour_RotateCoordinates();
				break;

			case ROBOT_STATE_AVERAGE_VECTOR:
				DEBUG_PRINT("goto State FIVE: Average Vector\n");
				StateFive_AverageVector();
				break;

			case ROBOT_STATE_CORRECT_LOCATIONS:
				DEBUG_PRINT("goto State SIX: Correct Locations Table\n");
				StateSix_CorrectLocations();
				break;

			case ROBOT_STATE_LOCOMOTION:
				DEBUG_PRINT("goto State SEVEN: Locomotion\n");
				StateSeven_Locomotion();
				break;

			case ROBOT_STATE_UPDATE_ORIENTATION:
				DEBUG_PRINT("goto State EIGHT: Update Orientation\n");
				StateEight_UpdateOrientation();
				break;

			case ROBOT_STATE_FOLLOW_GRADIENT_MAP:
				DEBUG_PRINT("goto State NICE: Follow Gradient Map\n");
				StateNine_FollowGradientMap();
				break;

			case ROBOT_STATE_UPDATE_LOCATION:
				turnOnLED(LED_ALL);
				if (updateLocation())
				{
					broadcastLocationMessageToLocalNeighbors();
					turnOffLED(LED_ALL);
				}
				setRobotState(ROBOT_STATE_IDLE);
				break;

			case ROBOT_STATE_UPDATE_GRADIENT_GOAL:
				StateNine_FollowGradientMap_UpdateGoal();
				break;

			case ROBOT_STATE_ACTUATOR_EXECUTE:
				StateNine_FollowGradientMap_ExecuteActuator();
				break;

			case ROBOT_STATE_ROTATE_TO_ANGLE_USE_STEP:
				if(rotateToAngleUseStepController())
					setRobotState(ROBOT_STATE_IDLE);
				break;

			case ROBOT_STATE_FORWARD_IN_ROTATE_USE_STEP:
				if(forwardInRotateUseStepController())
					setRobotState(ROBOT_STATE_IDLE);
				break;

			case ROBOT_STATE_FORWARD_IN_PERIOD_USE_STEP:
				if(forwardInPeriodUseStepController())
					setRobotState(ROBOT_STATE_IDLE);
				break;

			default: // ROBOT_STATE_IDLE
				toggleLED(LED_RED);
				ROM_SysCtlDelay(12500000);
				//ROM_SysCtlDelay(ROM_SysCtlClockGet() / (3 * 1000) * 750); // ~750ms
			break;
		}
	}
}

void initSystem(void)
{
	initSysClock();

	initUartDebug();

	DEBUG_PRINTS("Current ClockSpeed: %d Hz\n", ROM_SysCtlClockGet());

	initLowPowerMode();

	initDelay();
	DEBUG_PRINT("init Delay: OK\n");

	initLeds();
	DEBUG_PRINT("init LEDs: OK\n");

//	initProximitySensor();
//	DEBUG_PRINT("init IR LED: OK\n");

	initPeripheralsForAnalogFunction();
	DEBUG_PRINT("init Peripherals for analog feature: OK\n");

	initSpeaker();
	DEBUG_PRINT("init Speaker: OK\n");

	initMotors();
	DEBUG_PRINT("init Motors: OK\n");

	initEEPROM();
	DEBUG_PRINT("init EEPROM: OK\n");

	initRfModule(true);
	DEBUG_PRINT("init RF module: OK, in rx mode.\n");

	initRobotTaskTimer();
	DEBUG_PRINT("init Robot task timer: OK\n");

	int rev = readChipRev();
	DEBUG_PRINTS("MCU rev: %d\n", rev);

#ifdef HAVE_IMU
	initI2C();
	DEBUG_PRINT("init I2C: OK\n");

	//InvMPU mpu6050;
	//if(initIMU(&mpu6050))
	if(initIMU())
		DEBUG_PRINT("init IMU: OK\n");
	else
	{
		DEBUG_PRINT("CPU trapped! Please unplug VDD signal of IMU module then replug and try again...\n");
		//TODO: software reset
		while(true);
	}
#endif
}

void MCU_RF_IRQ_handler(void)
{
	uint32_t ui32MessageSize;
	uint8_t* pui8RxBuffer = 0;

	if (MCU_RF_IsInterruptPinAsserted())
	{
		g_bIsRfFlagAsserted = true;

		MCU_RF_ClearIntFlag();

		MCU_RF_DisableInterrupt();

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

		MCU_RF_EnableInterrupt();

		turnOffLED(LED_RED);
	}

	if(getCpuMode() != CPU_MODE_RUN)
		returnToSleep();
}

void FaultISRHandler(void)
{
	toggleLED(LED_RED | LED_GREEN);
	ROM_SysCtlDelay(12000000);
}

#ifdef REGION_COMMENT
void StateSix_Locomotion()
{
	vector2_t vectZero;
	vector2_t vectOne;
	vector2_t vectDiff;

//	float fAngleOffer = 0.2094395102; // ~12 degree
	float fRotateAngle = MATH_PI_DIV_2; // ~90 degree
//	float fThetaOne;
//	float fThetaTwo;

	float alphaP;
	float alphaQ;

	uint16_t ui16RandomValue;

	g_bIsNewTDOAResults = false;

	turnOffLED(LED_ALL);

	// save my old vector for initialize position V0
	vectZero.x = g_vector.x;
	vectZero.y = g_vector.y;

	// delay random
//	generateRandomByte();
//	while (g_ui8RandomNumber == 0)
//		;
//	g_ui8RandomNumber =
//			(g_ui8RandomNumber < 100) ?
//					(g_ui8RandomNumber + 100) :
//					(g_ui8RandomNumber);

//	ui16RandomValue = (g_ui32RobotID << 10) | (g_ui8RandomNumber << 2);
//
//	ui16RandomValue = g_ui8RandomNumber * 10;

	ui16RandomValue = (g_ui32RobotID & 0xFF) * DELAY_LOCOMOTION_PERIOD;

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

//	vectDiff.x = vectOne.x - vectZero.x;
//	vectDiff.y = vectOne.y - vectZero.y;
//	fThetaOne = calculateRobotAngleWithXAxis(vectDiff);
//
//	vectDiff.x = g_vector.x - vectOne.x;
//	vectDiff.y = g_vector.y - vectOne.y;
//	fThetaTwo = calculateRobotAngleWithXAxis(vectDiff);

	vectDiff.x = g_vector.x - vectOne.x;
	vectDiff.y = g_vector.y - vectOne.y;
	g_fRobotOrientedAngle = calculateRobotAngleWithXAxis(vectDiff);

//	if (fRotateAngle < 0)
//		fRotateAngle = fThetaOne - fRotateAngle;
//	else
//		fRotateAngle += fThetaOne;
//
//	while(fRotateAngle > MATH_PI_MUL_2)
//		fRotateAngle -= MATH_PI_MUL_2;

	// one - zero = P
	vectDiff.x = vectOne.x - vectZero.x;
	vectDiff.y = vectOne.y - vectZero.y;
	alphaP = calculateRobotAngleWithXAxis(vectDiff);

	// g_vector - zero = Q
	vectDiff.x = g_vector.x - vectZero.x;
	vectDiff.y = g_vector.y - vectZero.y;
	alphaQ = calculateRobotAngleWithXAxis(vectDiff);

//	if(fRotateAngle <= (fThetaTwo + fAngleOffer) && (fRotateAngle >= (fThetaTwo - fAngleOffer)))
	if (((alphaP > alphaQ) && ((alphaP - alphaQ) < MATH_PI))
			|| ((alphaQ - alphaP) > MATH_PI))
		g_bIsCounterClockwiseOriented = false; // DIFFERENT
	else
		g_bIsCounterClockwiseOriented = true; // SAME

	turnOnLED(LED_GREEN);

	g_eProcessState = IDLE;
}

void runForwardAndCalculatteNewPosition(float distance)
{
	/* Pseudo-code:
		clear locations table
		clear neighbors table
		delete onehop neighbors table

		broadcast RF signal REQUEST TO RUN

		IF no rejection THEN
			command move 5cm
		ELSE
			return FALSE TO RUN;

		activeRobotTask TASK
		-> TASK
		--{
			successFlag = FALSE;
			DO {
	   	   	   generate random values
			   clear isFlagAssert flag
			   isFlagAssert = RfTryToCaptureRfSignal(random, handlerInDelayRandom)
				   handlerInDelayRandom()
				   -- {
							call RF handler()
							-- {
								-> call handleDistanceResultAndVectorResponse:
								-- storeDistanceAndVectorOfResponseNeighbor();
							-- }
							return true; // alway return true to reset task timer
				    -- }
			   if (isFlagAssert == true)
			   	   reset Robot timer delay();
		    } WHILE (isFlagAssert == true);

		    IF !successFlag THEN
	   	   		IF tryToRequestLocalNeighborsForDistanceMeasurement(VECTOR_AND_MEASUREMENT) success THEN
	   	   			set successFlag = TRUE
	   	   		END
	   	   	ELSE

	   	   	END
		--}

	 */

//	uint8_t length;
//	uint8_t i;
//
//	vector2_t vectEstimatePosNew;
//	vector2_t vectEstimatePosOld;
//	vector2_t vectGradienNew;
//	vector2_t vectGradienOld;
//
//	distance = (distance > 5) ? (5) : distance;
//
//	clearNeighborTable(NeighborsTable, &g_ui8NeighborsCounter);
//	clearOneHopNeighborTable(OneHopNeighborsTable);
//
//	for (i = 0; i < LOCATIONS_TABLE_LENGTH; i++)
//	{
//		locs[i].ID = 0;
//		locs[i].vector.x = 0;
//		locs[i].vector.y = 0;
//	}
//	g_ui8LocsCounter = 0;
//
//	RF24_TX_buffer[0] = ROBOT_REQUEST_TO_RUN;
//	parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
//	broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
//
//	runForwardWithDistance(distance);
//
//	SysCtlDelay(2500);
//
//	// send request measure distance command
//	RF24_TX_buffer[0] = ROBOT_REQUEST_SAMPLING_MIC;
//	parse32BitTo4Bytes(g_ui32RobotID, &RF24_TX_buffer[1]); // 1->4
//	broadcastLocalNeighbor((uint8_t*) RF24_TX_buffer, 5);
//	// WARING!!! DO NOT INSERT ANY CODE IN HERE!
//	ROM_SysCtlDelay(DELAY_START_SPEAKER);
//	// WARING!!! DO NOT INSERT ANY CODE IN HERE!
//	startSpeaker();
//
//	RF24_RX_activate();
//
//	disableRF24Interrupt();
//
//	RF24_clearIrqFlag(RF24_IRQ_MASK);
//
//	g_ui8NeighborsCounter = 0;
//
//	// start state timer wait for new neighbor response
//	delayTimerA(DELAY_NEIGHBOR_RESPONSE_PERIOD, false); // may received ROBOT_RESPONSE_TDOA_DISTANCE command at here!
//
//	while (!g_bDelayTimerAFlagAssert)
//	{
//		if (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) == 0)
//		{
//			toggleLED(LED_GREEN);
//
//			reloadDelayTimerA();
//
//			if (RF24_getIrqFlag(RF24_IRQ_RX))
//			{
//				length = RF24_RX_getPayloadWidth();
//
//				RF24_RX_getPayloadData(length, RF24_RX_buffer);
//
//				RF24_clearIrqFlag(RF24_IRQ_RX);
//
//				if (RF24_RX_buffer[0] == ROBOT_RESPONSE_TDOA_DISTANCE
//							&& length == 15)
//				{
//					storeNeighorVectorAndDistanceToTables(RF24_RX_buffer);
//				}
//			}
//		}
//	}
//
//	turnOffLED(LED_GREEN);
//
//	enableRF24Interrupt();
//
//	if (g_ui8NeighborsCounter < 3)
//		return;
//
//	// active gradient descent to calculate new position
//
//	vectEstimatePosNew.x = g_vector.x;
//	vectEstimatePosNew.y = g_vector.y;
//
//	vectGradienNew.x = 0;
//	vectGradienNew.y = 0;
//
//	g_bIsGradientSearchStop = false;
//
//	while(!g_bIsGradientSearchStop)
//	{
//		toggleLED(LED_BLUE);
//
//		updateGradient(&vectGradienNew, false);
//
//		updatePosition(&g_vector, &vectEstimatePosNew, &vectEstimatePosOld, &vectGradienNew, &vectGradienOld, g_fStepSize);
//		synchronousLocsTableAndMyVector();
//
//		g_bIsGradientSearchStop = checkVarianceCondition(vectEstimatePosNew, vectEstimatePosOld, g_fStopCondition);
//	}
//
//	Tri_addLocation(g_ui32RobotID, g_vector.x, g_vector.y);
//
//	turnOffLED(LED_BLUE);
}

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

//	float a =10.0f;
//	float b = 3.49f;
//
//	float c_soft;
//	float c_hard;
//	uint32_t t0, t1, t2;
//
//	// Soft 21.25us :: Hard 12.5us
//	uint32_t ClockSpeed = ROM_SysCtlClockGet();
//	ROM_TimerLoadSet(TASK_TIMER_BASE, TIMER_A, ClockSpeed);
//	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
//	ROM_TimerEnable(TASK_TIMER_BASE, TIMER_A);
//
//	t0 = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//	c_hard = sqrtf(b);
//	t1 = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//	c_soft = sqrt_fast(b);
//	t2 = ROM_TimerValueGet(TASK_TIMER_BASE, TIMER_A);
//
//	ROM_TimerDisable(TASK_TIMER_BASE, TIMER_A);
//	ROM_TimerIntClear(TASK_TIMER_BASE, TIMER_TIMA_TIMEOUT);
#endif
