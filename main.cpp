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

//	setRobotIdentity();

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

			case ROBOT_STATE_CHECK_LOCATION:
				checkLocation();
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
	turnOnLED(LED_RED | LED_GREEN | LED_BLUE);
	ROM_SysCtlDelay(12500000);
}
