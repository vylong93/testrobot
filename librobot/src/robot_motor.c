/*
 * robot_motor.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot/inc/robot_motor.h"
#include "pwm_definition.h"

#include "libmath/inc/custom_math.h"
#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libcustom/inc/custom_delay.h"

// Basic Motor interface ======================
static bool bIsMotorDriverEnable = false;
static bool bIsMotorLeftEnable = false;
static bool bIsMotorRightEnable = false;

static uint32_t ui32PWMPeriod;

void initMotors(void)
{
	// Initilize PWM generator module 0
	ROM_SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	ROM_SysCtlDelay(2);
	ROM_SysCtlPeripheralEnable(MOTOR_PWM_CLOCK);

	uint32_t pwmClock = ROM_SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	ui32PWMPeriod = (pwmClock / MOTOR_PWM_FREQUENCY);

	// Configure GPIO pin to control Motor driver IC
	ROM_SysCtlPeripheralEnable(MOTOR_SLEEP_PIN_CLOCK);
	ROM_SysCtlDelay(2);
	ROM_GPIOPinTypeGPIOOutput(MOTOR_SLEEP_PIN_BASE, MOTOR_SLEEP_PIN);

	// Left motor (1) pin configure
	ROM_SysCtlPeripheralEnable(LEFT_MOTOR_PORT_CLOCK);
	ROM_SysCtlDelay(2);

	ROM_GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	ROM_GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);
	ROM_PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// left - M0PWM4
	ROM_PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2, 0);
	ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	// left - M0PWM5
	ROM_PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, 0);
	ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);

	// Right motor (2) pin configure
	ROM_SysCtlPeripheralEnable(RIGHT_MOTOR_PORT_CLOCK);
	ROM_SysCtlDelay(2);

	ROM_GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	ROM_GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);
	ROM_PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	// right - M0PWM2
	ROM_PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
	ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2, 0);
	ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	// right - M0PWM3
	ROM_PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
	ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, 0);
	ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);

	Motors_stop();

	initRobotMotorPairTimer();
}

void Motors_stop(void)
{
	if(bIsMotorDriverEnable)
	{
		MotorDriver_disable();
		MotorLeft_stop();
		MotorRight_stop();
	}
}

void MotorLeft_stop(void)
{
	if(bIsMotorLeftEnable)
	{
		ROM_PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
		ROM_GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
		ROM_GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);
		bIsMotorLeftEnable = false;
	}
}

void MotorRight_stop(void)
{
	if(bIsMotorRightEnable)
	{
		ROM_PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
		ROM_GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
		ROM_GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);
		bIsMotorRightEnable = false;
	}
}

void MotorDriver_enable(void)
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x01;

	bIsMotorDriverEnable = true;
}

void MotorDriver_disable(void)
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x00;

	bIsMotorDriverEnable = false;
}

void Motors_configure(Motor_t mLeftMotor, Motor_t mRightMotor)
{
	MotorDriver_disable();

	ROM_PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	ROM_PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	MotorLeft_assignActiveParameter(mLeftMotor);
	MotorRight_assignActiveParameter(mRightMotor);

	ROM_PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	ROM_PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	MotorDriver_enable();
}

void MotorLeft_assignActiveParameter(Motor_t mMotor)
{
	uint32_t ui32DutyCycle;

	if(mMotor.ui8Speed < MOTOR_SPEED_MINIMUM)
	{
		MotorLeft_stop();
		return;
	}

	if(mMotor.ui8Speed > MOTOR_SPEED_MAXIMUM)
		mMotor.ui8Speed = MOTOR_SPEED_MAXIMUM;

	ui32DutyCycle = ((256 - mMotor.ui8Speed) * ui32PWMPeriod) / 256;

	if (mMotor.eDirection == FORWARD)
	{
		// left - E5
		ROM_GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		ROM_GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN1);

		// left - M0PWM4
		ROM_GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		ROM_GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG2);
		ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2, ui32DutyCycle);

		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);
		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (mMotor.eDirection == REVERSE)
	{
		// left - M0PWM5
		ROM_GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		ROM_GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG1);
		ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, ui32DutyCycle);

		// left - E4
		ROM_GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		ROM_GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2, LEFT_MOTOR_IN2);

		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
		ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	}

	bIsMotorLeftEnable = true;
}

void MotorRight_assignActiveParameter(Motor_t mMotor)
{
	uint32_t ui32DutyCycle;

	if(mMotor.ui8Speed < MOTOR_SPEED_MINIMUM)
	{
		MotorRight_stop();
		return;
	}

	if(mMotor.ui8Speed > MOTOR_SPEED_MAXIMUM)
		mMotor.ui8Speed = MOTOR_SPEED_MAXIMUM;

	ui32DutyCycle = ((256 - mMotor.ui8Speed) * ui32PWMPeriod) / 256;

	if (mMotor.eDirection == FORWARD)
	{
		// right - B5
		ROM_GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		ROM_GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN1);

		// right - M0PWM2
		ROM_GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		ROM_GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG2);
		ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2, ui32DutyCycle);

		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (mMotor.eDirection == REVERSE)
	{
		// right - M0PWM3
		ROM_GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		ROM_GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG1);
		ROM_PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1, ui32DutyCycle);

		// right - B4
		ROM_GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		ROM_GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2, RIGHT_MOTOR_IN2);

		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
		ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	}

	bIsMotorRightEnable = true;
}


// Timer monitor interface ====================

static char g_flagMotorSelect = 1;

void initRobotMotorPairTimer(void)
{
	ROM_SysCtlPeripheralEnable(MOTOR_TIMER_CLOCK);
	TimerClockSourceSet(MOTOR_TIMER_BASE, TIMER_CLOCK_SYSTEM);
	ROM_TimerConfigure(MOTOR_TIMER_BASE, TIMER_CFG_ONE_SHOT);

	ROM_TimerConfigure(MOTOR_TIMER_BASE,
	TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);

	ROM_TimerIntEnable(MOTOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntEnable(MOTOR_TIMER_BASE, TIMER_TIMB_TIMEOUT);

	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMB_TIMEOUT);
}

void Motor_delay_timerA_ms(uint32_t ui32PeriodInMs)
{
	Motor_delay_timerA_us(ui32PeriodInMs * 1000);
}

void Motor_delay_timerA_us(uint32_t ui32PeriodInUs)
{
	if (ui32PeriodInUs == 0)
		return;

	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / 1000000) * ui32PeriodInUs;

	//
	// Reset timer counter
	//
	ROM_TimerLoadSet(MOTOR_TIMER_BASE, TIMER_A, delayPeriod);

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Start timer
	//
	ROM_TimerEnable(MOTOR_TIMER_BASE, TIMER_A);

	while(1)
	{
		//
		// Get delay status
		//
		ui32Status = ROM_TimerIntStatus(MOTOR_TIMER_BASE, false);

		//
		// Check for delay timeout
		//
		if (ui32Status & TIMER_TIMA_TIMEOUT)
			break;
	}

	//
	// Clear timer interrupt flag
	//
	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMA_TIMEOUT);
}

void Robot_activeMotorsTask(uint32_t ui32PeriodInMs, e_RobotMovement eRobotMovement, bool (*pfnTask)(e_RobotMovement eMovement))
{
	if (ui32PeriodInMs == 0)
		return;

	uint32_t ui32Status;
	uint32_t delayPeriod = (ROM_SysCtlClockGet() / 1000) * ui32PeriodInMs;

	ROM_TimerLoadSet(MOTOR_TIMER_BASE, TIMER_B, delayPeriod);

	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMB_TIMEOUT);

	ROM_TimerEnable(MOTOR_TIMER_BASE, TIMER_B);

	while(1)
	{
		ui32Status = ROM_TimerIntStatus(MOTOR_TIMER_BASE, false);

		if (ui32Status & TIMER_TIMB_TIMEOUT)
			break;

	    if((*pfnTask)(eRobotMovement))
	    	break;
	}

	ROM_TimerIntClear(MOTOR_TIMER_BASE, TIMER_TIMB_TIMEOUT);
}

void Robot_stepRotate_tunning(e_RobotRotateDirection eRotateDirection, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod)
{
	g_flagMotorSelect ^= 1;

	e_MotorDirection leftD, rightD;
	if (eRotateDirection == ROBOT_ROTATE_CLOCKWISE)
	{
		leftD = REVERSE;
		rightD = FORWARD;
	}
	else if (eRotateDirection == ROBOT_ROTATE_COUNTERCLOSEWISE)
	{
		leftD = FORWARD;
		rightD = REVERSE;
	}
	else
	{
		return;
	}

	if(g_flagMotorSelect)
		MotorLeft_commandStep(leftD, ui32ActivePeriod, ui32PausePeriod);
	else
		MotorRight_commandStep(rightD, ui32ActivePeriod, ui32PausePeriod);
}

void MotorLeft_commandStep(e_MotorDirection directionLeftMotor, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod)
{
	MotorRight_stop();

	Motor_t mLeftMotor;
	mLeftMotor.eDirection = directionLeftMotor;
	mLeftMotor.ui8Speed = STEP_MAX_SPEED;
	MotorLeft_assignActiveParameter(mLeftMotor);

	MotorDriver_enable();

//	Motor_t mLeft, mRight;
//	mLeft.eDirection = directionLeftMotor;
//	mRight.eDirection = FORWARD; // No effect
//
//	mLeft.ui8Speed = STEP_MAX_SPEED;
//	mRight.ui8Speed = 0;
//
//	Motors_configure(mLeft, mRight);

	Motor_delay_timerA_ms(ui32ActivePeriod);

	Motors_stop();
	Motor_delay_timerA_ms(ui32PausePeriod);
}

void MotorRight_commandStep(e_MotorDirection directionRightMotor, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod)
{
	MotorLeft_stop();

	Motor_t mRightMotor;
	mRightMotor.eDirection = directionRightMotor;
	mRightMotor.ui8Speed = STEP_MAX_SPEED;
	MotorRight_assignActiveParameter(mRightMotor);

	MotorDriver_enable();

//	Motor_t mLeft, mRight;
//	mLeft.eDirection = FORWARD; // No effect
//	mRight.eDirection = directionRightMotor;
//
//	mLeft.ui8Speed = 0;
//	mRight.ui8Speed = STEP_MAX_SPEED;
//
//	Motors_configure(mLeft, mRight);

	Motor_delay_timerA_ms(ui32ActivePeriod);

	Motors_stop();
	Motor_delay_timerA_ms(ui32PausePeriod);
}

//==================================================================
void Robot_stepMovementWithPeriod(uint16_t ui16PeriodMs, e_RobotMovement eRobotMovement)
{
	Robot_activeMotorsTask(ui16PeriodMs, eRobotMovement, Robot_stepMovementTask);
}

bool Robot_stepMovementTask(e_RobotMovement eRobotRotateMovement)
{
	e_MotorDirection leftD, rightD;

	switch(eRobotRotateMovement)
	{
		case ROBOT_MOVEMENT_CLOCKWISE:
			leftD = REVERSE;
			rightD = FORWARD;
			break;

		case ROBOT_MOVEMENT_COUNTERCLOSEWISE:
			leftD = FORWARD;
			rightD = REVERSE;
			break;

		case ROBOT_MOVEMENT_REVERSE:
			leftD = REVERSE;
			rightD = REVERSE;
			break;

		case ROBOT_MOVEMENT_FORWARD:
			leftD = FORWARD;
			rightD = FORWARD;
			break;

		default:
			return true;
	}

	g_flagMotorSelect ^= 1;
	if(g_flagMotorSelect)
		MotorLeft_commandStep(leftD, 60, 0);
	else
		MotorRight_commandStep(rightD, 50, 0);

	return false;
}

// Test only ==================================
uint8_t g_ui8LeftMotorOffset = 100;
uint8_t g_ui8RightMotorOffset = 100;
float g_fPeriodMotorOffset = 1000.0f;

void setLeftMotorOffset(uint8_t ui8Parameter)
{
	g_ui8LeftMotorOffset = ui8Parameter;
}

void setRightMotorOffset(uint8_t ui8Parameter)
{
	g_ui8RightMotorOffset = ui8Parameter;
}

void setPeriodMotorOffset(uint16_t ui16Parameter)
{
	g_fPeriodMotorOffset = ui16Parameter * 1.0f;
}

void Robot_move(e_RobotMoveDirection eMoveDirection)
{
	Motor_t mLeft, mRight;

	mLeft.ui8Speed = g_ui8LeftMotorOffset;
	mRight.ui8Speed = g_ui8RightMotorOffset;

	if (eMoveDirection == ROBOT_MOVE_MAINTAIN)
		return;

	if (eMoveDirection == ROBOT_MOVE_FORWARD)
	{
		mLeft.eDirection = FORWARD;
		mRight.eDirection = FORWARD;
	}
	else if (eMoveDirection == ROBOT_MOVE_REVERSE)
	{
		mLeft.eDirection = REVERSE;
		mRight.eDirection = REVERSE;
	}

	Motors_configure(mLeft, mRight);
}

void Robot_rotate(e_RobotRotateDirection eRotateDirection)
{
	Motor_t mLeft, mRight;

	mLeft.ui8Speed = (uint8_t)(g_ui8LeftMotorOffset);
	mRight.ui8Speed = (uint8_t)(g_ui8RightMotorOffset);

	if(eRotateDirection == ROBOT_ROTATE_MAINTAIN)
		return;

	if (eRotateDirection == ROBOT_ROTATE_CLOCKWISE)
	{
		mLeft.eDirection = REVERSE;
		mRight.eDirection = FORWARD;
	}
	else if (eRotateDirection == ROBOT_ROTATE_COUNTERCLOSEWISE)
	{
		mLeft.eDirection = FORWARD;
		mRight.eDirection = REVERSE;
	}

	Motors_configure(mLeft, mRight);
}

void Robot_rotate_tuning(e_RobotRotateDirection eRotateDirection, uint8_t ui8LeftM, uint8_t ui8RightM)
{
	static char flag = 1;
	flag ^= 1;

	Motor_t mLeft, mRight;

	if(flag)
	{
		mLeft.ui8Speed = 0;
		mRight.ui8Speed = ui8RightM;
	}
	else
	{
		mLeft.ui8Speed = ui8LeftM;
		mRight.ui8Speed = 0;
	}

	if(eRotateDirection == ROBOT_ROTATE_MAINTAIN)
		return;

	if (eRotateDirection == ROBOT_ROTATE_CLOCKWISE)
	{
		mLeft.eDirection = REVERSE;
		mRight.eDirection = FORWARD;
	}
	else if (eRotateDirection == ROBOT_ROTATE_COUNTERCLOSEWISE)
	{
		mLeft.eDirection = FORWARD;
		mRight.eDirection = REVERSE;
	}

	Motors_configure(mLeft, mRight);
}

void applyWheelSpeedsToRotate(float vel_l, float vel_r, uint8_t *pui8LeftSpeed, uint8_t *pui8RightSpeed)
{
// Attemp 0:
	*pui8LeftSpeed = g_ui8LeftMotorOffset;
	*pui8RightSpeed = g_ui8RightMotorOffset;
	
// Attemp 1:
//	int32_t diff = g_ui8LeftMotorOffset - g_ui8RightMotorOffset;
//	diff = abs(diff) / 2;
//
//	if(g_ui8LeftMotorOffset > g_ui8RightMotorOffset)
//	{
//		*pui8LeftSpeed = (uint8_t)((int32_t)(fabsf(vel_l) + diff + 0.5f));
//
//		if (vel_r > diff)
//			*pui8RightSpeed = (uint8_t)((int32_t)(fabsf(vel_r) - diff + 0.5f));
//		else
//			*pui8RightSpeed = (uint8_t)((int32_t)(fabsf(vel_r) + 0.5f));
//	}
//	else
//	{
//		if (vel_l > diff)
//			*pui8LeftSpeed = (uint8_t)((int32_t)(fabsf(vel_l) - diff + 0.5f));
//		else
//			*pui8LeftSpeed = (uint8_t)((int32_t)(fabsf(vel_l) + 0.5f));
//
//		*pui8RightSpeed = (uint8_t)((int32_t)(fabsf(vel_r) + diff + 0.5f));
//	}
}

void rotateClockwiseWithAngle(float fAngleInRadian)
{
	bool bCurrentInterruptStage;
	MCU_RF_PauseInterruptState(&bCurrentInterruptStage);

	fAngleInRadian = atan2f(sinf(fAngleInRadian), cosf(fAngleInRadian));

	if (fAngleInRadian > 0)
	{
		Robot_rotate(ROBOT_ROTATE_CLOCKWISE);
	}
	else
	{
		Robot_rotate(ROBOT_ROTATE_COUNTERCLOSEWISE);
		fAngleInRadian = 0 - fAngleInRadian;
	}

	delay_ms((uint32_t)((g_fPeriodMotorOffset * fAngleInRadian / MATH_PI_DIV_2) + 0.5));

	Motors_stop();

	MCU_RF_ContinueInterruptStateBeforePause(bCurrentInterruptStage);
}

void runForwardWithDistance(float fDistanceInCm)
{
	bool bCurrentInterruptStage;
	MCU_RF_PauseInterruptState(&bCurrentInterruptStage);

	if (fDistanceInCm > 0)
	{
		Robot_move(ROBOT_MOVE_FORWARD);
	}
	else
	{
		Robot_move(ROBOT_MOVE_REVERSE);

		fDistanceInCm = 0 - fDistanceInCm;
	}

	delay_ms((uint32_t)((g_fPeriodMotorOffset * fDistanceInCm / 8.0) + 0.5));

	Motors_stop();

	MCU_RF_ContinueInterruptStateBeforePause(bCurrentInterruptStage);
}

