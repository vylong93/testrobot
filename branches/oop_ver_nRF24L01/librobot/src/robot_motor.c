/*
 * robot_motor.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_motor.h"
#include "pwm_definition.h"

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
}

void stopMotors(void)
{
	disableMOTOR();
	MotorLeft_stop();
	MotorRight_stop();
}

void enableMOTOR(void)
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x01;
}

void disableMOTOR(void)
{
	ASSERT(_GPIOBaseValid(MOTOR_SLEEP_PIN_BASE));
	HWREG(MOTOR_SLEEP_PIN_BASE + (GPIO_O_DATA + (MOTOR_SLEEP_PIN << 2))) =
			0x00;
}

void configureMotors(Motor_t mLeftMotor, Motor_t mRightMotor)
{
	disableMOTOR();

	ROM_PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	ROM_PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	MotorLeft_assignActiveParameter(mLeftMotor);
	MotorRight_assignActiveParameter(mRightMotor);

	ROM_PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	ROM_PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	enableMOTOR();
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
}

void MotorLeft_stop()
{
	ROM_PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
	ROM_PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	ROM_GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	ROM_GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);
}

void MotorRight_stop()
{
	ROM_PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
	ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
	ROM_PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	ROM_GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	ROM_GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);
}


