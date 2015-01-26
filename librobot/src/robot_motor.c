/*
 * robot_motor.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */


#include "librobot\inc\robot_motor.h"

static uint32_t ui32PWMPeriod;

inline void initMotor()
{
	// Initilize PWM generator module 0
	SysCtlPWMClockSet(PWM_CLOCK_SELECT);
	SysCtlDelay(2);
	SysCtlPeripheralEnable(MOTOR_PWM_CLOCK);

	uint32_t pwmClock = SysCtlClockGet() / PWM_CLOCK_PRESCALE;
	ui32PWMPeriod = (pwmClock / MOTOR_PWM_FREQUENCY);

	// Configure GPIO pin to control Motor driver IC
	SysCtlPeripheralEnable(MOTOR_SLEEP_PIN_CLOCK);
	SysCtlDelay(2);
	GPIOPinTypeGPIOOutput(MOTOR_SLEEP_PIN_BASE, MOTOR_SLEEP_PIN);

	// Left motor (1) pin configure
	SysCtlPeripheralEnable(LEFT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);

	GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);

	// left - M0PWM4
	PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2, 0);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	// left - M0PWM5
	PWMGenConfigure(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1, 0);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);

	// Right motor (2) pin configure
	SysCtlPeripheralEnable(RIGHT_MOTOR_PORT_CLOCK);
	SysCtlDelay(2);

	GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	// right - M0PWM2
	PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, ui32PWMPeriod);
	PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2, 0);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	// right - M0PWM3
	PWMGenConfigure(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN, PWM_GEN_MODE_DOWN);
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

void setMotorLeftDirectionAndSpeed(MotorDirection_t direction, uint8_t speed)
{
	if (direction == FORWARD)
	{
		// left - E5
		GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1, LEFT_MOTOR_IN1);

		// left - M0PWM4
		GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG2);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2,
				(speed * ui32PWMPeriod) / 100);

		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, false);
		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (direction == REVERSE)
	{
		// left - M0PWM5
		GPIOPinTypePWM(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN1);
		GPIOPinConfigure(LEFT_MOTOR_PWM_CONFIG1);
		PWMPulseWidthSet(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1,
				(speed * ui32PWMPeriod) / 100);

		// left - E4
		GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2);
		GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2, LEFT_MOTOR_IN2);

		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
		PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, false);
	}
}

void setMotorRightDirectionAndSpeed(MotorDirection_t direction, uint8_t speed)
{
	if (direction == FORWARD)
	{
		// right - B5
		GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN1);

		// right - M0PWM2
		GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG2);
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2,
				(speed * ui32PWMPeriod) / 100);

		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, true);
	}
	else if (direction == REVERSE)
	{
		// right - M0PWM3
		GPIOPinTypePWM(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN1);
		GPIOPinConfigure(RIGHT_MOTOR_PWM_CONFIG1);
		PWMPulseWidthSet(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1,
				(speed * ui32PWMPeriod) / 100);

		// right - B4
		GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2);
		GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2, RIGHT_MOTOR_IN2);

		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, true);
		PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	}
}

void configureMotors(MotorDirection_t leftDir, uint8_t leftDutyCycle, MotorDirection_t rightDir, uint8_t rightDutyCycle)
{
	if (leftDutyCycle > 99)
		leftDutyCycle = 99;
	if (rightDutyCycle > 99)
		rightDutyCycle = 99;

	leftDutyCycle = 100 - leftDutyCycle;
	rightDutyCycle = 100 - rightDutyCycle;

	disableMOTOR();

	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	setMotorLeftDirectionAndSpeed(leftDir, leftDutyCycle);
	setMotorRightDirectionAndSpeed(rightDir, rightDutyCycle);

	PWMGenEnable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMGenEnable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);

	enableMOTOR();
}

void stopMotorLeft()
{
	PWMGenDisable(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_GEN);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT1_BIT, true);
	PWMOutputState(MOTOR_PWM_BASE, LEFT_MOTOR_PWM_OUT2_BIT, true);
	GPIOPinTypeGPIOOutput(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1);
	GPIOPinWrite(LEFT_MOTOR_PORT_BASE, LEFT_MOTOR_IN2 | LEFT_MOTOR_IN1, 0);

}

void stopMotorRight()
{
	PWMGenDisable(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_GEN);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT1_BIT, false);
	PWMOutputState(MOTOR_PWM_BASE, RIGHT_MOTOR_PWM_OUT2_BIT, false);
	GPIOPinTypeGPIOOutput(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1);
	GPIOPinWrite(RIGHT_MOTOR_PORT_BASE, RIGHT_MOTOR_IN2 | RIGHT_MOTOR_IN1, 0);

}

void stopMotors()
{
	disableMOTOR();
	stopMotorLeft();
	stopMotorRight();
}

