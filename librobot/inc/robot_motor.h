/*
 * robot_motor.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_MOTOR_H_
#define ROBOT_MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc\hw_memmap.h"
#include "inc\hw_types.h"
#include "inc\hw_gpio.h"
#include "driverlib\debug.h"
#include "driverlib\pin_map.h"
#include "driverlib\sysctl.h"
#include "driverlib\rom.h"
#include "driverlib\pwm.h"
#include "driverlib\gpio.h"

#define MOTOR_SPEED_MINIMUM		1
#define MOTOR_SPEED_MAXIMUM		250

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

typedef enum tag_RobotMoveDirection
{
	ROBOT_MOVE_REVERSE = 0,
	ROBOT_MOVE_FORWARD = 1,
	ROBOT_MOVE_MAINTAIN = 2
} e_RobotMoveDirection;

typedef enum tag_RobotRotateDirection
{
	ROBOT_ROTATE_COUNTERCLOSEWISE = 0,
	ROBOT_ROTATE_CLOCKWISE = 1,
	ROBOT_ROTATE_MAINTAIN = 2
} e_RobotRotateDirection;

typedef enum tag_MotorDirection
{
  FORWARD = 0,
  REVERSE = 1
} e_MotorDirection;

typedef struct tag_Motor
{
	e_MotorDirection eDirection;
	uint8_t	ui8Speed;
} Motor_t;

void setLeftMotorOffset(uint8_t ui8Parameter);
void setRightMotorOffset(uint8_t ui8Parameter);
void setPeriodMotorOffset(uint16_t ui16Parameter);

void Robot_move(e_RobotMoveDirection eMoveDirection);
void Robot_rotate(e_RobotRotateDirection eRotateDirection);
void Robot_rotate_tuning(e_RobotRotateDirection eRotateDirection, uint8_t ui8LeftM, uint8_t ui8RightM);

void applyWheelSpeedsToRotate(float vel_l, float vel_r, uint8_t *pui8LeftSpeed, uint8_t *pui8RightSpeed);

void rotateClockwiseWithAngle(float fAngleInRadian);
void runForwardWithDistance(float fDistanceInCm);

void initMotors(void);
void stopMotors(void);
void enableMOTOR(void);
void disableMOTOR(void);

void configureMotors(Motor_t mLeftMotor, Motor_t mRightMotor);
void MotorLeft_assignActiveParameter(Motor_t motor);
void MotorRight_assignActiveParameter(Motor_t motor);

void MotorLeft_stop(void);
void MotorRight_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_MOTOR_H_ */
