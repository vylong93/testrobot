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
#include "driverlib\timer.h"
#include "timers_definition.h"

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

typedef enum tag_RobotMovement
{
	ROBOT_MOVEMENT_COUNTERCLOSEWISE = 0,
	ROBOT_MOVEMENT_CLOCKWISE = 1,
	ROBOT_MOVEMENT_REVERSE = 2,
	ROBOT_MOVEMENT_FORWARD = 3,
} e_RobotMovement;

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

// Basic Motor interface ======================
void initMotors(void);
void Motors_stop(void);
void MotorLeft_stop(void);
void MotorRight_stop(void);
void MotorDriver_enable(void);
void MotorDriver_disable(void);

void Motors_configure(Motor_t mLeftMotor, Motor_t mRightMotor);
void MotorLeft_assignActiveParameter(Motor_t motor);
void MotorRight_assignActiveParameter(Motor_t motor);

// Timer monitor interface ====================
#define STEP_ACTIVE_MOTORS_MS 	50
#define STEP_PAUSE_MOTORS_MS 	50

#define STEP_MAX_SPEED 			254
#define STEP_MIN_SPEED 			10

void initRobotMotorPairTimer(void);
void Motor_delay_timer_ms(uint32_t ui32PeriodInMs);
void Motor_delay_timer_us(uint32_t ui32PeriodInUs);
void Motor_task_timer_start(uint32_t ui32PeriodInMs);
bool Motor_task_timer_isExpired(void);
void Robot_activeMotorsTask(uint32_t ui32PeriodInMs, e_RobotMovement eRobotMovement, bool (*pfnTask)(e_RobotMovement eMovement));

void Robot_stepRotate_tunning(e_RobotRotateDirection eRotateDirection, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod);
void MotorLeft_commandStep(e_MotorDirection directionLeftMotor, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod);
void MotorRight_commandStep(e_MotorDirection directionRightMotor, uint32_t ui32ActivePeriod, uint32_t ui32PausePeriod);

void Robot_stepMovementWithPeriod(uint32_t ui32PeriodMs, e_RobotMovement eRobotMovement);
bool Robot_stepMovementTask(e_RobotMovement eRobotMovement);

// Test only ==================================
void setLeftMotorOffset(uint8_t ui8Parameter);
void setRightMotorOffset(uint8_t ui8Parameter);
void setPeriodMotorOffset(uint16_t ui16Parameter);

void Robot_move(e_RobotMoveDirection eMoveDirection);
void Robot_rotate(e_RobotRotateDirection eRotateDirection);
void Robot_rotate_tuning(e_RobotRotateDirection eRotateDirection, uint8_t ui8LeftM, uint8_t ui8RightM);

void applyWheelSpeedsToRotate(float vel_l, float vel_r, uint8_t *pui8LeftSpeed, uint8_t *pui8RightSpeed);

void rotateClockwiseWithAngle(float fAngleInRadian);
void runForwardWithDistance(float fDistanceInCm);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_MOTOR_H_ */
