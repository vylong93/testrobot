/*
 * robot_motor.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_MOTOR_H_
#define ROBOT_MOTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"

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



#endif /* ROBOT_MOTOR_H_ */
