/*
 * robot_smartphone.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_SMARTPHONE_H_
#define ROBOT_SMARTPHONE_H_


bool tryToGetMotorsParameterInEEPROM(int8_t *pi8Motor1Speed, int8_t *pi8Motor2Speed);
void goStraight();
void spinClockwise();
void spinCounterclockwise();
void goBackward();

#endif /* ROBOT_SMARTPHONE_H_ */
