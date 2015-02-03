/*
 * robot_math.h
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_MATH_H_
#define ROBOT_MATH_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#define MATH_PI_MUL_2			6.283185307
#define MATH_PI_MUL_3_DIV_2	    4.71238898
#define MATH_PI 				3.141592654
#define MATH_PI_DIV_2			1.570796327
#define MINUS_MATH_PI_DIV_2		-1.570796327
#define MATH_PI_DIV_2_MUL_32768	51471.85404
#define _180_DIV_PI				57.29577951
#define EPPROM_SINE_TABLE_ADDRESS       0x0080  // Block 2
#define EPPROM_ARC_SINE_TABLE_ADDRESS   0x0200  // Block 5

//#define ANGLE_MIN_IN_RAD	0.15	// ~ 8.594366927 degree
//#define COSINE_ANGLE_MIN	0.9887710779 // cos(ANGLE_MIN_IN_RAD)

#define ANGLE_MIN_IN_RAD	0.1047197551 // ~ 6 degree
#define COSINE_ANGLE_MIN	0.9945218954 // cos(ANGLE_MIN_IN_RAD)

//#define ANGLE_MIN_IN_RAD	0.1745329252 // ~ 10 degree
//#define COSINE_ANGLE_MIN	0.9999953604 // cos(ANGLE_MIN_IN_RAD)

float vsqrtf(float op1);
float calSin(float x);
float calCos(float x);
float calASin(float x);
float calACos(float x);
float cosinesRuleForTriangles(float a, float b, float c);
bool isTriangle(float a, float b, float c);
bool isValidTriangle(uint32_t a, uint32_t b, uint32_t c);


#endif /* ROBOT_MATH_H_ */
