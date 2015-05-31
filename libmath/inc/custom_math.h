/*
 * custom_math.h
 *
 *  Created on: Mar 14, 2015
 *      Author: VyLong
 */

#ifndef LIBMATH_INC_CUSTOM_MATH_H_
#define LIBMATH_INC_CUSTOM_MATH_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"

typedef struct tagLineEquation {
	float a;
	float b;
	float c;
} LineEquation_t;

typedef struct tagCircleEquation {
	float x0;
	float y0;
	float R;
} CircleEquation_t;

#define MATH_PI_MUL_2			6.283185307
#define MATH_PI_MUL_3_DIV_2	    4.71238898
#define MATH_PI 				3.141592654
#define MATH_PI_DIV_2			1.570796327
#define MINUS_MATH_PI_DIV_2		-1.570796327
#define MATH_PI_DIV_2_MUL_32768	51471.85404
#define _180_DIV_PI				57.29577951

#define MATH_RAD2DEG			57.29577951
#define MATH_DEG2RAD			0.017453293

//#define ANGLE_MIN_IN_RAD	0.15	// ~ 8.594366927 degree
//#define COSINE_ANGLE_MIN	0.9887710779 // cos(ANGLE_MIN_IN_RAD)

#define ANGLE_MIN_IN_RAD	0.1047197551 // ~ 6 degree
#define COSINE_ANGLE_MIN	0.9945218954 // cos(ANGLE_MIN_IN_RAD)

//#define ANGLE_MIN_IN_RAD	0.1745329252 // ~ 10 degree
//#define COSINE_ANGLE_MIN	0.9999953604 // cos(ANGLE_MIN_IN_RAD)

float signFloatNumber(float a);

bool isTwoAngleOverlay(float a, float b, float errorInDeg);

bool isValidTriangle(uint16_t a, uint16_t b, uint16_t c);
bool isTriangle(uint32_t a, uint32_t b, uint32_t c);
float findCosAngleUseCosineRuleForTriangle(float fSide1, float fSide2, float fOppositeSide);

float findAltitudeOfTriangle(float a, float b, float c);
bool solveQuadraticEquation(float A, float _B, float C, float* pX1, float *pX2);

#ifdef __cplusplus
}
#endif

#endif /* LIBMATH_INC_CUSTOM_MATH_H_ */
