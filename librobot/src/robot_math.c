/*
 * robot_math.c
 *
 *  Created on: Jan 26, 2015
 *      Author: VyLong
 */

#include "librobot\inc\robot_math.h"

float vsqrtf(float op1)
{
	if (op1 <= 0.f)
		return 0.f;

	return sqrtf(op1);

//	float result;
//	__ASM
//	volatile ("vsqrt.f32 %0, %1" : "=w" (result) : "w" (op1) );
//	return (result);
}

float calSin(float x)
{

	float tempX;
	float angleX;
	uint32_t angleIndex;
	int8_t resultSigned;
	uint8_t selectResult;
	uint32_t resultIndex;
	uint16_t pui16ReadBuffer[2] =
	{ 0, 0 };

	x *= _180_DIV_PI;

	tempX = (x > 0) ? (x) : (360 + x);

	// angleX = (tempX > 360) ? (tempX - 360.0) : (tempX);
	angleX = tempX;
	while(angleX > 360)
		angleX -= 360.0;

	angleIndex = (int) (angleX * 2 + 0.5);

	if (angleIndex < 180)
	{
		resultSigned = 1;
		resultIndex = angleIndex;
	}
	else if (angleIndex < 360)
	{
		resultSigned = 1;
		resultIndex = 360 - angleIndex;
	}
	else if (angleIndex < 540)
	{
		resultSigned = -1;
		resultIndex = angleIndex - 360;
	}
	else
	{
		resultSigned = -1;
		resultIndex = 720 - angleIndex;
	}

	selectResult = resultIndex & 0x01;

	resultIndex &= 0xFFFFFFFE;
	resultIndex <<= 1;
	resultIndex += EPPROM_SINE_TABLE_ADDRESS;

	EEPROMRead((uint32_t*) pui16ReadBuffer, resultIndex,
			sizeof(pui16ReadBuffer));

	return (((resultSigned * pui16ReadBuffer[selectResult])) / 32768.0);
}

float calCos(float x)
{
	return calSin(x + MATH_PI_DIV_2);
}

float calASin(float x)
{
	int8_t resultSigned;
	uint8_t selectResult;
	uint32_t resultIndex;
	uint16_t pui16ReadBuffer[2] =
	{ 0, 0 };

	if (x > 0)
	{
		resultSigned = 1;
		resultIndex = (int) ((x * 180) + 0.25);
	}
	else
	{
		resultSigned = -1;
		resultIndex = (int) (((-x) * 180) + 0.25);
	}

	selectResult = resultIndex & 0x01;

	resultIndex &= 0xFFFFFFFE;
	resultIndex <<= 1;
	resultIndex += EPPROM_ARC_SINE_TABLE_ADDRESS;

	EEPROMRead((uint32_t*) pui16ReadBuffer, resultIndex,
			sizeof(pui16ReadBuffer));

	return ((resultSigned * pui16ReadBuffer[selectResult]) / 32768.0);
}

float calACos(float x)
{
	return (MATH_PI_DIV_2 - calASin(x));
}

float cosinesRuleForTriangles(float a, float b, float c)
{
	return (((a * a) + (b * b) - (c * c)) / (2 * a * b));
}

bool isTriangle(float a, float b, float c)
{
	if (((a + b) > c) && ((b + c) > a) && ((a + c) > b))
		return true;
	return false;
}

bool isValidTriangle(uint32_t a, uint32_t b, uint32_t c)
{
	float cosA;
	float cosB;
	float cosC;

	float fa = a / 256.0f;
	float fb = b / 256.0f;
	float fc = c / 256.0f;

	if (!isTriangle(fa, fb, fc))
		return false;

	cosA = cosinesRuleForTriangles(fb, fc, fa);
	if (cosA > COSINE_ANGLE_MIN)
	 return false;

	cosB = cosinesRuleForTriangles(fa, fc, fb);
	if (cosB > COSINE_ANGLE_MIN)
	 return false;

	cosC = cosinesRuleForTriangles(fa, fb, fc);
	if (cosC > COSINE_ANGLE_MIN)
	 return false;

	return true;
}
