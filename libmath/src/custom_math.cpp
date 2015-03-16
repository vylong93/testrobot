/*
 * custom_math.c
 *
 *  Created on: Mar 14, 2015
 *      Author: VyLong
 */

#include "libmath/inc/custom_math.h"

bool isValidTriangle(uint16_t a, uint16_t b, uint16_t c)
{
	float cosA;
	float cosB;
	float cosC;

	if (!isTriangle(a, b, c))
		return false;

	float fa = a / 256.0f;
	float fb = b / 256.0f;
	float fc = c / 256.0f;

	cosA = findCosAngleUseCosineRuleForTriangle(fb, fc, fa);
	if (cosA > COSINE_ANGLE_MIN)
	 return false;

	cosB = findCosAngleUseCosineRuleForTriangle(fa, fc, fb);
	if (cosB > COSINE_ANGLE_MIN)
	 return false;

	cosC = findCosAngleUseCosineRuleForTriangle(fa, fb, fc);
	if (cosC > COSINE_ANGLE_MIN)
	 return false;

	return true;
}

bool isTriangle(uint32_t a, uint32_t b, uint32_t c)
{
	/* The Triangle
	 *  Edges: a, b, c; Angle: A, B, C.
	 *		   .
	 *		  .  .
	 *       .  B  .
	 *	  c .        . a
	 *	   .           .
	 *    . A          C .
	 *   . . . . . . . . . .
	 *           b
	 */
	if (((a + b) > c) && ((b + c) > a) && ((a + c) > b))
		return true;
	return false;
}

float findCosAngleUseCosineRuleForTriangle(float fSide1, float fSide2, float fOppositeSide)
{
	/* The Triangle
	 *  Edges: a, b, c; Angle: C.
	 *		   .
	 *		  .  .
	 *       .     .
	 *	  c .        . a
	 *	   .           .
	 *    .           C  .
	 *   . . . . . . . . . .
	 *           b
	 */
	// Cosin Rule: c^2 = a^2 + b^2 - 2*a*b*cos(C)

	return ((fSide1*fSide1 + fSide2*fSide2 - fOppositeSide*fOppositeSide) / (2*fSide1*fSide2));
}



