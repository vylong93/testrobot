/*
 * MovementSavePoint.c
 *
 *  Created on: May 27, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/MovementSavePoint.h"
#include "libmath/inc/custom_math.h"
#include "libcontroller/inc/Controller.h"

Vector2<float> g_pSavePoint[NUMBER_OF_SAVE_POINTS];
int g_Pointer = 0;
int PointCounter = 0;

int getSavePointCounter(void)
{
	return PointCounter;
}

void pushNewPoint(float x, float y)
{
	g_pSavePoint[g_Pointer].x = x;
	g_pSavePoint[g_Pointer].y = y;

	g_Pointer++;
	g_Pointer = (g_Pointer >= NUMBER_OF_SAVE_POINTS) ? (0) : (g_Pointer);

	PointCounter++;
	PointCounter = (PointCounter >= NUMBER_OF_SAVE_POINTS) ? (NUMBER_OF_SAVE_POINTS) : (PointCounter);
}

bool getLastPoint(int offset, Vector2<float>* pPointOut)
{
	int index = g_Pointer - 1 + offset;

	if (index < 0)
		index += NUMBER_OF_SAVE_POINTS;

	if(index >= PointCounter)
		return false;

	pPointOut->x = g_pSavePoint[index].x;
	pPointOut->y = g_pSavePoint[index].y;
	return true;
}

bool calculateLastForwardOrientation(float *pTheta)
{
	if (PointCounter < 2)
		return false;

	Vector2<float> pointNow;
	Vector2<float> pointPre;

	getLastPoint(0, &pointNow);
	getLastPoint(-1, &pointPre);

	pointNow.x = pointNow.x - pointPre.x;
	pointNow.y = pointNow.y - pointPre.y;

	if (pointPre.getMagnitude() < (CONTROLLER_POSITION_ERROR_CM * 1.3f)) // 3.9cm
		return false;

	*pTheta = atan2f(pointNow.y, pointNow.x);
	return true;
}

bool calculateLastBackwardOrientation(float *pTheta)
{
	if (PointCounter < 2)
		return false;

	Vector2<float> pointNow;
	Vector2<float> pointPre;

	getLastPoint(0, &pointNow);
	getLastPoint(-1, &pointPre);

	pointPre.x = pointPre.x - pointNow.x;
	pointPre.y = pointPre.y - pointNow.y;

	if (pointPre.getMagnitude() < (CONTROLLER_POSITION_ERROR_CM * 1.3f)) // 3.9cm
		return false;

	*pTheta = atan2f(pointPre.y, pointPre.x);
	return true;
}

e_Locomotion tryToCalculateLastLocomotion(Vector2<float> pPoint[])
{
	Vector2<float> diff;

	diff.x = pPoint[1].x - pPoint[0].x;
	diff.y = pPoint[1].y - pPoint[0].y;
	float alphaP = atan2f(diff.y, diff.x);
	alphaP = (alphaP < 0) ? (alphaP + MATH_PI_MUL_2) : (alphaP);

	diff.x = pPoint[2].x - pPoint[0].x;
	diff.y = pPoint[2].y - pPoint[0].y;
	float alphaQ = atan2f(diff.y, diff.x);
	alphaQ = (alphaQ < 0) ? (alphaQ + MATH_PI_MUL_2) : (alphaQ);

	if (((alphaP > alphaQ) && ((alphaP - alphaQ) < MATH_PI))
			|| ((alphaQ - alphaP) > MATH_PI))
		return LOCOMOTION_DIFFERENT;
	else
		return LOCOMOTION_SAME;
}
