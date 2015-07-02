/*
 * DASHController.cpp
 *
 *  Created on: May 31, 2015
 *      Author: VyLong
 */

#include "libcontroller/inc/DASHController.h"
#include "libcontroller/inc/Controller.h"
#include "libmath/inc/custom_math.h"
#include "libalgorithm/inc/Trilateration.h"

bool DASHController::isTwoPositionOverlay(Vector2<float>* pPointA, Vector2<float>* pPointB, float errorInCm)
{
	Vector2<float> vectorAB;

	vectorAB.x = pPointB->x - pPointA->x;
	vectorAB.y = pPointB->y - pPointA->y;

	return (vectorAB.getMagnitude() < errorInCm);
}

bool DASHController::isHaveClearshotToTheGoal(Vector2<float>* pPointCurrent, Vector2<float>* pPointGoal)
{
	/* Pseudo-code
	   R |
	   M :--------::
	   R |_       |_..
	     |        |
	     :--------::
	     |
	         d    R M
	   Height = 2 * Robot's radius (5.0cm) + Moving Margin (2.5cm)
	   Width = distance between current and goal  +  Robot's radius (5.0cm) + Moving Margin (2.5cm)
	 */

	float marginCurrent = 2 * R_CENTER + MOVE_MARRGIN;
	Vector2<float> vectorToGoal(pPointGoal->x - pPointCurrent->x,
							    pPointGoal->y - pPointCurrent->y);

	float lengthOfVectorToGoal = sqrtf(vectorToGoal.x * vectorToGoal.x
			 	 	 	 	 	 	   + vectorToGoal.y * vectorToGoal.y);

	Vector2<float> vectorNormalizeToGoal(vectorToGoal.x / lengthOfVectorToGoal,
										 vectorToGoal.y / lengthOfVectorToGoal);

	Vector2<float> vectorMarginCurrent(marginCurrent * vectorNormalizeToGoal.x,
									   marginCurrent * vectorNormalizeToGoal.y);

	// rotate vectorMarginToGoal 90 and add pointCurrent
	Vector2<float> PointA(-vectorMarginCurrent.y + pPointCurrent->x,
						  vectorMarginCurrent.x + pPointCurrent->y);

	// rotate vectorMarginToGoal -90 and add pointCurrent
	Vector2<float> PointB(vectorMarginCurrent.y + pPointCurrent->x,
						  -vectorMarginCurrent.x + pPointCurrent->y);

	float marginGoal = marginCurrent + lengthOfVectorToGoal;
	Vector2<float> vectorMarginGoal(marginGoal * vectorNormalizeToGoal.x,
								    marginGoal * vectorNormalizeToGoal.y);

	Vector2<float> PointD(PointA.x + vectorMarginGoal.x,
						  PointA.y + vectorMarginGoal.y);

	// 2/ Check for collision: M(x,y) is inside if (0 < AM.AB < AB.AB) ^ (0 < AM.AD < AD.AD)
	Vector2<float> vectorAM;
	Vector2<float> vectorBM;
	Vector2<float> vectorDM;
	Vector2<float> vectorAB(PointB.x - PointA.x, PointB.y - PointA.y);
	Vector2<float> vectorAD(PointD.x - PointA.x, PointD.y - PointA.y);

	int i;
	int iNumberOfNeighbors = RobotLocationsTable_getSize();
	Vector2<float> neighborLocation;
	for(i = 0; i < iNumberOfNeighbors; i++)
	{
		if (RobotLocationsTable_getIdAtIndex(i) == pRobotIdentity->Self_ID)
			continue;

		RobotLocationsTable_getLocationAtIndex(i, &neighborLocation.x, &neighborLocation.y);

		// AB.AM tu continue
		vectorAM.x = neighborLocation.x - PointA.x;
		vectorAM.y = neighborLocation.y - PointA.y;
		if (vectorAB.DotProduct(vectorAM) < 0)
			continue;

		// AD.AM tu continue
		if (vectorAD.DotProduct(vectorAM) < 0)
			continue;

		// AB.BM nhon continue
		vectorBM.x = neighborLocation.x - PointB.x;
		vectorBM.y = neighborLocation.y - PointB.y;
		if (vectorAB.DotProduct(vectorBM) > 0)
			continue;

		// AD.DM nhon continue
		vectorDM.x = neighborLocation.x - PointD.x;
		vectorDM.y = neighborLocation.y - PointD.y;
		if (vectorAD.DotProduct(vectorDM) > 0)
			continue;

		return false;
	}
	return true;
}

DASHController::DASHController(GradientMap* pGMap, RobotIdentity_t* pRobotId)
{
	this->pGradientMap = pGMap;
	this->pRobotIdentity = pRobotId;
}

DASHController::~DASHController()
{}

void DASHController::calculateTheNextGoal(Vector2<float>* pPointNextGoal)
{
	/* Pseudo-code: Plan next noal use Gradient map
	   IF in center of gradient map THEN
	   	   IF in shape segment THEN
	   	   	   goal is the most positive GM value
	   	   ELSE IF	in [-1] of trapped segment THEN
	   	   	   // not implement yet
	   	   ELSE
	   	   	   goal is the larger GM value
	   	   END

	   	   // Modify next goal use Neighbors Location Table
	   	   IF the goal is stuck THEN
	   	   	   slide to edge point
	   	   END
	   ELSE
	   	   goal is the center of gradient map
	   END
	*/
	Vector2<float> pointCurrent(pRobotIdentity->x, pRobotIdentity->y);
	Vector2<float> pointCenter;
	pGradientMap->coordinateOfTheCenterGradientPixelOfRobotLocation(pointCurrent, pointCenter);

	Vector2<float> pointLeft(pointCenter.x - PIXEL_SIZE_IN_CM, pointCenter.y);
	Vector2<float> pointRight(pointCenter.x + PIXEL_SIZE_IN_CM, pointCenter.y);
	Vector2<float> pointUp(pointCenter.x, pointCenter.y + PIXEL_SIZE_IN_CM);
	Vector2<float> pointDown(pointCenter.x, pointCenter.y - PIXEL_SIZE_IN_CM);

	int32_t gmCenterValue = pGradientMap->valueOf(pointCenter);
	GradientUnit guRobot(&pointCurrent, gmCenterValue);

#define GRADIENT_UNIT_BUFFER_LENGTH	4
	GradientUnit pGu[GRADIENT_UNIT_BUFFER_LENGTH];
	pGu[0].setContent(&pointLeft, pGradientMap->valueOf(pointLeft));
	pGu[1].setContent(&pointRight, pGradientMap->valueOf(pointRight));
	pGu[2].setContent(&pointUp, pGradientMap->valueOf(pointUp));
	pGu[3].setContent(&pointDown, pGradientMap->valueOf(pointDown));

	selectionSortGradientUnitArray(pGu, GRADIENT_UNIT_BUFFER_LENGTH);

	reArangeGradientUnitArray(pGu, guRobot);

	bool bIsTargetUpdate = false;

	GradientUnit guGoal;
	int i;
//	e_SegmentType segmentType = pGradientMap->getSegmentType(pointCenter);
//	if(segmentType == SEGMENT_TRAPPED && guRobot.Value == START_PIXEL_VALUE_OF_NON_SHAPE_SEGMENT)
//	{
		//TODO: implement
//	}
//	else
//	{

		for(i = 0; i < GRADIENT_UNIT_BUFFER_LENGTH; i++)
		{
			if(pGu[i].Value > guRobot.Value)
			{
				guGoal.setContent(pGu[i].pPosition, pGu[i].Value);

				if(isHaveClearshotToTheGoal(&pointCurrent, guGoal.pPosition))
				{
					bIsTargetUpdate = true;
					break;
				}
			}
		}
//	}

	if (bIsTargetUpdate)
	{
		pPointNextGoal->x = guGoal.pPosition->x;
		pPointNextGoal->y = guGoal.pPosition->y;
		return;
	}
	else
	{
		if (!isTwoPositionOverlay(&pointCurrent, &pointCenter, CONTROLLER_POSITION_ERROR_CM)
				&& isHaveClearshotToTheGoal(&pointCurrent, &pointCenter))
		{
			pPointNextGoal->x = pointCenter.x;
			pPointNextGoal->y = pointCenter.y;
			return;
		}
	}
	pPointNextGoal->x = pointCurrent.x;
	pPointNextGoal->y = pointCurrent.y;
}

void DASHController::selectionSortGradientUnitArray(GradientUnit pGu[], int Length)
{
	if(Length <= 0)
		return;

	int indexMax;
	for(int i = 0; i < Length - 1; i++)
	{
		indexMax = i;
		int8_t valueMax = pGu[i].Value;
		for(int j = i + 1; j < Length; j++)
		{
			if(pGu[j].Value > valueMax)
			{
				valueMax = pGu[j].Value;
				indexMax = j;
			}
		}
		if(indexMax != i)
		{
			swapGradientUnit(pGu[indexMax], pGu[i]);
		}
	}
}

void DASHController::reArangeGradientUnitArray(GradientUnit pGu[], GradientUnit& guRobot)
{
	if(guRobot.Value < 0 && pGu[0].Value == pGu[1].Value)
	{
		if(guRobot.pPosition->x >= 0
				&& guRobot.pPosition->x < (float)(((int32_t)pGradientMap->Width - pGradientMap->ColumnOfStartShapePixel) * PIXEL_SIZE_IN_CM))
		{
			swapGradientUnit(pGu[0], pGu[1]);
		}
	}
}

void DASHController::swapGradientUnit(GradientUnit& a, GradientUnit& b)
{
	Vector2<float>* pTempPosition = a.pPosition;
	a.pPosition = b.pPosition;
	b.pPosition = pTempPosition;

	a.Value = a.Value ^ b.Value;
	b.Value = a.Value ^ b.Value;
	a.Value = a.Value ^ b.Value;
}

