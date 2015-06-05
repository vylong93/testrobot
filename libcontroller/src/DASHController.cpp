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
	// 0/ Find the rectangle
	Vector2<float> point[4];

	Vector2<float> n(pPointGoal->x - pPointCurrent->x, pPointGoal->y - pPointCurrent->y);
	LineEquation_t lineCurrent;
	lineCurrent.a = n.x;
	lineCurrent.b = n.y;
	lineCurrent.c = -(lineCurrent.a * pPointCurrent->x + lineCurrent.b * pPointCurrent->y);
	CircleEquation_t circleCurrent;
	circleCurrent.x0 = pPointCurrent->x;
	circleCurrent.y0 = pPointCurrent->y;
	circleCurrent.R = 2 * R_CENTER + MOVE_MARRGIN;
	if (!Tri_calculateTwoCrossPointBetweenLineAndCircle(lineCurrent, circleCurrent, &point[0], &point[1]))
		return false;

	float distance = n.getMagnitude() + R_CENTER + MOVE_MARRGIN;
	n.normalize();
	n = n * distance;
	n.x = n.x + pPointCurrent->x;
	n.y = n.y + pPointCurrent->y;

	LineEquation_t lineGoal;
	lineGoal.a = lineCurrent.a;
	lineGoal.b = lineCurrent.b;
	lineGoal.c = -(lineGoal.a * n.x + lineGoal.b * n.y);
	CircleEquation_t circleGoal;
	circleGoal.x0 = n.x;
	circleGoal.y0 = n.y ;
	circleGoal.R = 2 * R_CENTER + MOVE_MARRGIN;
	if (!Tri_calculateTwoCrossPointBetweenLineAndCircle(lineGoal, circleGoal, &point[2], &point[3]))
		return false;

	// 1/ Find point arrangement
	Vector2<float> vect01(point[1].x - point[0].x, point[1].y - point[0].y);
	Vector2<float> vect02(point[2].x - point[0].x, point[2].y - point[0].y);
	Vector2<float> vect03(point[3].x - point[0].x, point[3].y - point[0].y);
	Vector2<float>* pPointA;
	Vector2<float>* pPointB;
	Vector2<float>* pPointD;
	if (fabsf(vect01.DotProduct(vect02)) < 0.02)
	{
		pPointA = &point[0];
		pPointB = &point[1];
		pPointD = &point[2];
	}
	else if (fabsf(vect01.DotProduct(vect03)) < 0.02)
	{
		pPointA = &point[0];
		pPointB = &point[1];
		pPointD = &point[3];
	}
	else if (fabsf(vect02.DotProduct(vect03)) < 0.02)
	{
		pPointA = &point[0];
		pPointB = &point[2];
		pPointD = &point[3];
	}
	else
		return false;

	// 2/ Check for collision: M(x,y) is inside if (0 < AM.AB < AB.AB) ^ (0 < AM.AD < AD.AD)
	Vector2<float> vectorAM;
	Vector2<float> vectorBM;
	Vector2<float> vectorDM;
	Vector2<float> vectorAB(pPointB->x - pPointA->x, pPointB->y - pPointA->y);
	Vector2<float> vectorAD(pPointD->x - pPointA->x, pPointD->y - pPointA->y);

	int i;
	int iNumberOfNeighbors = RobotLocationsTable_getSize();
	Vector2<float> neighborLocation;
	for(i = 0; i < iNumberOfNeighbors; i++)
	{
		if (RobotLocationsTable_getIdAtIndex(i) == pRobotIdentity->Self_ID)
			continue;

		RobotLocationsTable_getLocationAtIndex(i, &neighborLocation.x, &neighborLocation.y);

		// AB.AM tu continue
		vectorAM.x = neighborLocation.x - pPointA->x;
		vectorAM.y = neighborLocation.y - pPointA->y;
		if (vectorAB.DotProduct(vectorAM) < 0)
			continue;

		// AD.AM tu continue
		if (vectorAD.DotProduct(vectorAM) < 0)
			continue;

		// AB.BM nhon continue
		vectorBM.x = neighborLocation.x - pPointB->x;
		vectorBM.y = neighborLocation.y - pPointB->y;
		if (vectorAB.DotProduct(vectorBM) > 0)
			continue;

		// AD.DM nhon continue
		vectorDM.x = neighborLocation.x - pPointD->x;
		vectorDM.y = neighborLocation.y - pPointD->y;
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

	if (isTwoPositionOverlay(&pointCurrent, &pointCenter, CONTROLLER_POSITION_ERROR_CM * 1.7f))
	{
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
		e_SegmentType segmentType = pGradientMap->getSegmentType(pointCenter);
		if (segmentType == SEGMENT_SHAPE)
		{
			for(i = 0; i < GRADIENT_UNIT_BUFFER_LENGTH; i++)
			{
				if(pGu[i].Value > 0 && pGu[i].Value > guRobot.Value)
				{
					guGoal.setContent(pGu[i].pPosition, pGu[i].Value);

					if(isHaveClearshotToTheGoal(&pointCurrent, guGoal.pPosition))
					{
						bIsTargetUpdate = true;
						break;
					}
				}
			}
		}
//		else if (segmentType == SEGMENT_TRAPPED
//				&& gmCurrentValue == START_PIXEL_VALUE_OF_NON_SHAPE_SEGMENT)
//		{
//			//TODO: implement
//		}
		else // External Segment & Trapped Segment
		{
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
#ifdef SLIDING_SHAPE_EDGE_BEHAVIOUR
					else
					{
						if(pGu[i].Value >= 0) // at outside of the bolder of shape segment
							break;
					}
#endif
				}
			}
		}

		if (bIsTargetUpdate)
		{
			pPointNextGoal->x = guGoal.pPosition->x;
			pPointNextGoal->y = guGoal.pPosition->y;
			return;
		}

#ifdef SLIDING_SHAPE_EDGE_BEHAVIOUR
		// Target cannot update, calculate the sliding goal
		if(guRobot.Value < 0 && guGoal.Value >= 0) // at outside of the bolder of shape segment
		{
			Vector2<float> vectorToGoal(guGoal.pPosition->x - pointCurrent.x, guGoal.pPosition->y - pointCurrent.y);
			vectorToGoal.normalize();

			Vector2<float> vectorToSlidingGoal = vectorToGoal.getRotate(MATH_PI_DIV_2);
			vectorToSlidingGoal.x *= PIXEL_SIZE_IN_CM;
			vectorToSlidingGoal.y *= PIXEL_SIZE_IN_CM;

			vectorToSlidingGoal.x += pointCurrent.x;
			vectorToSlidingGoal.y += pointCurrent.y;

			bool bIsSlidingGoalStuck = false;

			bool bIsLeftPositive = (pGu[0].Value >= 0);
			bool bIsRightPositive = (pGu[1].Value >= 0);
			bool bIsUpPositive = (pGu[2].Value >= 0);
			bool bIsDownPositive = (pGu[3].Value >= 0);

			bool bLeftClear = true;
			bool bRightClear = true;
			bool bUpClear = true;
			bool bDownClear = true;

			Vector2<float> neighborLocation;
			for(i = 0; i < RobotLocationsTable_getSize(); i++)
			{
				RobotLocationsTable_getLocationAtIndex(i, &neighborLocation.x, &neighborLocation.y));
				if (isTwoPositionOverlay(&vectorToSlidingGoal, &neighborLocation, CONTROLLER_POSITION_MOVE_MARRGIN_CM))
				{
					pGradientMap->coordinateOfTheCenterGradientPixelOfRobotLocation(neighborLocation, *(guGoal.pPosition));
					guGoal.Value = pGradientMap->valueOf((*guGoal.pPosition));
					bIsSlidingGoalStuck = true;
				}

				if(isTwoPositionOverlay(pointLeft, &neighborLocation, CONTROLLER_POSITION_MOVE_MARRGIN_CM))
					bLeftClear = false;

				if(isTwoPositionOverlay(pointRight, &neighborLocation, CONTROLLER_POSITION_MOVE_MARRGIN_CM))
					bRightClear = false;

				if(isTwoPositionOverlay(pointUp, &neighborLocation, CONTROLLER_POSITION_MOVE_MARRGIN_CM)
					bUpClear = false;

				if(isTwoPositionOverlay(pointDown, &neighborLocation, CONTROLLER_POSITION_MOVE_MARRGIN_CM))
					bDownClear = false;
			}

			if(bLeftClear && bIsLeftPositive)
			{
				pPointNextGoal->x = pointLeft.x;
				pPointNextGoal->y = pointLeft.y;
				return;
			}

			if(bRightClear && bIsRightPositive)
			{
				pPointNextGoal->x = pointRight.x;
				pPointNextGoal->y = pointRight.y;
				return;
			}

			if(bUpClear && bIsUpPositive)
			{
				pPointNextGoal->x = pointUp.x;
				pPointNextGoal->y = pointUp.y;
				return;
			}

			if(bDownClear && bIsDownPositive)
			{
				pPointNextGoal->x = pointDown.x;
				pPointNextGoal->y = pointDown.y;
				return;
			}

			if (!bIsSlidingGoalStuck)
			{
				pPointNextGoal->x = vectorToSlidingGoal->x;
				pPointNextGoal->y = vectorToSlidingGoal->y;
				return;
			}
		}
#endif

		pPointNextGoal->x = pRobotIdentity->x;
		pPointNextGoal->y = pRobotIdentity->y;
		return;
	}
	else
	{
		if(isHaveClearshotToTheGoal(&pointCurrent, &pointCenter))
		{
			pPointNextGoal->x = pointCenter.x;
			pPointNextGoal->y = pointCenter.y;
			return;
		}
		else
		{
			pPointNextGoal->x = pRobotIdentity->x;
			pPointNextGoal->y = pRobotIdentity->y;
		}
	}
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

