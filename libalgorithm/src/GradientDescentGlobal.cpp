/*
 * GradientDescentGlobal.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: VyLong
 */

#include "libalgorithm/inc/GradientDescentGlobal.h"
#include "librobot/inc/robot_analog.h"
#include "libstorage/inc/CustomLinkedList.h"
#include "libstorage/inc/RobotMeas.h"
#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotLocation.h"
#include "libstorage/inc/robot_data.h"

extern CustomLinkedList<RobotMeas> g_NeighborsTable;
extern CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
extern CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void GradientDescent_updateGradient(uint32_t ui32SelfId, Vector2<float> vectSelf, Vector2<float>* pvectGradientNew, bool bEnableRandomCalculation)
{
	uint8_t ui8RandomValue;

	uint16_t ui16Distance;
	float fDistanceMagnitude;
	float fTemplateParameter;
	Vector2<float> vectorDistance;
	Vector2<float> vectVariance;
	Vector2<float> vectTarget;
	int i;
	for (i = 0; i < g_NeighborsTable.Count; i++)
	{
		if (g_NeighborsTable[i].ID == ui32SelfId)
			continue;

		if (bEnableRandomCalculation)
		{
			ui8RandomValue = generateRandomByte();
			if (ui8RandomValue < 0x80) // ~50%
				continue;
		}

		ui16Distance = g_NeighborsTable[i].Distance;

		if(!RobotLocationsTable_getVectorOfRobot(g_NeighborsTable[i].ID, &vectTarget.x, &vectTarget.y))
			continue;

		vectorDistance = vectSelf - vectTarget;
		if (vectorDistance.x == 0 && vectorDistance.y == 0)
		{
			vectorDistance.x = 1;
			vectorDistance.y = 1;
		}

		fDistanceMagnitude = vectorDistance.getMagnitude();

		fTemplateParameter = (fDistanceMagnitude - (float) ((ui16Distance / 256.0f))) / fDistanceMagnitude;

		(*pvectGradientNew) = (*pvectGradientNew) + vectorDistance * fTemplateParameter;
	}
}

void GradientDescent_updatePosition(Vector2<float>* pvectAverageCoordination,
		Vector2<float>* pvectEstimatePosNew, Vector2<float>* pvectEstimatePosOld,
		Vector2<float>* pvectGradientNew, Vector2<float>* pvectGradientOld, float fStepSize)
{
	(*pvectEstimatePosOld) = (*pvectEstimatePosNew);

	(*pvectEstimatePosNew) = (*pvectEstimatePosOld) - (*pvectGradientNew) * fStepSize;

	(*pvectAverageCoordination) = (*pvectEstimatePosNew);

	(*pvectGradientOld) = (*pvectGradientNew);

	pvectGradientNew->x = 0;
	pvectGradientNew->y = 0;
}

bool GradientDescent_checkVarianceCondition(Vector2<float> vectEstimatePosNew,
		Vector2<float> vectEstimatePosOld, float fStopCondition)
{
	Vector2<float> vectVariance;

	vectVariance = vectEstimatePosNew - vectEstimatePosOld;

	vectVariance.x = (vectVariance.x < 0) ? (-vectVariance.x) : (vectVariance.x);
	vectVariance.y = (vectVariance.y < 0) ? (-vectVariance.y) : (vectVariance.y);

	return ((vectVariance.x < fStopCondition) && (vectVariance.y < fStopCondition));
}
