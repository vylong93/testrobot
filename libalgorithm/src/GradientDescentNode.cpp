/*
 * GradientDescentNode.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: VyLong
 */

#include "libalgorithm/inc/GradientDescentNode.h"
#include "libstorage/inc/CustomLinkedList.h"
#include "libstorage/inc/RobotMeas.h"
#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotLocation.h"
#include "libstorage/inc/robot_data.h"

extern CustomLinkedList<RobotMeas> g_NeighborsTable;
extern CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
extern CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void GradientDescentNode::init(RobotLocation* pTarget, float fStepSize, float fStopCondition)
{
	StepSize = fStepSize;
	StopCondition = fStopCondition;

	pRobot = pTarget;

	EstimatePosNew = pRobot->vector;
	GradienNew = 0;

	isGradientSearchStop = false;

	oneHopIndex = OneHopNeighborsTable_getIndexOfRobot(pRobot->ID);
}

void GradientDescentNode::run(void)
{
	if((g_OneHopNeighborsTable.Count > 0 && oneHopIndex < 0) || (g_NeighborsTable.Count <= 0))
		return;

	uint16_t ui16Distance;
	float fDistanceMagnitude;
	float fTemplateParameter;
	Vector2<float> distanceVector;
	Vector2<float> vectVariance;

	RobotMeas destinationRobot;
	int destIndex;

	// Gradient Update
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		destinationRobot.ID = g_RobotLocationsTable[i].ID;

		if(g_OneHopNeighborsTable.Count > 0) // search in One Hop Neighbors table
		{
			destIndex = g_OneHopNeighborsTable[oneHopIndex].pNeighborsTable->isContain(destinationRobot);
			if (destIndex < 0)
				continue;
			ui16Distance = g_OneHopNeighborsTable[oneHopIndex].pNeighborsTable->ElementAt(destIndex).Distance;
		}
		else // search in Neighbors table
		{
			destIndex = g_NeighborsTable.isContain(destinationRobot);
			if (destIndex < 0)
				continue;
			ui16Distance = g_NeighborsTable.ElementAt(destIndex).Distance;
		}

		distanceVector =  pRobot->vector - g_RobotLocationsTable[i].vector;
		if (distanceVector.x == 0 && distanceVector.y == 0)
			distanceVector = 1;

		fDistanceMagnitude = distanceVector.getMagnitude();

		fTemplateParameter = (fDistanceMagnitude - (float)(ui16Distance / 256.0f)) / fDistanceMagnitude;

		GradienNew = GradienNew + distanceVector * fTemplateParameter;
	}

	// Position update
	EstimatePosOld = EstimatePosNew;
	EstimatePosNew = EstimatePosOld - GradienNew * StepSize;
	pRobot->vector = EstimatePosNew;
	GradienOld = GradienNew;
	GradienNew = 0;

	// Stop flag update
	vectVariance = EstimatePosNew - EstimatePosOld;

	vectVariance.x = (vectVariance.x < 0) ? (-vectVariance.x) : (vectVariance.x);
	vectVariance.y = (vectVariance.y < 0) ? (-vectVariance.y) : (vectVariance.y);

	isGradientSearchStop = (vectVariance.x < StopCondition) && (vectVariance.y < StopCondition);
}


