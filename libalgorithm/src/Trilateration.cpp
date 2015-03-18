/*
 * Trilateration.c
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */
#include "libcustom/inc/custom_uart_debug.h"
#include "libalgorithm/inc/Trilateration.h"
#include "libalgorithm/inc/GradientDescentNode.h"
#include "libstorage/inc/robot_data.h"
#include "libprotocol/inc/network.h"
#include "libstorage/inc/CustomLinkedList.h"
#include "libstorage/inc/RobotLocation.h"
#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotMeas.h"
#include "libmath/inc/Vector2.h"
#include "libmath/inc/custom_math.h"
#include "librobot/inc/robot_analog.h"
#include <math.h>

extern CustomLinkedList<RobotMeas> g_NeighborsTable;
extern CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
extern CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void initDataOfRobot3(void)
{
	//	Robot Locations Table 0xbead01]: Count = 5
	//	___Robot [0xbead01] (0.0000cm;	0.0000cm)
	//	___Robot [0xbead05] (20.4219cm;	0.0000cm)
	//	___Robot [0xbead03] (20.5795cm;	20.6205cm)
	//	___Robot [0xbead08] (16.4993cm;	-19.6116cm)
	//	___Robot [0xbead04] (31.7628cm;	-18.6782cm)

	//	Neighbors Table of Robot [0xBEAD03]:
	//	Robot [0xBEAD05] :: 19.4492 cm
	//	Robot [0xBEAD08] :: 27.6055 cm
	//	Robot [0xBEAD01] :: 16.7422 cm
	//	Robot [0xBEAD04] :: 17.7695 cm

	//	One Hop Neighbors Table of Robot [0x00BEAD03]:
	//	first Hop ID = 0xBEAD04:
	//	Robot [0xBEAD03] :: 17.7695 cm
	//	Robot [0xBEAD01] :: 17.457 cm
	//	Robot [0xBEAD05] :: 31.0547 cm
	//	Robot [0xBEAD08] :: 19.4023 cm
	//
	//	first Hop ID = 0xBEAD01:
	//	Robot [0xBEAD03] :: 16.7422 cm
	//	Robot [0xBEAD05] :: 17.3672 cm
	//	Robot [0xBEAD08] :: 16.625 cm
	//	Robot [0xBEAD04] :: 17.457 cm
	//
	//	first Hop ID = 0xBEAD08:
	//	Robot [0xBEAD03] :: 27.6055 cm
	//	Robot [0xBEAD01] :: 16.625 cm
	//	Robot [0xBEAD04] :: 19.4023 cm
	//	Robot [0xBEAD05] :: 29.3555 cm
	//
	//	first Hop ID = 0xBEAD05:
	//	Robot [0xBEAD03] :: 19.4492 cm
	//	Robot [0xBEAD01] :: 17.3672 cm
	//	Robot [0xBEAD04] :: 31.0547 cm
	//	Robot [0xBEAD08] :: 29.3555 cm

	Network_setSelfAddress(0xBEAD03);
	DEBUG_PRINTS("init Data of Robot[0x%06x].........\n", Network_getSelfAddress());

	// (1) Fill Neighbors Table
	NeighborsTable_add(0xBEAD05, (uint16_t)(19.4492f * 256));
	NeighborsTable_add(0xBEAD08, (uint16_t)(27.6055f * 256));
	NeighborsTable_add(0xBEAD01, (uint16_t)(16.7422f * 256));
	NeighborsTable_add(0xBEAD04, (uint16_t)(17.7695f * 256));

	// (2) Fill OneHopNeighbors Table
	RobotMeas robot(0, 0);
	CustomLinkedList<RobotMeas> NeighborsTable;
	OneHopMeas oneHopMeas;

	oneHopMeas.firstHopID = 0xBEAD04;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(17.7695f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(17.457f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(31.0547f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(19.4023f * 256);
	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD01;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(16.7422f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(17.3672f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(16.625f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(17.457f * 256);
	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD08;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(27.6055f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(16.625f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(19.4023f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(29.3555f * 256);
	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD05;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(19.4492f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(17.3672f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(31.0547f * 256);
	NeighborsTable.add(robot);
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(29.3555f * 256);
	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	g_RobotLocationsTable.clearAll();

	DEBUG_PRINT("------------initialize Data OK\n");

	DEBUG_PRINT("Robots' position in real world\n");
	DEBUG_PRINT("          [3]   [4]\n");
	DEBUG_PRINT("                      \n");
	DEBUG_PRINT("       [5]   [1]   [8]\n");
	DEBUG_PRINT("------------------------------\n");
}

bool Tri_tryToCalculateRobotLocationsTable(void)
{
	/* Pseudo code
	 	(0) Verify Neighbors Table and OneHopNeighborTable to have the same distance
		(1) try to find two best neighbors as X, Y role
		IF found THEN
			(2) calculate X and Y locations
			(3) try to find all points from first two points
			(4) try to find all possible remain points
		END
	 */

	DEBUG_PRINT("Entering Tri_tryToCalculateRobotLocationsTable........\n");

	uint32_t ui32RobotOsId = Network_getSelfAddress();
	uint32_t ui32RobotXsId;
	uint32_t ui32RobotYsId;
	uint16_t ui16EdgeOX;
	uint16_t ui16EdgeOY;
	uint16_t ui16EdgeXY;

	//TODO: (0) Verify Neighbors Table and OneHopNeighborTable to have the same distance

	if (Tri_tryToFindTwoBestXYNeighbors(&ui32RobotXsId, &ui32RobotYsId, &ui16EdgeOX, &ui16EdgeOY, &ui16EdgeXY) == false)
	{
		DEBUG_PRINT("........Returning FALSE from Tri_tryToCalculateRobotLocationsTable\n");
		return false;
	}
	DEBUG_PRINTS2("Found Robot X [0x%06x], Robot Y [0x%06x]\n", ui32RobotXsId, ui32RobotYsId);

	Tri_calculateXYLocations(ui32RobotXsId, ui32RobotYsId, ui16EdgeOX, ui16EdgeOY, ui16EdgeXY);

	Tri_findAllPointsFromFirstTwoPoints(ui32RobotOsId, ui32RobotXsId, ui32RobotYsId, ui16EdgeOX, ui16EdgeOY, ui16EdgeXY);

	if(g_RobotLocationsTable.Count < g_NeighborsTable.Count)
		Tri_findAllPossibleRemainPoints();

//	GradientDescentMulti_correctLocationsTable(ui32RobotOsId);

	DEBUG_PRINT("........Returning TRUE from Tri_tryToCalculateRobotLocationsTable\n");
	return true;
}

bool Tri_tryToFindTwoBestXYNeighbors(uint32_t* pui32RobotXsId, uint32_t* pui32RobotYsId, uint16_t* pui16EdgeOX, uint16_t* pui16EdgeOY, uint16_t* pui16EdgeXY)
{
	DEBUG_PRINT("Entering Tri_tryToFindTwoBestXYNeighbors........\n");

	uint32_t ui32IdRobotX;
	uint32_t ui32IdRobotY;

	uint16_t ui16EdgeOX;
	uint16_t ui16EdgeXY;
	uint16_t ui16EdgeOY;

	CustomLinkedList<RobotMeas> NullNeighborsTable;
	OneHopMeas oneHopMeasX(0, &NullNeighborsTable);
	int oneHopIndexOfX;

	RobotMeas robotMeasY(0, 0);
	int neighborsIndexOfY;

	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		// (1) pick one neighbor play as X role
		ui32IdRobotX = g_NeighborsTable[i].ID;
		ui16EdgeOX = g_NeighborsTable[i].Distance;
		DEBUG_PRINTS2("Pick Robot X from O neighbors = [0x%06x :: OX = %2.4fcm]\n", ui32IdRobotX, ui16EdgeOX / 256.0f);

		// (1.1) map current i to g_OneHopNeighborsTable index to get corresponding X neighbors table
		oneHopMeasX.firstHopID = ui32IdRobotX;
		oneHopIndexOfX = g_OneHopNeighborsTable.isContain(oneHopMeasX);
		if(oneHopIndexOfX < 0)
			continue;
		//oneHopMeasX.pNeighborsTable = g_OneHopNeighborsTable[oneHopIndexOfX].pNeighborsTable;

		// (2) try to a second neighbor play as Y role
		int j;
		for(j = 0; j < g_OneHopNeighborsTable[oneHopIndexOfX].pNeighborsTable->Count; j++)
		{
			// pick one candidate in X neighbors for Y role
			robotMeasY = g_OneHopNeighborsTable[oneHopIndexOfX].pNeighborsTable->ElementAt(j);

			// make sure it not my ID
			if (robotMeasY.ID == Network_getSelfAddress())
				continue;

			ui32IdRobotY = robotMeasY.ID;
			ui16EdgeXY = robotMeasY.Distance;
			DEBUG_PRINTS2("Pick Robot Y from X neighbors = [0x%06x :: XY = %2.4fcm]\n", ui32IdRobotY, ui16EdgeXY / 256.0f);

			neighborsIndexOfY = g_NeighborsTable.isContain(robotMeasY);
			if(neighborsIndexOfY < 0)
				continue;

			ui16EdgeOY = g_NeighborsTable[neighborsIndexOfY].Distance;
			DEBUG_PRINTS("Robot Y in O neighbors OY = %2.4fcm\n", ui16EdgeOY / 256.0f);

			if (isValidTriangle(ui16EdgeOX, ui16EdgeOY, ui16EdgeXY)) // This function must be define at libmath
			{
				*pui32RobotXsId = ui32IdRobotX;
				*pui32RobotYsId = ui32IdRobotY;

				*pui16EdgeOX = ui16EdgeOX;
				*pui16EdgeOY = ui16EdgeOY;
				*pui16EdgeXY = ui16EdgeXY;

				DEBUG_PRINT("........Returning TRUE from Tri_tryToFindTwoBestXYNeighbors\n");
				return true;
			}
		}
	}

	*pui32RobotXsId = 0;
	*pui32RobotYsId = 0;

	*pui16EdgeOX = 0;
	*pui16EdgeOY = 0;
	*pui16EdgeXY = 0;

	DEBUG_PRINT("........Return FALSE from Tri_tryToFindTwoBestXYNeighbors\n");
	return false;
}

void Tri_calculateXYLocations(uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY)
{
	DEBUG_PRINT("Entering Tri_calculateXYLocations........\n");

	Vector2<float> vectorOX(0, 0);
	Vector2<float> vectorOY(0, 0);

	float fEdgeOX = ui16EdgeOX / 256.0f;
	float fEdgeOY = ui16EdgeOY / 256.0f;
	float fEdgeXY = ui16EdgeXY / 256.0f;
	DEBUG_PRINTS3("Valid Triangle: OX = %2.4f, OY = %2.4f, XY = %2.4f\n", fEdgeOX, fEdgeOY, fEdgeXY);

	float fCosAlphaYOX = findCosAngleUseCosineRuleForTriangle(fEdgeOX, fEdgeOY, fEdgeXY);
	float fSinAlphaYOX = sinf(acosf(fCosAlphaYOX));

	vectorOX.x = fEdgeOX;
	vectorOX.y = 0;
	RobotLocationsTable_add(ui32RobotXsId, vectorOX.x, vectorOX.y);
	DEBUG_PRINTS3("Add Robot X[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotXsId, vectorOX.x, vectorOX.y);

	vectorOY.x = fEdgeOY * fCosAlphaYOX;
	vectorOY.y = fEdgeOY * fSinAlphaYOX;
	RobotLocationsTable_add(ui32RobotYsId, vectorOY.x, vectorOY.y);
	DEBUG_PRINTS3("Add Robot Y[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotYsId, vectorOY.x, vectorOY.y);

	DEBUG_PRINT("........Returning from Tri_calculateXYLocations\n");
}

void Tri_findAllPointsFromFirstTwoPoints(uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY)
{
	/* Pseudo code
		LOOP through Neighbors Table to pick a Robot Z
			IF Robot Z already have locations THEN
				continue pick another neighbor to retry
			END

			// This condition never wrong due to State 1 has cover this
			IF I don't have Neighbors Table of Robot Z THEN
				continue pick another neighbor to retry
			END

			IF Robot Z's Neighbors Table not contain Robot O THEN
				continue pick another neighbor to retry
			END

			IF Robot Z's Neighbors Table not contain Robot X THEN
				continue pick another neighbor to retry
			END

			IF Robot Z's Neighbors Table not contain Robot Y THEN
				continue pick another neighbor to retry
			END

			get ZO, ZX and ZY distance
			calculate Robot Z location
			add to Locations Table
			END
		ENDLOOP
	 */

	DEBUG_PRINT("Entering Tri_findAllPointsFromFirstTwoPoints........\n");

	uint32_t ui32RobotZsId;
	int oneHopIndexsZ;
	int neighborsIndexsO;
	int neighborsIndexsX;
	int neighborsIndexsY;
	RobotLocation locOfRobotZ(0, NULL);
	RobotMeas robotMeas;
	Vector2<float> vectorZ;
	GradientDescentNode correctLocationAlgorithm;

	float fEdgeOZ, fEdgeXZ, fEdgeYZ;
	float fAlphaZ;

	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		ui32RobotZsId = g_NeighborsTable[i].ID;
		DEBUG_PRINTS("Pick Robot Z from O neighbors = [0x%06x]\n", ui32RobotZsId);

		if(RobotLocationTable_isContainRobot(ui32RobotZsId))
			continue;

		oneHopIndexsZ = OneHopNeighborsTable_getIndexOfRobot(ui32RobotZsId);
		if (oneHopIndexsZ < 0)	// This condition never wrong due to State 1 has cover this
			continue;

		robotMeas.ID = ui32RobotOsId;
		neighborsIndexsO = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->isContain(robotMeas);
		if (neighborsIndexsO < 0)
			continue;

		robotMeas.ID = ui32RobotXsId;
		neighborsIndexsX = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->isContain(robotMeas);
		if (neighborsIndexsX < 0)
			continue;

		robotMeas.ID = ui32RobotYsId;
		neighborsIndexsY = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->isContain(robotMeas);
		if (neighborsIndexsY < 0)
			continue;

		// Old Attempt:
//		vectorZ = Tri_trilaterateFromSixEdgeAndAngleOffset(
//				ui16EdgeOY, ui16EdgeOX,
//				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsO).Distance,
//				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsY).Distance,
//				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsX).Distance,
//				ui16EdgeXY, 0);
//		if (vectorZ.x == 0 && vectorZ.y == 0)
//			continue;

		// New Attempt:
		if(Tri_calculateAlphaFromSixEdge(&fAlphaZ, ui16EdgeOY, ui16EdgeOX,
				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsO).Distance,
				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsY).Distance,
				g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsX).Distance,
				ui16EdgeXY) == false)
			continue;

		fEdgeOZ = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsO).Distance / 256.0f;
		fEdgeXZ = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsX).Distance / 256.0f;
		fEdgeYZ = g_OneHopNeighborsTable[oneHopIndexsZ].pNeighborsTable->ElementAt(neighborsIndexsY).Distance / 256.0f;

		vectorZ = Tri_chooseTheBestVectorFromThreePoints(fAlphaZ, ui32RobotOsId, ui32RobotXsId, ui32RobotYsId, fEdgeOZ, fEdgeXZ, fEdgeYZ);
		if (vectorZ == NULL)
			continue;


		locOfRobotZ.ID = ui32RobotZsId;
		locOfRobotZ.vector = vectorZ;

//		// Each
//		correctLocationAlgorithm.init(&locOfRobotZ);
//		while(!correctLocationAlgorithm.isGradientSearchStop)
//		{
//			correctLocationAlgorithm.run();
//			DEBUG_PRINT("Robot:0x%06x (%2.4f; %2.4f)\n", locOfRobotZ.ID, locOfRobotZ.vector.x, locOfRobotZ.vector.y);
//		}

		g_RobotLocationsTable.add(locOfRobotZ);
		DEBUG_PRINTS3("Add Robot Z[0x%06x] (%2.4f; %2.4f) to Locations Table\n", locOfRobotZ.ID, locOfRobotZ.vector.x, locOfRobotZ.vector.y);
	}

	DEBUG_PRINT("........Returning from Tri_findAllPointsFromFirstTwoPoints\n");
}

void GradientDescentMulti_correctLocationsTable(uint32_t ui32OriginalID)
{
	DEBUG_PRINT("Entering GradientDescentMulti_correctLocationsTable........\n");

	bool isGradienttSearchContinue = true;
	GradientDescentNode* pTarget = NULL;
	CustomLinkedList<GradientDescentNode> listCorrectLocationAlgorithm;

	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if(g_RobotLocationsTable[i].ID == ui32OriginalID)
			continue;
		pTarget = new GradientDescentNode;
		pTarget->init(&g_RobotLocationsTable[i]);
		listCorrectLocationAlgorithm.add(*pTarget);
	}

	// active Gradient descent to calculate new position
	while(isGradienttSearchContinue)
	{
		isGradienttSearchContinue = false;

		for(i = 0; i < listCorrectLocationAlgorithm.Count; i++)
		{
			listCorrectLocationAlgorithm[i].run();
		}

		for(i = 0; i < listCorrectLocationAlgorithm.Count; i++)
		{
			if(listCorrectLocationAlgorithm[i].isGradientSearchStop == false)
			{
				isGradienttSearchContinue = true;
				break;
			}
		}
	}

	DEBUG_PRINT("........Returning from GradientDescentMulti_correctLocationsTable\n");
}

void Tri_findAllPossibleRemainPoints()
{
	/* Pseudo code
	   START LOOP
	   	   pick robot K in Neighbors Table

	   	   IF robot K already have locations THEN
	   	   	   pick another K to try again
	   	   END

		   // This condition never wrong due to State 1 has cover this
		   IF I don't have Neighbors Table of Robot K THEN
		   	   pick another K to try again
		   END

		   try to find two located robot in K neighbors table
	   	   IF success THEN
	   	   	   calculate location of robot K
	   	   	   add to Locations table
	   	   	   reset loop counter
	   	   ELSE
	   	   	   pick another K to try again
	   	   END
	   END
	 */

	uint32_t ui32RobotKsId;
	uint32_t ui32RobotPsId;
	uint32_t ui32RobotQsId;
	int oneHopIndexsK;
	RobotLocation locOfRobotK(0, NULL);
	OneHopMeas oneHopMeasOfRobotK(0, 0);
	RobotMeas robotMeas;
	Vector2<float> vectorK;

	uint16_t ui16EdgeOK;
	uint16_t ui16EdgePK;
	uint16_t ui16EdgeQK;
	uint16_t ui16EdgeOP;
	uint16_t ui16EdgeOQ;
	uint16_t ui16EdgePQ;

	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		ui32RobotKsId = g_NeighborsTable[i].ID;
		DEBUG_PRINTS("Pick Robot Z from O neighbors = [0x%06x]\n", ui32RobotKsId);

		locOfRobotK.ID = ui32RobotKsId;
		if (g_RobotLocationsTable.isContain(locOfRobotK) > 0)
			continue;

		oneHopMeasOfRobotK.firstHopID = ui32RobotKsId;
		oneHopIndexsK =	g_OneHopNeighborsTable.isContain(oneHopMeasOfRobotK);
		if (oneHopIndexsK < 0)	// This condition never wrong due to State 1 has cover this
			continue;

	   	//TODO: doi vi tri O, X, Y, Z cho nhau
		//Tri_tryToFindTwoLocatedNeighbors(g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable, &ui16EdgePK, ui16EdgeQK);
		bool bIsFoundP = false;
		bool bIsFoundQ = false;
		int j;
		for(j = 0; j < g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->Count; )
		{
			do
			{
				if (j >= g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->Count)
					break;

				if(RobotLocationTable_isContainRobot(g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID))
				{
					ui32RobotPsId = g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID;
					ui16EdgePK = g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).Distance;
					bIsFoundP = true;
				}
				j++;
			} while (!bIsFoundP);

			do
			{
				if (j >= g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->Count)
					break;

				if(RobotLocationTable_isContainRobot(g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID))
				{
					ui32RobotQsId = g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID;
					ui16EdgeQK = g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).Distance;
					bIsFoundQ = true;
				}
				j++;
			} while (!bIsFoundQ);
		}

	   	if (!(bIsFoundP && bIsFoundQ))
	   		continue;

		if(Tri_tryToGetEdgesOfThreeRobotOPQ(ui32RobotPsId, ui32RobotQsId, &ui16EdgeOP, &ui16EdgeOQ, &ui16EdgePQ) == false)
			continue;

		ui16EdgeOK = g_NeighborsTable[i].Distance;

		// (1) calculate location of robot K
		//float fEdgeOK = ui16EdgeOK / 256.0f;
		float fEdgePK = ui16EdgePK / 256.0f;
		float fEdgeQK = ui16EdgeQK / 256.0f;
		float fEdgeOP = ui16EdgeOP / 256.0f;
		float fEdgeOQ = ui16EdgePQ / 256.0f;

		float fAlphaP;
		float fAlphaQ;

		// tinh goc P va goc Q, ket qua la so duong
		Tri_tryToFindAngleInLocalCoordination(fEdgeOP, ui32RobotPsId, &fAlphaP);
		Tri_tryToFindAngleInLocalCoordination(fEdgeOQ, ui32RobotQsId, &fAlphaQ);

		// chon goc alphaP la goc nho hon trong 2 goc Q va P
		if (((fAlphaP > fAlphaQ) && ((fAlphaP - fAlphaQ) < MATH_PI))
				|| ((fAlphaQ - fAlphaP) > MATH_PI))
		{
			float fTempEdge = fEdgeOP;
			fEdgeOP = fEdgeOQ;
			fEdgeOQ = fTempEdge;

			fTempEdge = fEdgeQK;
			fEdgeQK = fEdgePK;
			fEdgePK = fTempEdge;

			fAlphaP = fAlphaQ;
		}

		// tinh toa do cua K roi offset goc P
		vectorK = Tri_trilaterateFromSixEdgeAndAngleOffset(ui16EdgeOQ, ui16EdgeOP, ui16EdgeOK,
				ui16EdgeQK, ui16EdgePK, ui16EdgePQ, fAlphaP);

		// (2) add to Locations table
		if (vectorK.x == 0 && vectorK.y == 0)
			continue;

//		locOfRobotK.x = vectorK.x;
//		locOfRobotK.y = vectorK.y;
		locOfRobotK.vector = vectorK;

		g_RobotLocationsTable.add(locOfRobotK);
		DEBUG_PRINTS3("Add Robot Z[0x%06x] (%2.4f; %2.4f) to Locations Table\n", locOfRobotK.ID, locOfRobotK.vector.x, locOfRobotK.vector.y);

		// (3) reset loop counter
		i = 0;
	}
}

bool Tri_tryToGetEdgesOfThreeRobotOPQ(uint32_t ui32RobotPsId, uint32_t ui32RobotQsId, uint16_t *pui16EdgeOP, uint16_t *pui16EdgeOQ, uint16_t *pui16EdgePQ)
{
	bool bFoundOP = false;
	bool bFoundOQ = false;
	bool bFoundPQ = false;

	*pui16EdgeOP = 0;
	*pui16EdgeOQ = 0;
	*pui16EdgePQ = 0;

	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		if (g_NeighborsTable[i].ID == ui32RobotPsId && bFoundOP == false)
		{
			*pui16EdgeOP = g_NeighborsTable[i].Distance;
			bFoundOP = true;
		}

		if (g_NeighborsTable[i].ID == ui32RobotQsId && bFoundOQ == false)
		{
			*pui16EdgeOQ = g_NeighborsTable[i].Distance;
			bFoundOQ = true;
		}
	}

	if(bFoundOP && bFoundOQ)
	{
		int oneHopIndexsO;
		OneHopMeas oneHopMeasOfRobot(ui32RobotPsId, NULL);
		oneHopIndexsO = g_OneHopNeighborsTable.isContain(oneHopMeasOfRobot);
		if (oneHopIndexsO > 0)
		{
			int j;
			for(j = 0; j < g_OneHopNeighborsTable[oneHopIndexsO].pNeighborsTable->Count; j++)
			{
				if (g_OneHopNeighborsTable[oneHopIndexsO].pNeighborsTable->ElementAt(j).ID == ui32RobotQsId && bFoundPQ == false)
				{
					*pui16EdgePQ = g_OneHopNeighborsTable[oneHopIndexsO].pNeighborsTable->ElementAt(j).Distance;
					bFoundPQ = true;
				}
			}
		}
	}

	return (bFoundOP && bFoundOQ && bFoundPQ);
}

bool Tri_tryToFindAngleInLocalCoordination(float fEdgeR, uint32_t ui32RobotId, float* pfAlpha)
{
	//TODO: rename function

	float fCosAlpha;

	int i;
	for (i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if (g_RobotLocationsTable[i].ID == ui32RobotId)
		{
			fCosAlpha = g_RobotLocationsTable[i].vector.x / fEdgeR;
			*pfAlpha = acosf(fCosAlpha);

			if (g_RobotLocationsTable[i].vector.y < 0)
				*pfAlpha = MATH_PI_MUL_2 - *pfAlpha;

			return true;
		}
	}
	return false;
}

bool Tri_calculateAlphaFromSixEdge(float* pfAlphaPIJ, uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ,
		uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ)
{
	DEBUG_PRINT("Entering Tri_calculateAlphaFromSixEdgeAndAngleOffset........\n");

	if (!isTriangle(ui16EdgeIP, ui16EdgeIJ, ui16EdgePJ))
		return false;

	if (!isTriangle(ui16EdgeIQ, ui16EdgeIJ, ui16EdgeQJ))
		return false;

	if (!isTriangle(ui16EdgeIP, ui16EdgeIQ, ui16EdgePQ))
		return false;

	float fEdgeIP = ui16EdgeIP / 256.0f;
	float fEdgeIJ = ui16EdgeIJ / 256.0f;
	float fEdgePJ = ui16EdgePJ / 256.0f;

	*pfAlphaPIJ = acosf(findCosAngleUseCosineRuleForTriangle(fEdgeIP, fEdgeIJ, fEdgePJ));

	DEBUG_PRINT("........Returning from Tri_calculateAlphaFromSixEdge\n");

	return true;
}

Vector2<float> Tri_chooseTheBestVectorFromThreePoints(float fAlphaZ, uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ)
{
	int indexO, indexX, indexY;

	Vector2<float> vectorZNegSigned;
	Vector2<float> vectorZPosSigned;

	float errorNegSigned = 0;
	float errorPosSigned = 0;

	indexO = RobotLocationTable_getIndexOfRobot(ui32RobotOsId);
	if(indexO < 0)	return NULL;

	indexX = RobotLocationTable_getIndexOfRobot(ui32RobotXsId);
	if(indexX < 0)	return NULL;

	indexY = RobotLocationTable_getIndexOfRobot(ui32RobotYsId);
	if(indexY < 0)	return NULL;

	vectorZPosSigned.x = fEdgeOZ * cosf(fAlphaZ);
	vectorZPosSigned.y = fEdgeOZ * sinf(fAlphaZ);

	vectorZNegSigned.x = fEdgeOZ * cosf(-fAlphaZ);
	vectorZNegSigned.y = fEdgeOZ * sinf(-fAlphaZ);

	errorPosSigned = Tri_calculateErrorOfVectorZ(vectorZPosSigned,
												g_RobotLocationsTable[indexO].vector,
												g_RobotLocationsTable[indexX].vector,
												g_RobotLocationsTable[indexY].vector,
												fEdgeOZ, fEdgeXZ, fEdgeYZ);

	errorNegSigned = Tri_calculateErrorOfVectorZ(vectorZNegSigned,
												g_RobotLocationsTable[indexO].vector,
												g_RobotLocationsTable[indexX].vector,
												g_RobotLocationsTable[indexY].vector,
												fEdgeOZ, fEdgeXZ, fEdgeYZ);

	if(errorPosSigned < errorNegSigned)
		return vectorZPosSigned;
	else
		return vectorZNegSigned;
}
float Tri_calculateErrorOfVectorZ(Vector2<float> vectZ, Vector2<float> vectO, Vector2<float> vectX, Vector2<float> vectY, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ)
{
	Vector2<float> vectOZ = vectO - vectZ;
	Vector2<float> vectXZ = vectX - vectZ;
	Vector2<float> vectYZ = vectY - vectZ;

	float errorOZ = vectOZ.getMagnitude() - fEdgeOZ;
	float errorXZ = vectXZ.getMagnitude() - fEdgeXZ;
	float errorYZ = vectYZ.getMagnitude() - fEdgeYZ;

	errorOZ = absFloatNumber(errorOZ);
	errorXZ = absFloatNumber(errorXZ);
	errorYZ = absFloatNumber(errorYZ);

	return (errorOZ + errorXZ + errorYZ);
}

Vector2<float> Tri_trilaterateFromSixEdgeAndAngleOffset(uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ,
		uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ, float fAngleOffset)
{
	DEBUG_PRINT("Entering Tri_trilaterateFromSixEdgeAndAngleOffset........\n");

	Vector2<float> vectorZ(0, 0);

	if (!isTriangle(ui16EdgeIP, ui16EdgeIJ, ui16EdgePJ))
		return vectorZ;

	if (!isTriangle(ui16EdgeIQ, ui16EdgeIJ, ui16EdgeQJ))
		return vectorZ;

	if (!isTriangle(ui16EdgeIP, ui16EdgeIQ, ui16EdgePQ))
		return vectorZ;

	float fEdgeIQ = ui16EdgeIQ / 256.0f;
	float fEdgeIP = ui16EdgeIP / 256.0f;
	float fEdgeIJ = ui16EdgeIJ / 256.0f;
	float fEdgeQJ = ui16EdgeQJ / 256.0f;
	float fEdgePJ = ui16EdgePJ / 256.0f;
	float fEdgePQ = ui16EdgePQ / 256.0f;

	float fAlphaPIJ = acosf(findCosAngleUseCosineRuleForTriangle(fEdgeIP, fEdgeIJ, fEdgePJ));
	float fBetaQIJ = acosf(findCosAngleUseCosineRuleForTriangle(fEdgeIQ, fEdgeIJ, fEdgeQJ));
	float fThetaQIP = acosf(findCosAngleUseCosineRuleForTriangle(fEdgeIP, fEdgeIQ, fEdgePQ));

	int8_t signedAlphaJ = Tri_findSignedYaxis(fAlphaPIJ, fBetaQIJ, fThetaQIP);

	if (signedAlphaJ == 0)
			return vectorZ;

	float fAlphaJ;
	fAlphaJ = fAlphaPIJ * signedAlphaJ;
	fAlphaJ += fAngleOffset;

	vectorZ.x = fEdgeIJ * cosf(fAlphaJ);
	vectorZ.y = fEdgeIJ * sinf(fAlphaJ);

	DEBUG_PRINT("........Returning from Tri_trilaterateFromSixEdgeAndAngleOffset\n");
	return vectorZ;
}

int8_t Tri_findSignedYaxis(float fAlpha, float fBeta, float fTheta)
{
	float fPhi = fAlpha - fTheta;

	fPhi = (fPhi > 0) ? (fPhi) : (-fPhi);

	if ((fBeta < (fPhi + ANGLE_MIN_IN_RAD))
			&& (fBeta > (fPhi - ANGLE_MIN_IN_RAD)))
		return (1);

//	fTemp = fAlpha + fTheta;
//	if ((fBeta < (fTemp + ANGLE_MIN_IN_RAD))
//			&& (fBeta > (fTemp - ANGLE_MIN_IN_RAD)))
		return (-1);
//
//	return (0);
}
