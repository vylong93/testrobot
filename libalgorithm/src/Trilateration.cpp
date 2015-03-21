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
#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotMeas.h"
#include "libmath/inc/Vector2.h"
#include "libmath/inc/custom_math.h"
#include "librobot/inc/robot_analog.h"
#include "data_manipulation.h"
#include <math.h>

extern "C" void setRobotIdentityVector(float x, float y);

extern CustomLinkedList<RobotMeas> g_NeighborsTable;
extern CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
extern CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void initData(uint8_t* pui8MessageData)
{
//	Locations Table of Robot [0x00BEAD04]:
//	Robot:0xBEAD04 (0; 0)
//	Robot:0xBEAD05 (33.676; 0.111511)
//	Robot:0xBEAD01 (15.3616; 8.43095)
//	Robot:0xBEAD03 (24.5245; 22.4866)
//	Robot:0xBEAD08 (4.43593; 20.7595)

	RobotLocation item(0,0);
	item.ID = 0xBEAD04; item.vector.x = 0; 			item.vector.y = 0;			g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD05; item.vector.x = 33.676f; 	item.vector.y = 0.111511f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD01; item.vector.x = 15.3616f; 	item.vector.y = 8.43095f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD03; item.vector.x = 24.5245f; 	item.vector.y = 22.4866f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD08; item.vector.x = 4.43593f; 	item.vector.y = 20.7595f;	g_RobotLocationsTable.add(item);

//	Locations Table of Robot [0x00BEAD05]:
//	Robot:0xBEAD05 (21.3613; 0.266083)
//	Robot:0xBEAD04 (-9.41879; 13.8079)
//	Robot:0xBEAD01 (1.23299; 0.0153503)
//	Robot:0xBEAD03 (3.86044; -16.5552)
//	Robot:0xBEAD08 (-13.7907; -6.79756)

	CustomLinkedList<RobotLocation> OriLocationsTable;
	item.ID = 0xBEAD05; item.vector.x = 21.3613f; 	item.vector.y = 0.266083f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD04; item.vector.x = -9.41879f; 	item.vector.y = 13.8079f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD01; item.vector.x = 1.23299f; 	item.vector.y = 0.0153503f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD03; item.vector.x = 3.86044f; 	item.vector.y = -16.5552f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD08; item.vector.x = -13.7907f; 	item.vector.y = -6.79756f;		OriLocationsTable.add(item);

	int32_t i32Template;
	int pointer = 0;
	int i;
	for(i = 0; i < OriLocationsTable.Count; i++)
	{
		parse32bitTo4Bytes(&pui8MessageData[pointer], OriLocationsTable[i].ID);
		pointer += 4;

		i32Template = (int32_t)(OriLocationsTable[i].vector.x * 65535 + 0.5);
		parse32bitTo4Bytes(&pui8MessageData[pointer], i32Template);
		pointer += 4;

		i32Template = (int32_t)(OriLocationsTable[i].vector.y * 65535 + 0.5);
		parse32bitTo4Bytes(&pui8MessageData[pointer], i32Template);
		pointer += 4;
	}
}

bool Tri_tryToCalculateRobotLocationsTable(uint32_t ui32RobotOsId)
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

	uint32_t ui32RobotXsId;
	uint32_t ui32RobotYsId;
	uint16_t ui16EdgeOX;
	uint16_t ui16EdgeOY;
	uint16_t ui16EdgeXY;

	//TODO: (0) Verify Neighbors Table and OneHopNeighborTable to have the same distance

	if (Tri_tryToFindTwoBestXYNeighbors(ui32RobotOsId, &ui32RobotXsId, &ui32RobotYsId, &ui16EdgeOX, &ui16EdgeOY, &ui16EdgeXY) == false)
	{
		DEBUG_PRINT("........Returning FALSE from Tri_tryToCalculateRobotLocationsTable\n");
		return false;
	}
	DEBUG_PRINTS2("Found Robot X [0x%06x], Robot Y [0x%06x]\n", ui32RobotXsId, ui32RobotYsId);

	Tri_calculateXYLocations(ui32RobotXsId, ui32RobotYsId, ui16EdgeOX, ui16EdgeOY, ui16EdgeXY);

	Tri_findAllPointsFromFirstTwoPoints(ui32RobotOsId, ui32RobotXsId, ui32RobotYsId, ui16EdgeOX, ui16EdgeOY, ui16EdgeXY);

	if(g_RobotLocationsTable.Count < g_NeighborsTable.Count)
		Tri_findAllPossibleRemainPoints();

	GradientDescentMulti_correctLocationsTable(ui32RobotOsId, 0);

	DEBUG_PRINT("........Returning TRUE from Tri_tryToCalculateRobotLocationsTable\n");
	return true;
}

bool Tri_tryToFindTwoBestXYNeighbors(uint32_t ui32RobotOsId, uint32_t* pui32RobotXsId, uint32_t* pui32RobotYsId, uint16_t* pui16EdgeOX, uint16_t* pui16EdgeOY, uint16_t* pui16EdgeXY)
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
			if (robotMeasY.ID == ui32RobotOsId)
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

		if(RobotLocationsTable_isContainRobot(ui32RobotZsId))
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

void GradientDescentMulti_correctLocationsTable(uint32_t ui32OriginalID, uint32_t ui32RotationHopID)
{
	DEBUG_PRINT("Entering GradientDescentMulti_correctLocationsTable........\n");

	bool isGradienttSearchContinue = true;
	GradientDescentNode* pTarget = NULL;
	CustomLinkedList<GradientDescentNode> listCorrectLocationAlgorithm;

	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if(g_RobotLocationsTable[i].ID == ui32OriginalID || g_RobotLocationsTable[i].ID == ui32RotationHopID)
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

				if(RobotLocationsTable_isContainRobot(g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID))
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

				if(RobotLocationsTable_isContainRobot(g_OneHopNeighborsTable[oneHopIndexsK].pNeighborsTable->ElementAt(j).ID))
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

bool Tri_tryToRotateLocationsTable(uint32_t ui32SelfID, uint32_t ui32RotationHopID, float fRotationHopXvalue, float fRotationHopYvalue, uint8_t* pui8OriLocsTableBufferPointer, int32_t ui32SizeOfOriLocsTable)
{
	DEBUG_PRINT("Entering Tri_tryToRotateLocationsTable........\n");

	RobotLocation item(0, 0);
	CustomLinkedList<RobotLocation> OriLocationsTable;
	int32_t i32Template;
	int pointer = 0;
	int i;
	for(i = 0; i < ui32SizeOfOriLocsTable; i++)
	{
		item.ID = construct4Byte(&pui8OriLocsTableBufferPointer[pointer]);
		pointer += 4;

		i32Template = construct4Byte(&pui8OriLocsTableBufferPointer[pointer]);
		item.vector.x = (float)(i32Template / 65536.0f);
		pointer += 4;

		i32Template = construct4Byte(&pui8OriLocsTableBufferPointer[pointer]);
		item.vector.y = (float)(i32Template / 65536.0f);
		pointer += 4;

		OriLocationsTable.add(item);
	}

	Vector2<float> vectRotationHop(fRotationHopXvalue, fRotationHopYvalue);
	Vector2<float> vectTransform;

	float fCorrectionAngle;
	bool bIsNeedMirroring;
	uint32_t ui32IdsRobotJ;
	if(Tri_tryToGetCommonNeighborID(&OriLocationsTable, &ui32IdsRobotJ, ui32SelfID, ui32RotationHopID))
	{
		if(Tri_calculateCorrectionAngle(ui32SelfID, ui32RotationHopID, ui32IdsRobotJ, &OriLocationsTable, &fCorrectionAngle, &bIsNeedMirroring))
		{
			RobotLocationsTable_rotate(fCorrectionAngle, bIsNeedMirroring);

			vectTransform = Tri_updateRobotVectorToWorldFrame(ui32SelfID, vectRotationHop, &OriLocationsTable);

			// Testing Only
			// RobotLocationsTable_linearTransform(vectTransform.x, vectTransform.y);
			// RobotLocationsTable_setVectorOfRobot(ui32RotationHopID, fRotationHopXvalue, fRotationHopYvalue);

			DEBUG_PRINT("........Returning TRUE from Tri_tryToRotateLocationsTable\n");

			return true;
		}
	}

	DEBUG_PRINT("........Returning FALSE from Tri_tryToRotateLocationsTable\n");

	return false;
}

Vector2<float> Tri_chooseTheBestVectorFromThreePoints(float fAlphaZ, uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ)
{
	int indexO, indexX, indexY;

	Vector2<float> vectorZNegSigned;
	Vector2<float> vectorZPosSigned;

	float errorNegSigned = 0;
	float errorPosSigned = 0;

	indexO = RobotLocationsTable_getIndexOfRobot(ui32RobotOsId);
	if(indexO < 0)	return NULL;

	indexX = RobotLocationsTable_getIndexOfRobot(ui32RobotXsId);
	if(indexX < 0)	return NULL;

	indexY = RobotLocationsTable_getIndexOfRobot(ui32RobotYsId);
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

bool Tri_tryToGetCommonNeighborID(CustomLinkedList<RobotLocation>* pOriLocsTable, uint32_t* pui32CommonID, uint32_t ui32SelfID, uint32_t ui32RotationHopID)
{
	uint32_t ui32TargetId;
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		ui32TargetId = g_RobotLocationsTable[i].ID;

		if(ui32TargetId == ui32SelfID || ui32TargetId == ui32RotationHopID)
			continue;

		int j;
		for(j = 0; j < pOriLocsTable->Count; j++)
		{
			if (ui32TargetId == pOriLocsTable->ElementAt(j).ID)
			{
				*pui32CommonID = ui32TargetId;
				return true;
			}
		}
	}
	*pui32CommonID = 0;
	return false;
}

bool Tri_calculateCorrectionAngle(uint32_t ui32SelfID, uint32_t ui32RotationHopID, uint32_t ui32IdsRobotJ, CustomLinkedList<RobotLocation>* pOriLocsTable, float *pfCorrectionAngle, bool* pbIsNeedMirroring)
{
	//
	// New Attempt: calculate correction angle
	//
//	float fAlphaK;
//	float fBetaI;
//	float fCorrectionAngleNoMirroring;
//	float fCorrectionAngleNeedMirroring;
//	Vector2<float> vectRobotJinOriLocs(0, 0);
//	Vector2<float> vectRobotJinLocs(0, 0);
//	Vector2<float> vectRotatedNoMirror(0, 0);
//	Vector2<float> vectRotatedMirroring(0, 0);
//	float errorInMirroring;
//	float errorNoMirror;
//
//	fAlphaK = Tri_getRobotAngleFromLocationsTable(ui32SelfID, pOriLocsTable);
//	fBetaI = Tri_getRobotAngleFromLocationsTable(ui32RotationHopID, &g_RobotLocationsTable);
//
//	fCorrectionAngleNoMirroring = fBetaI - fAlphaK + MATH_PI;
//	fCorrectionAngleNeedMirroring = fBetaI + fAlphaK;
////	fCorrectionAngleNoMirroring = MATH_PI_MUL_2 - (fBetaI - fAlphaK + MATH_PI);
////	fCorrectionAngleNeedMirroring = MATH_PI_MUL_2 - (fBetaI + fAlphaK);
//
////	fCorrectionAngleNoMirroring = atan2f(sinf(fCorrectionAngleNoMirroring), cosf(fCorrectionAngleNoMirroring));
////	fCorrectionAngleNeedMirroring = atan2f(sinf(fCorrectionAngleNeedMirroring), cosf(fCorrectionAngleNeedMirroring));
//
//	vectRobotJinLocs = Tri_getRobotVectorFromLocationsTable(ui32IdsRobotJ, &g_RobotLocationsTable);
//
//	vectRotatedNoMirror.x = vectRobotJinLocs.x * cosf(fCorrectionAngleNoMirroring) - vectRobotJinLocs.y * sinf(fCorrectionAngleNoMirroring);
//	vectRotatedNoMirror.y = vectRobotJinLocs.x * sinf(fCorrectionAngleNoMirroring) + vectRobotJinLocs.y * cosf(fCorrectionAngleNoMirroring);
//
//	vectRotatedMirroring.x = -(vectRobotJinLocs.x * cosf(fCorrectionAngleNeedMirroring) - vectRobotJinLocs.y * sinf(fCorrectionAngleNeedMirroring));
//	vectRotatedMirroring.y = vectRobotJinLocs.x * sinf(fCorrectionAngleNeedMirroring) + vectRobotJinLocs.y * cosf(fCorrectionAngleNeedMirroring);
//
//	vectRobotJinOriLocs = Tri_getRobotVectorFromLocationsTable(ui32IdsRobotJ, pOriLocsTable) - Tri_getRobotVectorFromLocationsTable(ui32SelfID, pOriLocsTable);
//
//	errorNoMirror = absFloatNumber(vectRotatedNoMirror.x - vectRobotJinOriLocs.x) + absFloatNumber(vectRotatedNoMirror.y - vectRobotJinOriLocs.y);
//	errorInMirroring = absFloatNumber(vectRotatedMirroring.x - vectRobotJinOriLocs.x) + absFloatNumber(vectRotatedMirroring.y - vectRobotJinOriLocs.y);
//
//	if(errorNoMirror < errorInMirroring)
//	{
//		*pfCorrectionAngle = fCorrectionAngleNoMirroring;
////		*pfCorrectionAngle = fCorrectionAngleNoMirroring + MATH_PI_MUL_2;
//		*pbIsNeedMirroring = false;
//	}
//	else
//	{
//		*pfCorrectionAngle = fCorrectionAngleNeedMirroring;
////		*pfCorrectionAngle = fCorrectionAngleNoMirroring + MATH_PI_MUL_2;
//		*pbIsNeedMirroring = true;
//	}

	//
	// Old Attempt: calculate correction angle
	//
	float fAlphaJ, fAlphaK, fAlphaJK;
	float fBetaJ, fBetaI, fBetaJI;

	fAlphaJ = Tri_getRobotAngleFromLocationsTable(ui32IdsRobotJ,pOriLocsTable);
	fAlphaK = Tri_getRobotAngleFromLocationsTable(ui32SelfID, pOriLocsTable);

	fAlphaJK = fAlphaJ - fAlphaK;
	fAlphaJK = (fAlphaJK < 0) ? (MATH_PI_MUL_2 + fAlphaJK) : (fAlphaJK);

	fBetaJ = Tri_getRobotAngleFromLocationsTable(ui32IdsRobotJ, &g_RobotLocationsTable);
	fBetaI = Tri_getRobotAngleFromLocationsTable(ui32RotationHopID, &g_RobotLocationsTable);

	fBetaJI = fBetaJ - fBetaI;
	fBetaJI = (fBetaJI < 0) ? (MATH_PI_MUL_2 + fBetaJI) : (fBetaJI);

	if ((fAlphaJK < MATH_PI && fBetaJI > MATH_PI)
			|| (fAlphaJK > MATH_PI && fBetaJI < MATH_PI))
	{
		*pbIsNeedMirroring = false;
		*pfCorrectionAngle = fBetaI - fAlphaK + MATH_PI;
	}
	else if ((fAlphaJK < MATH_PI && fBetaJI < MATH_PI)
			|| (fAlphaJK > MATH_PI && fBetaJI > MATH_PI))
	{
		*pbIsNeedMirroring = true;
		*pfCorrectionAngle = fBetaI + fAlphaK;
	}
	else
	{
		return false;
	}

	return true;
}

float Tri_getRobotAngleFromLocationsTable(uint32_t ui32RobotId, CustomLinkedList<RobotLocation>* pLocsTable)
{
	float angle;
	int i;
	for(i = 0; i < pLocsTable->Count; i++)
	{
		if(pLocsTable->ElementAt(i).ID == ui32RobotId)
		{
			if(pLocsTable->ElementAt(i).vector.y == 0 &&
					pLocsTable->ElementAt(i).vector.x == 0)
				return 0;

			angle = atan2f(pLocsTable->ElementAt(i).vector.y, pLocsTable->ElementAt(i).vector.x);

			if (angle > 0)
				return angle;
			else
				return (angle + MATH_PI_MUL_2);
		}
	}

	return 0;
}

Vector2<float> Tri_getRobotVectorFromLocationsTable(uint32_t ui32RobotId, CustomLinkedList<RobotLocation>* pLocsTable)
{
	Vector2<float> vector(0, 0);
	int i;
	for(i = 0; i < pLocsTable->Count; i++)
	{
		if(pLocsTable->ElementAt(i).ID == ui32RobotId)
		{
			return pLocsTable->ElementAt(i).vector;
		}
	}

	return vector;
}

Vector2<float> Tri_updateRobotVectorToWorldFrame(uint32_t ui32SelfId, Vector2<float> vectRotationHop, CustomLinkedList<RobotLocation>* pOriLocsTable)
{
	Vector2<float> vectorInWorlFrame(0, 0);
	int i;
	for(i = 0; i < pOriLocsTable->Count; i++)
	{
		if(pOriLocsTable->ElementAt(i).ID == ui32SelfId)
		{
			vectorInWorlFrame = vectRotationHop + pOriLocsTable->ElementAt(i).vector;

			setRobotIdentityVector(vectorInWorlFrame.x, vectorInWorlFrame.y);

			return vectorInWorlFrame;
		}
	}
	return vectorInWorlFrame;
}
