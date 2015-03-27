/*
 * Trilateration.c
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */
#include "libalgorithm/inc/Trilateration.h"

#include "libprotocol/inc/network.h"
#include "librobot/inc/robot_analog.h"

#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotMeas.h"
#include "libstorage/inc/robot_data.h"

#include "data_manipulation.h"
#include <math.h>

#include "libcustom/inc/custom_uart_debug.h"

extern CustomLinkedList<RobotMeas> g_NeighborsTable;
extern CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
extern CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void initDataOfRobot1(void)
{
	DEBUG_PRINT("init Data of Robot[1].........\n");

	Network_setSelfAddress(0xbead01);

	// (1) Fill Neighbors Table
	NeighborsTable_add(0xBEAD05, (uint16_t)(20.5352f * 256));
	NeighborsTable_add(0xBEAD08, (uint16_t)(19.8438f * 256));
	NeighborsTable_add(0xBEAD03, (uint16_t)(20.1797f * 256));
	NeighborsTable_add(0xBEAD04, (uint16_t)(20.255f * 256));

	// (2) Fill OneHopNeighbors Table
	RobotMeas robot(0, 0);
	CustomLinkedList<RobotMeas> NeighborsTable;
	OneHopMeas oneHopMeas;

	oneHopMeas.firstHopID = 0xBEAD04;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(21.2188f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(20.25f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(34.457f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(34.4922f * 256);	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD03;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(22.2539f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(20.1797f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(34.457f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(20.3281f * 256);	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD08;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD05; robot.Distance = (uint16_t)(35.293f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(19.8438f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(20.3281f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(34.4922f * 256);	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	oneHopMeas.firstHopID = 0xBEAD05;
	NeighborsTable.clearAll();
	robot.ID = 0xBEAD08; robot.Distance = (uint16_t)(35.293f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD04; robot.Distance = (uint16_t)(21.2188f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD03; robot.Distance = (uint16_t)(22.2539f * 256);	NeighborsTable.add(robot);
	robot.ID = 0xBEAD01; robot.Distance = (uint16_t)(20.5352f * 256);	NeighborsTable.add(robot);
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);

	g_RobotLocationsTable.clearAll();

	DEBUG_PRINT("------------initialize Data OK\n");
	DEBUG_PRINT("------------------------------\n");
}
void initData(uint8_t* pui8MessageData) // Test Only
{
//	Locations Table of Robot [0x00BEAD08]:
//	Robot:0xBEAD08 (0; 0)
//	Robot:0xBEAD03 (19.7641; 1.50394)
//	Robot:0xBEAD04 (19.3752; 20.8548)
//	Robot:0xBEAD01 (30.8832; 6.68922)
//	Robot:0xBEAD05 (10.7517; -11.0418)
//	Robot:0xBEAD06 (8.81143; 10.7117)

	RobotLocation item(0,0);
	item.ID = 0xBEAD08; item.vector.x = 0; 			item.vector.y = 0;			g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD03; item.vector.x = 19.7641f; 	item.vector.y = 1.50394f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD04; item.vector.x = 19.3752f; 	item.vector.y = 20.8548f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD01; item.vector.x = 30.8832f; 	item.vector.y = 6.68922f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD05; item.vector.x = 10.7517f; 	item.vector.y = -11.0418f;	g_RobotLocationsTable.add(item);
	item.ID = 0xBEAD06; item.vector.x = 8.81143f; 	item.vector.y = 10.7117f;	g_RobotLocationsTable.add(item);

//	Locations Table of Robot [0x00BEAD01]:
//	Robot:0xBEAD01 (0; 0)
//	Robot:0xBEAD08 (31.3622; -0.0520172)
//	Robot:0xBEAD03 (11.1132; 4.22351)
//	Robot:0xBEAD04 (8.11284; -15.9175)
//	Robot:0xBEAD05 (23.2277; 13.6088)
//	Robot:0xBEAD06 (19.8988; -7.58269)

	CustomLinkedList<RobotLocation> OriLocationsTable;
	item.ID = 0xBEAD01; item.vector.x = 0; 				item.vector.y = 0;				OriLocationsTable.add(item);
	item.ID = 0xBEAD08; item.vector.x = 31.3622f; 		item.vector.y = -0.0520172f;	OriLocationsTable.add(item);
	item.ID = 0xBEAD03; item.vector.x = 11.1132f; 		item.vector.y = 4.22351f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD04; item.vector.x = 8.11284f; 		item.vector.y = -15.9175f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD05; item.vector.x = 23.2277f; 		item.vector.y = 13.6088f;		OriLocationsTable.add(item);
	item.ID = 0xBEAD06; item.vector.x = 19.8988f; 		item.vector.y = -7.58269f;		OriLocationsTable.add(item);

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

//	Tri_findAllPointsFromFirstTwoPoints(ui32RobotOsId, ui32RobotXsId, ui32RobotYsId, ui16EdgeOX, ui16EdgeOY, ui16EdgeXY);
//	if(g_RobotLocationsTable.Count < g_NeighborsTable.Count)
//		Tri_findAllPossibleRemainPoints();

	Tri_tryToCalculateTheRemainPoints(ui32RobotOsId);

	RobotLocationsTable_selfCorrectByGradientDescent(ui32RobotOsId, 0);

	DEBUG_PRINT("........Returning TRUE from Tri_tryToCalculateRobotLocationsTable\n");
	return true;
}

//============= New Attempt: Edges Only =============================================================
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

	// New Attempt: Edges only - faster 4ps
	float fEdgeOX = ui16EdgeOX / 256.0f;
	float fEdgeOY = ui16EdgeOY / 256.0f;
	float fEdgeXY = ui16EdgeXY / 256.0f;
	DEBUG_PRINTS3("Valid Triangle: OX = %2.4f, OY = %2.4f, XY = %2.4f\n", fEdgeOX, fEdgeOY, fEdgeXY);

	Vector2<float> pointX(fEdgeOX, 0);
	RobotLocationsTable_add(ui32RobotXsId, pointX.x, pointX.y);
	DEBUG_PRINTS3("Add Robot X[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotXsId, pointX.x, pointX.y);

	Vector2<float> pointY1;
	pointY1.y = findAltitudeOfTriangle(fEdgeOX, fEdgeOY, fEdgeXY);
	pointY1.x = sqrtf(fEdgeOY*fEdgeOY - pointY1.y*pointY1.y);

	Vector2<float> pointY2(-pointY1.x, pointY1.y);
	Vector2<float> vectorXY1 = pointX - pointY1;
	Vector2<float> vectorXY2 = pointX - pointY2;

	float errorXY1 = absFloatNumber(fEdgeXY - vectorXY1.getMagnitude());
	float errorXY2 = absFloatNumber(fEdgeXY - vectorXY2.getMagnitude());
	if(errorXY1 < errorXY2)
	{
		RobotLocationsTable_add(ui32RobotYsId, pointY1.x, pointY1.y);
		DEBUG_PRINTS3("Add Robot Y[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotYsId, pointY1.x, pointY1.y);
	}
	else
	{
		RobotLocationsTable_add(ui32RobotYsId, pointY2.x, pointY2.y);
		DEBUG_PRINTS3("Add Robot Y[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotYsId, pointY2.x, pointY2.y);
	}

	// Old Attempt: Trigonometric with Angles
//	Vector2<float> vectorOX(0, 0);
//	Vector2<float> vectorOY(0, 0);
//
//	float fEdgeOX = ui16EdgeOX / 256.0f;
//	float fEdgeOY = ui16EdgeOY / 256.0f;
//	float fEdgeXY = ui16EdgeXY / 256.0f;
//	DEBUG_PRINTS3("Valid Triangle: OX = %2.4f, OY = %2.4f, XY = %2.4f\n", fEdgeOX, fEdgeOY, fEdgeXY);
//
//	float fCosAlphaYOX = findCosAngleUseCosineRuleForTriangle(fEdgeOX, fEdgeOY, fEdgeXY);
//	float fSinAlphaYOX = sinf(acosf(fCosAlphaYOX));
//
//	vectorOX.x = fEdgeOX;
//	vectorOX.y = 0;
//	RobotLocationsTable_add(ui32RobotXsId, vectorOX.x, vectorOX.y);
//	DEBUG_PRINTS3("Add Robot X[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotXsId, vectorOX.x, vectorOX.y);
//
//	vectorOY.x = fEdgeOY * fCosAlphaYOX;
//	vectorOY.y = fEdgeOY * fSinAlphaYOX;
//	RobotLocationsTable_add(ui32RobotYsId, vectorOY.x, vectorOY.y);
//	DEBUG_PRINTS3("Add Robot Y[0x%06x] (%2.4f; %2.4f) to Locations Table\n", ui32RobotYsId, vectorOY.x, vectorOY.y);

	DEBUG_PRINT("........Returning from Tri_calculateXYLocations\n");
}

void Tri_tryToCalculateTheRemainPoints(uint32_t ui32RobotIsId)
{
	/* Pseudo code
	   I'm robot I
	   LOOP through neighbors table to pick a Robot J
	   		IF Robot J already have locations OR
	   			I don't have Neighbors Table of Robot J OR
	   				Robot J's Neighbors Table not contain I
	   		THEN
				continue pick another neighbor
			END

			Try to pick 2 two located neighbors P, Q in Robot J's Table
			calculate robot J's location
			add to locations table
			reset counter
	 */

	uint32_t ui32RobotJsId;

	bool bIsFoundP;
	bool bIsFoundQ;

	uint16_t ui16EdgeIJ;
	uint16_t ui16EdgePJ;
	uint16_t ui16EdgeQJ;

	Vector2<float> pointI(0, 0);
	Vector2<float> pointP;
	Vector2<float> pointQ;
	Vector2<float> pointJ;

	int oneHopIndexsJ;
	RobotLocation locOfRobotJ(0, NULL);

	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		ui32RobotJsId = g_NeighborsTable[i].ID;
		DEBUG_PRINTS("Pick Robot J from O neighbors = [0x%06x]\n", ui32RobotJsId);

		if(RobotLocationsTable_isContainRobot(ui32RobotJsId))
			continue;

		oneHopIndexsJ = OneHopNeighborsTable_getIndexOfRobot(ui32RobotJsId);
		if (oneHopIndexsJ < 0)	// This condition never wrong due to State 1 has cover this
			continue;

		if(NeighborsTable_getDistanceOfRobot(ui32RobotJsId, &ui16EdgeIJ) == false)			// get IJ
			continue;

		bIsFoundP = false;
		bIsFoundQ = false;

		int j = 0;
		do	// Try to find robot P
		{
			if (j >= g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->Count)
				break;

			if(g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).ID == ui32RobotIsId)
			{
				j++;
				continue;
			}

			if(RobotLocationsTable_getVectorOfRobot(g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).ID, &(pointP.x), &(pointP.y)))
			{
				ui16EdgePJ = g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).Distance;
				bIsFoundP = true;
			}
			j++;
		} while (!bIsFoundP);

		do // Try to find robot Q
		{
			if (j >= g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->Count)
				break;

			if(g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).ID == ui32RobotIsId)
			{
				j++;
				continue;
			}

			if(RobotLocationsTable_getVectorOfRobot(g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).ID, &(pointQ.x), &(pointQ.y)))
			{
				ui16EdgeQJ = g_OneHopNeighborsTable[oneHopIndexsJ].pNeighborsTable->ElementAt(j).Distance;
				bIsFoundQ = true;
			}
			j++;
		} while (!bIsFoundQ);

	   	if (!(bIsFoundP && bIsFoundQ))
	   		continue;

		// (1) calculate location of robot J base on ui16EdgeIJ, ui16EdgePJ, ui16EdgeQJ, pointI(0, 0), pointP and pointQ
	    if(!Tri_calculateRobotLocationBaseOnThreeLocatedPoint(ui16EdgeIJ / 256.0, ui16EdgePJ / 256.0, ui16EdgeQJ / 256.0, pointI, pointP, pointQ, &pointJ))
	    	continue;

		// (2) add to Locations table
	   	locOfRobotJ.ID = ui32RobotJsId;
		locOfRobotJ.vector = pointJ;

		g_RobotLocationsTable.add(locOfRobotJ);
		DEBUG_PRINTS3("Add Robot J[0x%06x] (%2.4f; %2.4f) to Locations Table\n", locOfRobotJ.ID, locOfRobotJ.vector.x, locOfRobotJ.vector.y);

		// (3) reset loop counter
		i = 0;
	}
}
bool Tri_calculateRobotLocationBaseOnThreeLocatedPoint(float fEdgeIJ, float fEdgePJ, float fEdgeQJ, Vector2<float> pointI, Vector2<float> pointP, Vector2<float> pointQ, Vector2<float>* pPointJ)
{
	// (1) calculate Slope-Intercept Form of the straight-line cross I and P
	LineEquation_t lineIP;
	Vector2<float> vectorPI = pointP - pointI;
	lineIP.a = -vectorPI.y;
	lineIP.b = vectorPI.x;
	lineIP.c = vectorPI.y*pointI.x - vectorPI.x*pointI.y;

	// (2) find the H point
	float fEdgeIP = vectorPI.getMagnitude();
	float fHeightJH = findAltitudeOfTriangle(fEdgeIP, fEdgePJ, fEdgeIJ);

	Vector2<float> pointH;
	if(fHeightJH == fEdgeIJ)
	{
		pointH = pointI;
	}
	else if (fHeightJH == fEdgePJ)
	{
		pointH = pointP;
	}
	else
	{
		Vector2<float> pointH1;
		Vector2<float> pointH2;

		float fEdgeIH = sqrtf(fEdgeIJ * fEdgeIJ - fHeightJH * fHeightJH);
		CircleEquation_t circleI_IH;
		circleI_IH.x0 = pointI.x;
		circleI_IH.y0 = pointI.y;
		circleI_IH.R = fEdgeIH;

		if(!Tri_calculateTwoCrossPointBetweenLineAndCircle(lineIP, circleI_IH, &pointH1, &pointH2))
			return false;

		// (2.2) choose H
		Vector2<float> vectorPH1 = pointP - pointH1;
		Vector2<float> vectorPH2 = pointP - pointH2;

		float fEdgePH = sqrtf(fEdgePJ * fEdgePJ - fHeightJH * fHeightJH);
		float errorPH1 = absFloatNumber(fEdgePH - vectorPH1.getMagnitude());
		float errorPH2 = absFloatNumber(fEdgePH - vectorPH2.getMagnitude());

		if(errorPH1 < errorPH2)
			pointH = pointH1;
		else
			pointH = pointH2;
	}

	// (3) calculate the point J
	Vector2<float> pointJ1;
	Vector2<float> pointJ2;

	LineEquation_t lineJH;
	lineJH.a = lineIP.b;
	lineJH.b = -lineIP.a;
	lineJH.c = lineIP.a*pointH.y - lineIP.b*pointH.x;

	CircleEquation_t circleI_IJ;
	circleI_IJ.x0 = pointI.x;
	circleI_IJ.y0 = pointI.y;
	circleI_IJ.R = fEdgeIJ;

	if(!Tri_calculateTwoCrossPointBetweenLineAndCircle(lineJH, circleI_IJ, &pointJ1, &pointJ2))
		return false;

	// (4) choose the smaller error compare with QJ to determine the correct J point
	Vector2<float> vectorQJ1 = pointQ - pointJ1;
	Vector2<float> vectorQJ2 = pointQ - pointJ2;
	float errorQJ1 = absFloatNumber(fEdgeQJ - vectorQJ1.getMagnitude());
	float errorQJ2 = absFloatNumber(fEdgeQJ - vectorQJ2.getMagnitude());

	if(errorQJ1 < errorQJ2)
		*pPointJ = pointJ1;
	else
		*pPointJ = pointJ2;

	return true;
}
bool Tri_calculateTwoCrossPointBetweenLineAndCircle(LineEquation_t line, CircleEquation_t circle, Vector2<float>* pP1, Vector2<float>* pP2)
{
	if(line.b == 0) // Case 1: b == 0
	{
		float k = -line.c / line.a;
		float l = k - circle.x0;
		if(!solveQuadraticEquation(
				/*A =*/1,
				/*B =*/-circle.y0,
				/*C =*/circle.y0*circle.y0 + l*l - circle.R*circle.R,
				&pP1->y, &pP2->y))
			return false;

		pP1->x = pP2->x = k;
	}
	else	// Case 2: b != 0
	{
		float l = line.a / line.b;
		float j = line.c / line.b;
		float k = j + circle.y0;
		if(!solveQuadraticEquation(
				/*A =*/1 + l*l,
				/*B =*/l*k - circle.x0,
				/*C =*/circle.x0*circle.x0 + k*k - circle.R*circle.R,
				&pP1->x, &pP2->x))
			return false;

		pP1->y = -l*pP1->x - j;
		pP2->y = -l*pP2->x - j;
	}
	return true;
}

//============= Old Attempt: Trigonometric with Angles ==============================================
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

		g_RobotLocationsTable.add(locOfRobotZ);
		DEBUG_PRINTS3("Add Robot Z[0x%06x] (%2.4f; %2.4f) to Locations Table\n", locOfRobotZ.ID, locOfRobotZ.vector.x, locOfRobotZ.vector.y);
	}

	DEBUG_PRINT("........Returning from Tri_findAllPointsFromFirstTwoPoints\n");
}

void Tri_findAllPossibleRemainPoints(void)
{
	/* Pseudo code - HAVE NOT TESTED YET
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

		locOfRobotK.vector = vectorK;

		g_RobotLocationsTable.add(locOfRobotK);
		DEBUG_PRINTS3("Add Robot Z[0x%06x] (%2.4f; %2.4f) to Locations Table\n", locOfRobotK.ID, locOfRobotK.vector.x, locOfRobotK.vector.y);

		// (3) reset loop counter
		i = 0;
	}
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

//============= Rotate Coordinates ==================================================================
bool Tri_tryToRotateLocationsTable(RobotIdentity_t* pRobotIdentity, uint8_t* pui8OriLocsTableBufferPointer, int32_t ui32SizeOfOriLocsTable)
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

	Vector2<float> vectTransform;

	float fCorrectionAngle;
	bool bIsNeedMirroring;
	uint32_t ui32IdsRobotJ;
	if(Tri_tryToGetCommonNeighborID(pRobotIdentity, &OriLocationsTable, &ui32IdsRobotJ))
	{
		if(Tri_calculateCorrectionAngle(pRobotIdentity, ui32IdsRobotJ, &OriLocationsTable, &fCorrectionAngle, &bIsNeedMirroring))
		{
			RobotLocationsTable_rotate(fCorrectionAngle, bIsNeedMirroring);

			vectTransform = Tri_updateRobotVectorToWorldFrame(pRobotIdentity, &OriLocationsTable);

			DEBUG_PRINT("........Returning TRUE from Tri_tryToRotateLocationsTable\n");

			return true;
		}
	}

	DEBUG_PRINT("........Returning FALSE from Tri_tryToRotateLocationsTable\n");

	return false;
}

bool Tri_tryToGetCommonNeighborID(RobotIdentity_t* pRobotIdentity, CustomLinkedList<RobotLocation>* pOriLocsTable, uint32_t* pui32CommonID)
{
	uint32_t ui32TargetId;
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		ui32TargetId = g_RobotLocationsTable[i].ID;

		if(ui32TargetId == pRobotIdentity->Self_ID || ui32TargetId == pRobotIdentity->RotationHop_ID)
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

bool Tri_calculateCorrectionAngle(RobotIdentity_t* pRobotIdentity, uint32_t ui32IdsCommonNeighbor, CustomLinkedList<RobotLocation>* pLocsTableOfRotationHop, float *pfCorrectionAngle, bool* pbIsNeedFlipX)
{
	//
	// New Attempt: calculate correction angle
	//

//	// (1) calculate correction angle in two cases.
//	float fAlphaK = Tri_getRobotAngleFromLocationsTable(pRobotIdentity->Self_ID, pRobotIdentity->RotationHop_ID, pLocsTableOfRotationHop);
//	float fBetaI = Tri_getRobotAngleFromLocationsTable(pRobotIdentity->RotationHop_ID, pRobotIdentity->Self_ID, &g_RobotLocationsTable);
//
//	float fCorrectionAngleInCaseRotateOnly = MATH_PI_MUL_2 - (fBetaI - fAlphaK + MATH_PI);
//	float fCorrectionAngleInCaseRotateAndFlipX = MATH_PI_MUL_2 - (fBetaI + fAlphaK);
//
//	// (2) rotate x axis of vector IJ of locs table in two cases. One case will match IJ vector in oriLocs table
//	Vector2<float> pointCommonNeighborInLocalFrame = Tri_getRobotVectorFromLocationsTable(ui32IdsCommonNeighbor, &g_RobotLocationsTable);
//	Vector2<float> vectorRotatedOnlyOfCommonNeighborInLocalFrame;
//	Vector2<float> vectorRotatedAndMirrorOfCommonNeighborInLocalFrame;
//
//	vectorRotatedOnlyOfCommonNeighborInLocalFrame.x = pointCommonNeighborInLocalFrame.x * cosf(fCorrectionAngleInCaseRotateOnly) - pointCommonNeighborInLocalFrame.y * sinf(fCorrectionAngleInCaseRotateOnly);
//	vectorRotatedOnlyOfCommonNeighborInLocalFrame.y = pointCommonNeighborInLocalFrame.x * sinf(fCorrectionAngleInCaseRotateOnly) + pointCommonNeighborInLocalFrame.y * cosf(fCorrectionAngleInCaseRotateOnly);
//
//	vectorRotatedAndMirrorOfCommonNeighborInLocalFrame.x = -(pointCommonNeighborInLocalFrame.x * cosf(fCorrectionAngleInCaseRotateAndFlipX) - pointCommonNeighborInLocalFrame.y * sinf(fCorrectionAngleInCaseRotateAndFlipX));
//	vectorRotatedAndMirrorOfCommonNeighborInLocalFrame.y = pointCommonNeighborInLocalFrame.x * sinf(fCorrectionAngleInCaseRotateAndFlipX) + pointCommonNeighborInLocalFrame.y * cosf(fCorrectionAngleInCaseRotateAndFlipX);
//
//	// (3) map common neighbor X AXIS in local's frame to rotation hop's frame
//	Vector2<float> pointRotationHopInRotaionHopLocs = Tri_getRobotVectorFromLocationsTable(pRobotIdentity->RotationHop_ID, pLocsTableOfRotationHop);
//	Vector2<float> pointSelfInRotaionHopLocs = Tri_getRobotVectorFromLocationsTable(pRobotIdentity->Self_ID, pLocsTableOfRotationHop);
//	Vector2<float> pointSelfInRotaionHopFrame = pointSelfInRotaionHopLocs - pointRotationHopInRotaionHopLocs;
//
//	Vector2<float> vectorRotatedOnlyOfCommonNeighborInRotationHopFrame = vectorRotatedOnlyOfCommonNeighborInLocalFrame + pointSelfInRotaionHopFrame.x;
//	Vector2<float> vectorRotatedAndMirrorOfCommonNeighborInRotationHopFrame = vectorRotatedAndMirrorOfCommonNeighborInLocalFrame + pointSelfInRotaionHopFrame.x;
//
//	// (4) compare two results with the x axis of common neighbor in rotation hop's frame
//	Vector2<float> pointCommonNeighborInRotaionHopLocs = Tri_getRobotVectorFromLocationsTable(ui32IdsCommonNeighbor, pLocsTableOfRotationHop);
//	Vector2<float> pointCommonNeighborInRotaionHopFrame = pointCommonNeighborInRotaionHopLocs - pointRotationHopInRotaionHopLocs;
//	float errorInCaseRotateOnly = absFloatNumber(vectorRotatedOnlyOfCommonNeighborInRotationHopFrame.x - pointCommonNeighborInRotaionHopFrame.x) +
//									absFloatNumber(vectorRotatedOnlyOfCommonNeighborInRotationHopFrame.y - pointCommonNeighborInRotaionHopFrame.y);
//	float errorInCaseRotateAndFlipX = absFloatNumber(vectorRotatedAndMirrorOfCommonNeighborInRotationHopFrame.x - pointCommonNeighborInRotaionHopFrame.x) +
//										absFloatNumber(vectorRotatedAndMirrorOfCommonNeighborInRotationHopFrame.y - pointCommonNeighborInRotaionHopFrame.y);
//
//	if(errorInCaseRotateOnly < errorInCaseRotateAndFlipX)
//	{
//		*pfCorrectionAngle = fCorrectionAngleInCaseRotateOnly;
//		*pbIsNeedFlipX = false;
//	}
//	else
//	{
//		*pfCorrectionAngle = fCorrectionAngleInCaseRotateAndFlipX;
//		*pbIsNeedFlipX = true;
//	}

	//
	// Old Attempt: calculate correction angle
	//
	float fAlphaJ, fAlphaK, fAlphaJK;
	float fBetaJ, fBetaI, fBetaJI;

	fAlphaJ = Tri_getRobotAngleFromLocationsTable(ui32IdsCommonNeighbor, pRobotIdentity->RotationHop_ID, pLocsTableOfRotationHop);
	fAlphaK = Tri_getRobotAngleFromLocationsTable(pRobotIdentity->Self_ID, pRobotIdentity->RotationHop_ID, pLocsTableOfRotationHop);

	fAlphaJK = fAlphaJ - fAlphaK;
	fAlphaJK = atan2f(sinf(fAlphaJK), cosf(fAlphaJK));
	fAlphaJK = (fAlphaJK < 0) ? (MATH_PI_MUL_2 + fAlphaJK) : (fAlphaJK);

	fBetaJ = Tri_getRobotAngleFromLocationsTable(ui32IdsCommonNeighbor, pRobotIdentity->Self_ID, &g_RobotLocationsTable);
	fBetaI = Tri_getRobotAngleFromLocationsTable(pRobotIdentity->RotationHop_ID, pRobotIdentity->Self_ID, &g_RobotLocationsTable);

	fBetaJI = fBetaJ - fBetaI;
	fBetaJI = atan2f(sinf(fBetaJI), cosf(fBetaJI));
	fBetaJI = (fBetaJI < 0) ? (MATH_PI_MUL_2 + fBetaJI) : (fBetaJI);

	if ((fAlphaJK < MATH_PI && fBetaJI > MATH_PI)
			|| (fAlphaJK > MATH_PI && fBetaJI < MATH_PI))
	{
		*pbIsNeedFlipX = false;
		*pfCorrectionAngle = MATH_PI_MUL_2 - (fBetaI - fAlphaK + MATH_PI);
	}
	else if ((fAlphaJK < MATH_PI && fBetaJI < MATH_PI)
			|| (fAlphaJK > MATH_PI && fBetaJI > MATH_PI))
	{
		*pbIsNeedFlipX = true;
		*pfCorrectionAngle = MATH_PI_MUL_2 - (fBetaI + fAlphaK);
	}
	else
	{
		return false;
	}

	return true;
}

float Tri_getRobotAngleFromLocationsTable(uint32_t ui32EndPointRobotId, uint32_t ui32StartPointRobotId, CustomLinkedList<RobotLocation>* pLocsTable)
{
	int i, startPoint, endPoint;

	// (1) find start point
	for(i = 0; i < pLocsTable->Count; i++)
	{
		if(pLocsTable->ElementAt(i).ID == ui32StartPointRobotId)
			startPoint = i;
	}
	if(startPoint == pLocsTable->Count)
		return 0;

	// (2) find end point
	for(i = 0; i < pLocsTable->Count; i++)
	{
		if(pLocsTable->ElementAt(i).ID == ui32EndPointRobotId)
			endPoint = i;
	}
	if(endPoint == pLocsTable->Count)
		return 0;

	// (3) calculate the angle
	float angle = atan2f(pLocsTable->ElementAt(endPoint).vector.y - pLocsTable->ElementAt(startPoint).vector.y,
			pLocsTable->ElementAt(endPoint).vector.x - pLocsTable->ElementAt(startPoint).vector.x);

	if (angle > 0)
		return angle;
	else
		return (angle + MATH_PI_MUL_2);
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

Vector2<float> Tri_updateRobotVectorToWorldFrame(RobotIdentity_t* pRobotIdentity, CustomLinkedList<RobotLocation>* pOriLocsTable)
{
	Vector2<float> vectRotationHop(pRobotIdentity->RotationHop_x, pRobotIdentity->RotationHop_y);
	Vector2<float> vectorInWorlFrame(0, 0);
	int i;
	for(i = 0; i < pOriLocsTable->Count; i++)
	{
		if(pOriLocsTable->ElementAt(i).ID == pRobotIdentity->Self_ID)
		{
			vectorInWorlFrame = vectRotationHop + pOriLocsTable->ElementAt(i).vector;

			pRobotIdentity->x = vectorInWorlFrame.x;
			pRobotIdentity->y = vectorInWorlFrame.y;

			return vectorInWorlFrame;
		}
	}
	return vectorInWorlFrame;
}
