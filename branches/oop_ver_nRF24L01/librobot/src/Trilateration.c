/*
 * Trilateration.c
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#include "librobot/inc/Trilateration.h"

extern uint32_t g_ui32RobotID;
extern uint8_t g_ui8NeighborsCounter;

extern location_t locs[];

extern uint32_t g_ui8LocsCounter;

void Tri_clearLocs(location_t locsTable[], uint8_t *length)
{
	int i;

	for (i = 0; i < LOCATIONS_TABLE_LENGTH; i++)
	{
//		locs[i].ID = 0;
//		locs[i].vector.x = 0;
//		locs[i].vector.y = 0;
		locsTable[i].ID = 0;
		locsTable[i].vector.x = 0;
		locsTable[i].vector.y = 0;
	}

	// g_ui8LocsCounter = 0;
	*length = 0;
}

void Tri_addLocation(uint32_t ID, float x, float y)
{
	vector2_t tempVec;
	if (g_ui8LocsCounter < LOCATIONS_TABLE_LENGTH)
	{
		tempVec.x = x;
		tempVec.y = y;

		locs[g_ui8LocsCounter].ID = ID;
		locs[g_ui8LocsCounter].vector = tempVec;

		g_ui8LocsCounter++;
	}
	else
	{
		g_ui8LocsCounter = LOCATIONS_TABLE_LENGTH;
	}
}

void Tri_findLocs(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable)
{
	robotMeas_t robotX;
	robotMeas_t robotY;

	float edgeOX;
	float edgeOY;
	float edgeXY;

	float cosAlphaYOX;
	float sinAlphaYOX;

	edgeXY = Tri_tryToFindTwoXYNeighbors(pOneHopTable, &robotX, &robotY,
			pNeighborsTable) / 256.0f;

	if (edgeXY != 0)
	{
		edgeOX = robotX.distance / 256.0f;
		edgeOY = robotY.distance / 256.0f;

		cosAlphaYOX = cosinesRuleForTriangles(edgeOX, edgeOY, edgeXY);
		sinAlphaYOX = calSin(calACos(cosAlphaYOX));

		Tri_addLocation(robotX.ID, edgeOX, 0);
		Tri_addLocation(robotY.ID, edgeOY * cosAlphaYOX, edgeOY * sinAlphaYOX);

		Tri_findAllPointsFromFirstTwoPoints(pNeighborsTable, pOneHopTable,
				robotX, robotY, edgeXY);

		if (g_ui8LocsCounter >= (g_ui8NeighborsCounter + 1))
		{
			return;
		}

		Tri_findAllPossibleRemainPoints(pNeighborsTable, pOneHopTable);
	}
}

void Tri_findAllPointsFromFirstTwoPoints(robotMeas_t* pNeighborsTable,
		oneHopMeas_t* pOneHopTable, robotMeas_t robotX, robotMeas_t robotY,
		float edgeXY)
{
	int neighborPointer = 0;
	int ZneighborPointer = 0;

	uint8_t onehopTablePosition = 0;

	uint16_t distanceXZ = 0;
	uint16_t distanceYZ = 0;
	uint16_t distanceOZ = 0;

	float edgeOZ = 0;
	float edgeYZ = 0;
	float edgeXZ = 0;
	float edgeOX = 0;
	float edgeOY = 0;

	uint8_t successDistanceCounter;

	vector2_t newPos;

	// duyet cac neighbor Z trong Bang 1
	for (neighborPointer = 0; neighborPointer < g_ui8NeighborsCounter;
			neighborPointer++)
	{
		// neu Z da co toa do thi skip, nguoc lai lay khoan cach O->Z
		if (Tri_isLocationsTableContain(pNeighborsTable[neighborPointer].ID))
			continue;
		distanceOZ = pNeighborsTable[neighborPointer].distance;

		// neu khong tim thay bang neighbors cua Z thi skip, nguoc lai luu vi tri onehopTablePosition
		for (onehopTablePosition = 0;
				onehopTablePosition < ONEHOP_NEIGHBOR_TABLE_LENGTH;
				onehopTablePosition++)
		{
			if (pOneHopTable[onehopTablePosition].firstHopID
					== pNeighborsTable[neighborPointer].ID)
				break;
		}
		if (onehopTablePosition == ONEHOP_NEIGHBOR_TABLE_LENGTH)
			continue;

		// neu Z khong phai neighbor cua X thi skip, nguoc lai lay khoan cach X->Z
		distanceXZ = 0;
		distanceXZ = Tri_tryToGetDistance(pOneHopTable, robotX.ID,
				pNeighborsTable[neighborPointer].ID);
		if (distanceXZ == 0)
			continue;

		// neu Z khong phai neighbor cua Y thi skip, nguoc lai lay khoan cach Y->Z
		distanceYZ = 0;
		distanceYZ = Tri_tryToGetDistance(pOneHopTable, robotY.ID,
				pNeighborsTable[neighborPointer].ID);
		if (distanceXZ == 0)
			continue;

		// neu X hoac Y khong phai neighbor cua Z thi skip, nguoc lai lay khoan cach Z->X, Z->Y va Z->O
		successDistanceCounter = 0;
		for (ZneighborPointer = 0; ZneighborPointer < NEIGHBOR_TABLE_LENGTH;
				ZneighborPointer++)
		{
			if (pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].ID
					== robotX.ID)
			{
				distanceXZ =
						(distanceXZ
								+ pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].distance)
								/ 2;
				successDistanceCounter++;
			}
			else if (pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].ID
					== robotY.ID)
			{
				distanceYZ =
						(distanceYZ
								+ pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].distance)
								/ 2;
				successDistanceCounter++;
			}
			else if (pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].ID
					== g_ui32RobotID)
			{
				distanceOZ =
						(distanceOZ
								+ pOneHopTable[onehopTablePosition].neighbors[ZneighborPointer].distance)
								/ 2;
				successDistanceCounter++;
			}
			if (successDistanceCounter == 3)
				break;
		}
		if (successDistanceCounter != 3)
			continue;

		edgeOY = robotY.distance / 256.0f;
		edgeOX = robotX.distance / 256.0f;
		edgeOZ = distanceOZ / 256.0f;
		edgeYZ = distanceYZ / 256.0f;
		edgeXZ = distanceXZ / 256.0f;

		// tinh toa do cua Z, edgeXY, offset 0
		newPos = Tri_trilaterateFromSixEdgeAndAngleOffset(edgeOY, edgeOX,
				edgeOZ, edgeYZ, edgeXZ, edgeXY, 0);

		if (newPos.x == 0 && newPos.y == 0)
			continue;

		// luu lai vao bang toa do
		Tri_addLocation(pNeighborsTable[neighborPointer].ID, newPos.x, newPos.y);
	}
}

void Tri_findAllPossibleRemainPoints(robotMeas_t* pNeighborsTable,
		oneHopMeas_t* pOneHopTable)
{
	bool isCalculatedNewNeighborCooridation = false;
	bool isSuccess = false;

	int neighborPointer = 0;

	float edgeOP = 0;
	float edgeOQ = 0;
	float edgePQ = 0;

	float edgeOK = 0;
	float edgePK = 0;
	float edgeQK = 0;

	float alphaP = 0;
	float alphaQ = 0;

	robotMeas_t robotK;
	robotMeas_t robotQ;
	robotMeas_t robotP;

	vector2_t newPos;

	while (1)
	{
		// duyet cac neighbor Z trong Bang 1
		for (neighborPointer = 0; neighborPointer < g_ui8NeighborsCounter;
				neighborPointer++)
		{
			// neu K da co toa do thi skip
			robotK.ID = pNeighborsTable[neighborPointer].ID;
			if (Tri_isLocationsTableContain(robotK.ID))
				continue;

			// check bang neighbors cua Robot K tim khoan cach K->O
			robotK.distance = Tri_tryToGetDistance(pOneHopTable, robotK.ID,
					g_ui32RobotID);
			if (robotK.distance == 0)
				continue;

			// tinh trung binh voi khoan cach O->K
			robotK.distance = (robotK.distance
					+ pNeighborsTable[neighborPointer].distance) / 2;

			// tim 2 robot P va Q la neighbor cua K trong bang toa do
			isSuccess = Tri_tryToFindTwoLocatedNeighbors(pOneHopTable,
					robotK.ID, &robotP, &robotQ);

			if (isSuccess)
			{
				// edgeOK, edgePK va edgeQK da tinh xong trong robotK, robotP va robotQ, tim edgeOP, edgeOQ va edgePQ
				Tri_getThreeEdgeFormOriginal(pNeighborsTable, pOneHopTable,
						&edgeOP, &edgeOQ, &edgePQ, g_ui32RobotID, robotP.ID,
						robotQ.ID);

				edgeOK = robotK.distance / 256.0f;
				edgeQK = robotQ.distance / 256.0f;
				edgePK = robotP.distance / 256.0f;

				// tinh goc P va goc Q, ket qua la so duong
				alphaP = Tri_findAngleInLocalCoordination(edgeOP, robotP.ID);
				alphaQ = Tri_findAngleInLocalCoordination(edgeOQ, robotQ.ID);

				//chon goc alphaP la goc nho hon trong 2 goc Q va P
				// alphaP = (alphaP < alphaQ) ? (alphaP) : (alphaQ);
				if (((alphaP > alphaQ) && ((alphaP - alphaQ) < MATH_PI))
						|| ((alphaQ - alphaP) > MATH_PI))
				{

					float tempEdge = edgeOP;
					edgeOP = edgeOQ;
					edgeOQ = tempEdge;

					tempEdge = edgeQK;
					edgeQK = edgePK;
					edgePK = tempEdge;

					alphaP = alphaQ;
				}

				// tinh toa do cua Z roi offset goc P
				newPos = Tri_trilaterateFromSixEdgeAndAngleOffset(edgeOQ,
						edgeOP, edgeOK, edgeQK, edgePK, edgePQ, alphaP);

				if (newPos.x == 0 && newPos.y == 0)
					continue;

				// luu lai vao bang toa do
				Tri_addLocation(robotK.ID, newPos.x, newPos.y);

				// neu suy duoc them robot trong bang toa do thi duyet lai tu dau
				isCalculatedNewNeighborCooridation = true;

				break;
			}
		}
		if (isCalculatedNewNeighborCooridation)
		{
			isCalculatedNewNeighborCooridation = false;
			continue;
		}
		else
			break;
	}
}

float Tri_findAngleInLocalCoordination(float edgeR, uint32_t rID)
{
	float cosAlpha;
	float alpha;

	int locsPointer;

	for (locsPointer = 0; locsPointer < g_ui8LocsCounter; locsPointer++)
	{
		if (locs[locsPointer].ID == rID)
			break;
	}

	cosAlpha = locs[locsPointer].vector.x / edgeR;
	alpha = calACos(cosAlpha);

	if (locs[locsPointer].vector.y < 0)
		alpha = MATH_PI_MUL_2 - alpha;

	return alpha;
}

void Tri_getThreeEdgeFormOriginal(robotMeas_t* pNeighborsTable,
		oneHopMeas_t* pOneHopTable, float* edgeOP, float* edgeOQ, float* edgePQ,
		uint32_t oID, uint32_t pID, uint32_t qID)
{
	uint8_t neighborPointer;
	uint8_t onehopPointer;

	uint16_t distanceOP = 0;
	uint16_t distanceOQ = 0;
	uint16_t distancePQ = 0;

	for (neighborPointer = 0; neighborPointer < g_ui8NeighborsCounter;
			neighborPointer++)
	{
		if (pNeighborsTable[neighborPointer].ID == pID)
		{
			distanceOP += pNeighborsTable[neighborPointer].distance;
		}
		else if (pNeighborsTable[neighborPointer].ID == qID)
		{
			distanceOQ += pNeighborsTable[neighborPointer].distance;
		}
	}

	for (onehopPointer = 0; onehopPointer < ONEHOP_NEIGHBOR_TABLE_LENGTH;
			onehopPointer++)
	{
		if (pOneHopTable[onehopPointer].firstHopID == pID)
		{
			for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
					neighborPointer++)
			{
				if (pOneHopTable[onehopPointer].neighbors[neighborPointer].ID
						== oID)
				{
					distanceOP +=
							pOneHopTable[onehopPointer].neighbors[neighborPointer].distance;
				}
				else if (pOneHopTable[onehopPointer].neighbors[neighborPointer].ID
						== qID)
				{
					distancePQ +=
							pOneHopTable[onehopPointer].neighbors[neighborPointer].distance;
				}
			}
		}
		else if (pOneHopTable[onehopPointer].firstHopID == qID)
		{
			for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
					neighborPointer++)
			{
				if (pOneHopTable[onehopPointer].neighbors[neighborPointer].ID
						== oID)
				{
					distanceOQ +=
							pOneHopTable[onehopPointer].neighbors[neighborPointer].distance;
				}
				else if (pOneHopTable[onehopPointer].neighbors[neighborPointer].ID
						== pID)
				{
					distancePQ +=
							pOneHopTable[onehopPointer].neighbors[neighborPointer].distance;
				}
			}
		}
	}

	*edgeOP = distanceOP / 512.0f;
	*edgeOQ = distanceOQ / 512.0f;
	*edgePQ = distancePQ / 512.0f;
}

bool Tri_tryToFindTwoLocatedNeighbors(oneHopMeas_t* pOneHopTable,
		uint32_t robotK_ID, robotMeas_t* pRobotP, robotMeas_t* pRobotQ)
{
	uint8_t locsPointerP;
	uint8_t locsPointerQ;
	uint8_t oneHopPointer;

	// neu robot K khong co trong bang one hop thi bo qua robot K
	for (oneHopPointer = 0; oneHopPointer < g_ui8NeighborsCounter;
			oneHopPointer++)
	{
		if (pOneHopTable[oneHopPointer].firstHopID == robotK_ID)
			break;
	}
	if (oneHopPointer >= g_ui8NeighborsCounter)
		return false;

	// chon robot P trong bang toa do tu tren xuong, bo goc toa do O(0; 0)
	for (locsPointerP = 1; locsPointerP < g_ui8LocsCounter; locsPointerP++)
	{
		pRobotP->ID = locs[locsPointerP].ID;

		// neu robot P khong phai neighbor cua robot K thi skip, nguoc lai lay P->K
		pRobotP->distance = Tri_tryToGetDistance(pOneHopTable, pRobotP->ID,
				robotK_ID);
		if (pRobotP->distance == 0)
			continue;

		// robot P hop le, chon tiep robot Q tu vi tri robot P tro xuong trong bang toa do
		for (locsPointerQ = (locsPointerP + 1); locsPointerQ < g_ui8LocsCounter;
				locsPointerQ++)
		{
			pRobotQ->ID = locs[locsPointerQ].ID;

			// neu robot Q khong phai neighbor cua robot K thi skip, nguoc la lay Q->K
			pRobotQ->distance = Tri_tryToGetDistance(pOneHopTable, pRobotQ->ID,
					robotK_ID);
			if (pRobotQ->distance == 0)
				continue;

			// tinh edgePK va edgeQK tu bang neighbor cua robot K
			Tri_calculateAverageDistance(pOneHopTable[oneHopPointer].neighbors,
					pRobotP, pRobotQ);

			return true;
		}
	}
	return false;
}

vector2_t Tri_trilaterateFromSixEdgeAndAngleOffset(float edgeIQ, float edgeIP,
		float edgeIJ, float edgeQJ, float edgePJ, float edgePQ,
		float angleOffset)
{
	vector2_t tempVector;

	tempVector.x = 0;
	tempVector.y = 0;

	if (!isTriangle(edgeIP, edgeIJ, edgePJ))
		return tempVector;

	if (!isTriangle(edgeIQ, edgeIJ, edgeQJ))
		return tempVector;

	if (!isTriangle(edgeIP, edgeIQ, edgePQ))
		return tempVector;

	float AlphaPIJ = calACos(cosinesRuleForTriangles(edgeIP, edgeIJ, edgePJ));
	float BetaQIJ = calACos(cosinesRuleForTriangles(edgeIQ, edgeIJ, edgeQJ));
	float ThetaQIP = calACos(cosinesRuleForTriangles(edgeIP, edgeIQ, edgePQ));

	int8_t signedAlphaJ = Tri_findSignedYaxis(AlphaPIJ, BetaQIJ, ThetaQIP);

	if (signedAlphaJ == 0)
		return tempVector;

	float alphaJ;
	alphaJ = AlphaPIJ * signedAlphaJ;
	//alphaJ = angleOffset + AlphaPIJ;
	alphaJ += angleOffset;

	tempVector.x = edgeIJ * calCos(alphaJ);
	tempVector.y = edgeIJ * calSin(alphaJ);

	return tempVector;
}

int8_t Tri_findSignedYaxis(float Alpha, float Beta, float Theta)
{
	float temp = Alpha - Theta;

	temp = (temp > 0) ? (temp) : (-temp);

	if ((Beta < (temp + ANGLE_MIN_IN_RAD))
			&& (Beta > (temp - ANGLE_MIN_IN_RAD)))
		return (1);

//	temp = Alpha + Theta;
//	if ((Beta < (temp + ANGLE_MIN_IN_RAD))
//			&& (Beta > (temp - ANGLE_MIN_IN_RAD)))
		return (-1);
//
//	return (0);
}

uint16_t Tri_tryToGetNeighborsDistance(robotMeas_t* pNeighborsTable,
		uint32_t checkingID)
{
	int neighborPointer;
	for (neighborPointer = 0; neighborPointer < g_ui8NeighborsCounter;
			neighborPointer++)
	{
		if (pNeighborsTable[neighborPointer].ID == checkingID)
			return pNeighborsTable[neighborPointer].distance;
	}
	return 0;
}

uint16_t Tri_tryToGetDistance(oneHopMeas_t* pOneHopTable, uint32_t firstHopID,
		uint32_t checkingID)
{
	int onehopPointer;
	int neighborPointer;
	for (onehopPointer = 0; onehopPointer < ONEHOP_NEIGHBOR_TABLE_LENGTH;
			onehopPointer++)
	{
		if (pOneHopTable[onehopPointer].firstHopID == firstHopID)
			break;
	}

	if (onehopPointer == ONEHOP_NEIGHBOR_TABLE_LENGTH)
		return 0; // out of range

	for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
			neighborPointer++)
	{
		if (pOneHopTable[onehopPointer].neighbors[neighborPointer].ID
				== checkingID)
			return pOneHopTable[onehopPointer].neighbors[neighborPointer].distance;
	}
	return 0;
}

bool Tri_isLocationsTableContain(uint32_t robotID)
{
	int i;
	for (i = 0; i < LOCATIONS_TABLE_LENGTH; i++)
	{
		if (locs[i].ID == robotID)
		{
			return true;
		}
	}
	return false;
}

uint16_t Tri_tryToFindTwoXYNeighbors(oneHopMeas_t* pOneHopTable,
		robotMeas_t* pRobotX, robotMeas_t* pRobotY, robotMeas_t* pNeighborTable)
{
	int oneHopPointer;
	int neighborPointer;
	uint16_t YtoXdistance;
	uint16_t edgeXY;

	for (oneHopPointer = 0; oneHopPointer < ONEHOP_NEIGHBOR_TABLE_LENGTH;
			oneHopPointer++)
	{
		pRobotX->ID = pOneHopTable[oneHopPointer].firstHopID;

		for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
				neighborPointer++)
		{
			pRobotY->ID =
					pOneHopTable[oneHopPointer].neighbors[neighborPointer].ID;

			if (pRobotY->ID != g_ui32RobotID) // make sure it not my ID
			{
				YtoXdistance = Tri_isRobotYNeighborsIncludeRobotX(pOneHopTable,
						pRobotX, pRobotY);
				if (YtoXdistance != 0)
				{
					Tri_getDistanceFromRobotToMe(pRobotX,
							pOneHopTable[oneHopPointer].neighbors);

					Tri_calculateAverageDistance(pNeighborTable, pRobotX,
							pRobotY);

					edgeXY =
							(YtoXdistance
									+ pOneHopTable[oneHopPointer].neighbors[neighborPointer].distance)
									/ 2;

					if (isValidTriangle(pRobotX->distance, pRobotY->distance,
							edgeXY))
					{
						return edgeXY;
					}
					else
						break;
				}
			}
		}
	}

	pRobotX->ID = 0;

	pRobotY->ID = 0;

	return 0;
}

uint16_t Tri_isRobotYNeighborsIncludeRobotX(oneHopMeas_t* pOneHopTable,
		robotMeas_t* pRobotX, robotMeas_t* pRobotY)
{
	int oneHopPointer;
	int neighborPointer;

	// check if robot Y is my neighbor
	for (oneHopPointer = 0; oneHopPointer < ONEHOP_NEIGHBOR_TABLE_LENGTH;
			oneHopPointer++)
	{
		if (pOneHopTable[oneHopPointer].firstHopID == pRobotY->ID)
			break;
	}

	if (oneHopPointer >= ONEHOP_NEIGHBOR_TABLE_LENGTH)
		return 0; // out of range, robot Y not my neighbor

	for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
			neighborPointer++)
	{
		// check if robot X is robot Y neighbor
		if (pOneHopTable[oneHopPointer].neighbors[neighborPointer].ID
				== pRobotX->ID)
		{
			Tri_getDistanceFromRobotToMe(pRobotY,
					pOneHopTable[oneHopPointer].neighbors);

			// return Y->X distance
			return pOneHopTable[oneHopPointer].neighbors[neighborPointer].distance;
		}
	}
	return 0;
}

void Tri_getDistanceFromRobotToMe(robotMeas_t* pRobot,
		robotMeas_t* pNeighborsTable)
{
	// WARNING: this function must only be used in case ALWAY HAVE my ID in pNeighborsTable!

	int neighborPointer;

	for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
			neighborPointer++)
	{
		if (pNeighborsTable[neighborPointer].ID == g_ui32RobotID) // search my ID in robot neighbors
		{
			pRobot->distance = pNeighborsTable[neighborPointer].distance;
			return;
		}
	}
}

void Tri_calculateAverageDistance(robotMeas_t pNeighborsTable[],
		robotMeas_t* pRobotX, robotMeas_t* pRobotY)
{
	// WARNING: this function must only be used in case ALWAY HAVE robotX and robotY in pNeighborsTable!
	// and pRobotX.distance, pRobotY.distance ALREADY store distance between that robot to me!

	int foundCounter = 0;

	int neighborPointer;

	for (neighborPointer = 0; neighborPointer < NEIGHBOR_TABLE_LENGTH;
			neighborPointer++)
	{
		if (pNeighborsTable[neighborPointer].ID == pRobotX->ID)
		{
			pRobotX->distance = (pRobotX->distance
					+ pNeighborsTable[neighborPointer].distance) / 2;
			foundCounter++;
		}
		else if ((pNeighborsTable + neighborPointer)->ID == pRobotY->ID)
		{
			pRobotY->distance = (pRobotY->distance
					+ pNeighborsTable[neighborPointer].distance) / 2;
			foundCounter++;
		}
		if (foundCounter >= 2)
			return;
	}
}
