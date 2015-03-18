/*
 * Trilateration.h
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#ifndef TRILATERATION_H_
#define TRILATERATION_H_


#include "libstorage/inc/RobotLocation.h"
#include "libmath/inc/Vector2.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void initDataOfRobot3(void);

// (1) void Tri_clearLocs(location_t locsTable[], uint8_t *length);
// ----> replace by void RobotLocationsTable_clear(void);

// (2) void Tri_addLocation(uint32_t ID, float x, float y);
// ----> replace by void RobotLocationsTable_add(uint32_t ID, float x, float y);

// (OK) void Tri_findLocs(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable);
bool Tri_tryToCalculateRobotLocationsTable(void);

// (OK) uint16_t Tri_tryToFindTwoXYNeighbors(oneHopMeas_t* pOneHopTable, robotMeas_t* pRobotX, robotMeas_t* pRobotY, robotMeas_t* pNeighborTable);
bool Tri_tryToFindTwoBestXYNeighbors(uint32_t* pui32RobotXsId, uint32_t* pui32RobotYsId, uint16_t* pui16EdgeOX, uint16_t* pui16EdgeOY, uint16_t* pui16EdgeXY);
void Tri_calculateXYLocations(uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);

void Tri_findAllPointsFromFirstTwoPoints(uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);
void GradientDescentMulti_correctLocationsTable(uint32_t ui32OriginalID);

void Tri_findAllPossibleRemainPoints();

bool Tri_tryToGetEdgesOfThreeRobotOPQ(uint32_t ui32RobotPsId, uint32_t ui32RobotQsId, uint16_t *pui16EdgeOP, uint16_t *pui16EdgeOQ, uint16_t *pui16EdgePQ);
bool Tri_tryToFindAngleInLocalCoordination(float fEdgeR, uint32_t ui32RobotId, float* pfAlpha);
bool Tri_calculateAlphaFromSixEdge(float* pfAlphaPIJ, uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ,
		uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ);

//void Tri_getDistanceFromRobotToMe(robotMeas_t* pRobot, robotMeas_t* pNeighborsTable);
//void Tri_calculateAverageDistance(robotMeas_t pNeighborsTable[], robotMeas_t* pRobotX, robotMeas_t* pRobotY);
//
//void Tri_findAllPointsFromFirstTwoPoints(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable, robotMeas_t robotX, robotMeas_t robotY, float edgeXY);
//void Tri_findAllPossibleRemainPoints(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable);
//
//bool Tri_tryToFindTwoLocatedNeighbors(oneHopMeas_t* pOneHopTable, uint32_t robotK_ID, robotMeas_t* pRobotP, robotMeas_t* pRobotQ);
//
//bool Tri_isLocationsTableContain(uint32_t robotID);
//float Tri_findAngleInLocalCoordination(float edgeR, uint32_t rID);
//
//void Tri_getThreeEdgeFormOriginal(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable, float* edgeOP, float* edgeOQ, float* edgePQ, uint32_t oID, uint32_t pID, uint32_t qID);
//uint16_t Tri_tryToGetNeighborsDistance(robotMeas_t* pNeighborsTable, uint32_t checkingID);
//uint16_t Tri_tryToGetDistance(oneHopMeas_t* pOneHopTable, uint32_t firstHopID, uint32_t checkingID);

#ifdef __cplusplus
}
#endif

Vector2<float> Tri_chooseTheBestVectorFromThreePoints(float fAlphaZ, uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);
float Tri_calculateErrorOfVectorZ(Vector2<float> vectZ, Vector2<float> vectO, Vector2<float> vectX, Vector2<float> vectY, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);

Vector2<float> Tri_trilaterateFromSixEdgeAndAngleOffset(uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ, uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ, float fAngleOffset);
int8_t Tri_findSignedYaxis(float fAlpha, float fBeta, float fTheta);

#endif /* TRILATERATION_H_ */
