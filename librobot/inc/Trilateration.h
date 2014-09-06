/*
 * Trilateration.h
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#ifndef TRILATERATION_H_
#define TRILATERATION_H_

#include "librobot/inc/MainBoardDriver.h"

#define LOCATIONS_TABLE_LENGTH	10

typedef struct tagVector {
	float x;
	float y;
} vector2_t;

typedef struct tagLocation {
	uint32_t ID;
	vector2_t vector;
} location_t;

void Tri_clearLocs();
void Tri_addLocation(uint32_t ID, float x, float y);

void Tri_findLocs(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable);

uint16_t Tri_tryToFindTwoXYNeighbors(oneHopMeas_t* pOneHopTable, robotMeas_t* pRobotX, robotMeas_t* pRobotY, robotMeas_t* pNeighborTable);
uint16_t Tri_isRobotYNeighborsIncludeRobotX(oneHopMeas_t* pOneHopTable, robotMeas_t* pRobotX, robotMeas_t* pRobotY);
void Tri_getDistanceFromRobotToMe(robotMeas_t* pRobot, robotMeas_t* pNeighborsTable);
void Tri_calculateAverageDistance(robotMeas_t pNeighborsTable[], robotMeas_t* pRobotX, robotMeas_t* pRobotY);

void Tri_findAllPointsFromFirstTwoPoints(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable, robotMeas_t robotX, robotMeas_t robotY, float edgeXY);
void Tri_findAllPossibleRemainPoints(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable);

bool Tri_tryToFindTwoLocatedNeighbors(oneHopMeas_t* pOneHopTable, uint32_t robotK_ID, robotMeas_t* pRobotP, robotMeas_t* pRobotQ);

bool Tri_isLocationsTableContain(uint32_t robotID);
float Tri_findAngleInLocalCoordination(float edgeR, uint32_t rID);

void Tri_getThreeEdgeFormOriginal(robotMeas_t* pNeighborsTable, oneHopMeas_t* pOneHopTable, float* edgeOP, float* edgeOQ, float* edgePQ, uint32_t oID, uint32_t pID, uint32_t qID);
uint16_t Tri_tryToGetNeighborsDistance(robotMeas_t* pNeighborsTable, uint32_t checkingID);
uint16_t Tri_tryToGetDistance(oneHopMeas_t* pOneHopTable, uint32_t firstHopID, uint32_t checkingID);

vector2_t Tri_trilaterateFromSixEdgeAndAngleOffset(float edgeIQ, float edgeIP, float edgeIJ, float edgeQJ, float edgePJ, float edgePQ, float angleOffset);
bool Tri_isOnNegativeYaxis(float cosAlpha, float cosBeta, float cosTheta);

#endif /* TRILATERATION_H_ */
