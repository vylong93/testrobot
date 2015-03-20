/*
 * Trilateration.h
 *
 *  Created on: Sep 6, 2014
 *      Author: MasterE
 */

#ifndef TRILATERATION_H_
#define TRILATERATION_H_

#include "libstorage/inc/CustomLinkedList.h"
#include "libstorage/inc/RobotLocation.h"
#include "libmath/inc/Vector2.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void initData(uint8_t* pui8MessageData);

bool Tri_tryToCalculateRobotLocationsTable(uint32_t ui32RobotOsId);

bool Tri_tryToFindTwoBestXYNeighbors(uint32_t ui32RobotOsId, uint32_t* pui32RobotXsId, uint32_t* pui32RobotYsId, uint16_t* pui16EdgeOX, uint16_t* pui16EdgeOY, uint16_t* pui16EdgeXY);
void Tri_calculateXYLocations(uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);

void Tri_findAllPointsFromFirstTwoPoints(uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);
void GradientDescentMulti_correctLocationsTable(uint32_t ui32OriginalID, uint32_t ui32RotationHopID);

void Tri_findAllPossibleRemainPoints();

bool Tri_tryToGetEdgesOfThreeRobotOPQ(uint32_t ui32RobotPsId, uint32_t ui32RobotQsId, uint16_t *pui16EdgeOP, uint16_t *pui16EdgeOQ, uint16_t *pui16EdgePQ);
bool Tri_tryToFindAngleInLocalCoordination(float fEdgeR, uint32_t ui32RobotId, float* pfAlpha);
bool Tri_calculateAlphaFromSixEdge(float* pfAlphaPIJ, uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ,
		uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ);

bool Tri_tryToRotateLocationsTable(uint32_t ui32SelfID, uint32_t ui32RotationHopID, float fRotationHopXvalue, float fRotationHopYvalue, uint8_t* pui8OriLocsTableBufferPointer, int32_t ui32SizeOfOriLocsTable);

#ifdef __cplusplus
}
#endif

Vector2<float> Tri_chooseTheBestVectorFromThreePoints(float fAlphaZ, uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);
float Tri_calculateErrorOfVectorZ(Vector2<float> vectZ, Vector2<float> vectO, Vector2<float> vectX, Vector2<float> vectY, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);

Vector2<float> Tri_trilaterateFromSixEdgeAndAngleOffset(uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ, uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ, float fAngleOffset);
int8_t Tri_findSignedYaxis(float fAlpha, float fBeta, float fTheta);

bool Tri_tryToGetCommonNeighborID(CustomLinkedList<RobotLocation>* pOriLocsTable, uint32_t* pui32CommonID, uint32_t ui32SelfID, uint32_t ui32RotationHopID);
bool Tri_calculateCorrectionAngle(uint32_t ui32SelfID, uint32_t ui32RotationHopID, uint32_t ui32IdsRobotJ, CustomLinkedList<RobotLocation>* pOriLocsTable, float *pfCorrectionAngle, bool* pbIsNeedMirroring);
float Tri_getRobotAngleFromLocationsTable(uint32_t ui32RobotId, CustomLinkedList<RobotLocation>* pLocsTable);
Vector2<float> Tri_updateRobotVectorToWorldFrame(uint32_t ui32SelfId, Vector2<float> vectRotationHop, CustomLinkedList<RobotLocation>* pOriLocsTable);

#endif /* TRILATERATION_H_ */
