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
#include "libmath/inc/custom_math.h"
#include "libmath/inc/Vector2.h"

#include "libstorage/inc/RobotIdentity.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void initDataOfRobot1(void); // Test Only
void initData(uint8_t* pui8MessageData); // Test Only

bool Tri_tryToCalculateRobotLocationsTable(uint32_t ui32RobotOsId);

bool Tri_tryToRotateLocationsTable(RobotIdentity_t* pRobotIdentity, uint8_t* pui8OriLocsTableBufferPointer, int32_t ui32SizeOfOriLocsTable);

#ifdef __cplusplus
}
#endif

bool Tri_tryToFindTwoBestXYNeighbors(uint32_t ui32RobotOsId, uint32_t* pui32RobotXsId, uint32_t* pui32RobotYsId, uint16_t* pui16EdgeOX, uint16_t* pui16EdgeOY, uint16_t* pui16EdgeXY);
void Tri_calculateXYLocations(uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);

//============= New Attempt: Edges Only =============================================================
void Tri_tryToCalculateTheRemainPoints(uint32_t ui32RobotIsId);
bool Tri_calculateRobotLocationBaseOnThreeLocatedPoint(float fEdgeIJ, float fEdgePJ, float fEdgeQJ, Vector2<float> pointI, Vector2<float> pointP, Vector2<float> pointQ, Vector2<float>* pPointJ);
bool Tri_calculateTwoCrossPointBetweenLineAndCircle(LineEquation_t line, CircleEquation_t circle, Vector2<float>* pX1, Vector2<float>* pX2);

//============= Old Attempt: Trigonometric with Angles ==============================================
void Tri_findAllPointsFromFirstTwoPoints(uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, uint16_t ui16EdgeOX, uint16_t ui16EdgeOY, uint16_t ui16EdgeXY);
void Tri_findAllPossibleRemainPoints(void);

bool Tri_calculateAlphaFromSixEdge(float* pfAlphaPIJ, uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ, uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ);
Vector2<float> Tri_chooseTheBestVectorFromThreePoints(float fAlphaZ, uint32_t ui32RobotOsId, uint32_t ui32RobotXsId, uint32_t ui32RobotYsId, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);
float Tri_calculateErrorOfVectorZ(Vector2<float> vectZ, Vector2<float> vectO, Vector2<float> vectX, Vector2<float> vectY, float fEdgeOZ, float fEdgeXZ, float fEdgeYZ);

bool Tri_tryToGetEdgesOfThreeRobotOPQ(uint32_t ui32RobotPsId, uint32_t ui32RobotQsId, uint16_t *pui16EdgeOP, uint16_t *pui16EdgeOQ, uint16_t *pui16EdgePQ);
bool Tri_tryToFindAngleInLocalCoordination(float fEdgeR, uint32_t ui32RobotId, float* pfAlpha);
Vector2<float> Tri_trilaterateFromSixEdgeAndAngleOffset(uint16_t ui16EdgeIQ, uint16_t ui16EdgeIP, uint16_t ui16EdgeIJ, uint16_t ui16EdgeQJ, uint16_t ui16EdgePJ, uint16_t ui16EdgePQ, float fAngleOffset);
int8_t Tri_findSignedYaxis(float fAlpha, float fBeta, float fTheta);

//============= Rotate Coordinates ==================================================================
bool Tri_tryToGetCommonNeighborID(RobotIdentity_t* pRobotIdentity, CustomLinkedList<RobotLocation>* pOriLocsTable, uint32_t* pui32CommonID);
bool Tri_calculateCorrectionAngle(RobotIdentity_t* pRobotIdentity, uint32_t ui32IdsRobotJ, CustomLinkedList<RobotLocation>* pOriLocsTable, float *pfCorrectionAngle, bool* pbIsNeedMirroring);
float Tri_getRobotAngleFromLocationsTable(uint32_t ui32RobotId, CustomLinkedList<RobotLocation>* pLocsTable);
Vector2<float> Tri_getRobotVectorFromLocationsTable(uint32_t ui32RobotId, CustomLinkedList<RobotLocation>* pLocsTable);
Vector2<float> Tri_updateRobotVectorToWorldFrame(RobotIdentity_t* pRobotIdentity, CustomLinkedList<RobotLocation>* pOriLocsTable);

#endif /* TRILATERATION_H_ */
