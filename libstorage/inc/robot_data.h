/*
 * robot_data.h
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#ifndef ROBOT_DATA_H_
#define ROBOT_DATA_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "libstorage/inc/RobotIdentity.h"

#define SIZE_OF_ROBOT_ID				4
#define SIZE_OF_ROBOT_MEAS				6
#define SIZE_OF_ROBOT_LOCATION			12

#define MAXIMUM_SIZE_OF_ONEHOP_MEAS		64	// (4 +  NEIGHBORS_TABLE_LENGTH * SIZE_OF_ROBOT_MEAS)

#define NEIGHBORS_TABLE_LENGTH 			10
#define ONEHOP_NEIGHBORS_TABLE_LENGTH 	NEIGHBORS_TABLE_LENGTH
#define LOCATIONS_TABLE_LENGTH			NEIGHBORS_TABLE_LENGTH

void initLinkedList(void);

// ====== Neighbors Table manipulation methods =====================================
int NeighborsTable_getSize(void);
void NeighborsTable_clear(void);

void NeighborsTable_addOverride(uint32_t ui32NeighborId, uint16_t ui16Distance);
void NeighborsTable_add(uint32_t ui32NeighborId, uint16_t ui16Distance);
void NeighborsTable_updateNewDistanceForNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance);
bool NeighborsTable_isContainRobot(uint32_t ui32RobotId);
uint32_t NeighborsTable_getIdAtIndex(uint32_t ui32Index);
bool NeighborsTable_getDistanceOfRobot(uint32_t ui32RobotID, uint16_t *pui16Distance);
void NeighborsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

// ====== One Hop Neighbors Table manipulation methods =============================
void OneHopNeighborsTable_clear(void);
int OneHopNeighborsTable_getSize(void);

void OneHopNeighborsTable_add(uint32_t ui32NeighborId, uint8_t* pui8TableBuffer, uint32_t ui32TableSizeInByte);
bool OneHopNeighborsTable_isContainRobot(uint32_t ui32RobotId);
uint32_t OneHopNeighborsTable_getFirstHopIdAtIndex(uint32_t ui32Index);
int OneHopNeighborsTable_getIndexOfRobot(uint32_t ui32RobotID);
void OneHopNeighborsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

// ====== Robot Locations Table manipulation methods =============================
void RobotLocationsTable_clear(void);
int RobotLocationsTable_getSize(void);
void RobotLocationsTable_add(uint32_t id, float x, float y);
void RobotLocationsTable_remove(uint32_t id);
void RobotLocationsTable_updateLocation(uint32_t id, float x, float y);
int RobotLocationsTable_getIndexOfRobot(uint32_t ui32RobotID);
bool RobotLocationsTable_isContainRobot(uint32_t ui32RobotId);
bool RobotLocationsTable_getVectorOfRobot(uint32_t ui32RobotId, float* pfX, float *pfY);
uint32_t RobotLocationsTable_getIdAtIndex(uint32_t ui32Index);
bool RobotLocationTable_setVectorOfRobot(uint32_t ui32RobotId, float fX, float fY);
void RobotLocationsTable_rotate(float fAngle, bool bFlipXaxis);
void RobotLocationsTable_transformToWorldFrame(RobotIdentity_t* pRobotIdentity);
void RobotLocationsTable_linearTransform(float dx, float dy, uint32_t ui32OriginId);
void RobotLocationsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);
void RobotLocationsTable_fillContentToByteBufferOffsetLocal(uint32_t ui32LocalOriginId, uint8_t* pui8Buffer, uint32_t ui32TotalLength);
void RobotLocationsTable_selfCorrectByGradientDescent(uint32_t ui32OriginalID, uint32_t ui32RotationHopID);
#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DATA_H_ */
