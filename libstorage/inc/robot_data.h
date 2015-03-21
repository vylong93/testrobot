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

#define SIZE_OF_ROBOT_ID				4
#define SIZE_OF_ROBOT_MEAS				6
#define SIZE_OF_ROBOT_LOCATION			12

#define MAXIMUM_SIZE_OF_ONEHOP_MEAS		64	// (4 +  NEIGHBORS_TABLE_LENGTH * SIZE_OF_ROBOT_MEAS)

#define NEIGHBORS_TABLE_LENGTH 			10
#define ONEHOP_NEIGHBORS_TABLE_LENGTH 	NEIGHBORS_TABLE_LENGTH
#define LOCATIONS_TABLE_LENGTH			NEIGHBORS_TABLE_LENGTH


//typedef struct tagVector {
//	float x;
//	float y;
//} vector2_t;
//
//typedef struct tagLocation {
//	uint32_t ID;
//	vector2_t vector;
//} location_t;

void initLinkedList(void);

// ====== Neighbors Table manipulation methods =====================================
int NeighborsTable_getSize(void);
void NeighborsTable_clear(void);

void NeighborsTable_addOverride(uint32_t ui32NeighborId, uint16_t ui16Distance);
void NeighborsTable_add(uint32_t ui32NeighborId, uint16_t ui16Distance);
bool NeighborsTable_isContainRobot(uint32_t ui32RobotId);
uint32_t NeighborsTable_getIdAtIndex(uint32_t ui32Index);
uint16_t NeighborsTable_getDistanceOfRobot(uint32_t ui32RobotID);
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
int RobotLocationsTable_getIndexOfRobot(uint32_t ui32RobotID);
bool RobotLocationsTable_isContainRobot(uint32_t ui32RobotId);
uint32_t RobotLocationsTable_getIdAtIndex(uint32_t ui32Index);
void RobotLocationsTable_rotate(float fAngle, bool bFlipXaxis);
void RobotLocationsTable_transformToWorldFrame(uint32_t ui32RotationHopId, float fRotationHopXvalue, float fRotationHopYvalue);
void RobotLocationsTable_linearTransform(float dx, float dy);
void RobotLocationsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DATA_H_ */
