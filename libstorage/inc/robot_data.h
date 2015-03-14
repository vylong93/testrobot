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
void clearNeighborsTable(void);
int getCurrentNeighborsNumber(void);

void addOverrideToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance);
void addToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance);
bool isNeighborsTableContainRobot(uint32_t ui32RobotId);
void fillNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);
void constructNeighborsTableFromByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

// ====== One Hop Neighbors Table manipulation methods =============================
void clearOneHopNeighborsTable(void);
int getCurrentOneHopNeighborsNumber(void);
void addToOneHopNeighborsTable(uint32_t ui32NeighborId, uint8_t* pui8TableBuffer, uint32_t ui32TableSizeInByte);
bool isOneHopNeighborsTableContainRobot(uint32_t ui32RobotId);
void fillOneHopNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DATA_H_ */
