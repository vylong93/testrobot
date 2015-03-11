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

#define SIZE_OF_ROBOT_MEAS				6

#define NEIGHBORS_TABLE_LENGTH 			10
#define ONEHOP_NEIGHBORS_TABLE_LENGTH 	NEIGHBORS_TABLE_LENGTH
#define LOCATIONS_TABLE_LENGTH			NEIGHBORS_TABLE_LENGTH

//typedef struct tagRobotMeas
//{
//	uint32_t ID;
//	uint16_t distance; // <8.8>
//} robotMeas_t;
//
//typedef struct tagNeighborsTable
//{
//	robotMeas_t pNeighbors[NEIGHBOR_TABLE_LENGTH];
//	uint8_t ui8Counter;
//} neighborsArray_t;

//typedef struct tagOneHopMeas
//{
//	uint32_t firstHopID;
//	robotMeas_t neighbors[NEIGHBOR_TABLE_LENGTH];
//} oneHopMeas_t;
//
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

void clearNeighborsTable(void);
void addOverrideToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance);
void addToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance);
void fillNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_DATA_H_ */
