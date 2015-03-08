/*
 * robot_data.c
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/robot_data.h"
#include "libprotocol/inc/network.h"

neighborsArray_t g_NeighborsArray;

//oneHopMeas_t OneHopNeighborsTable[ONEHOP_NEIGHBOR_TABLE_LENGTH];
//location_t locs[LOCATIONS_TABLE_LENGTH];

void initLinkedList(void)
{
	clearNeighborsTable();
}

void clearNeighborsTable(void)
{
	//
	// Clear neighbors table
	//
	uint8_t i;
	for(i = 0; i < NEIGHBOR_TABLE_LENGTH; i++)
	{
		g_NeighborsArray.pNeighbors[i].ID = 0;
		g_NeighborsArray.pNeighbors[i].distance = 0;
	}
	g_NeighborsArray.ui8Counter = 0;
}

void addToNeighborsTable(uint32_t neighborId, uint16_t distance)
{
	if(g_NeighborsArray.ui8Counter < NEIGHBOR_TABLE_LENGTH)
	{
		g_NeighborsArray.pNeighbors[g_NeighborsArray.ui8Counter].ID = neighborId;
		g_NeighborsArray.pNeighbors[g_NeighborsArray.ui8Counter].distance = distance;

		g_NeighborsArray.ui8Counter = g_NeighborsArray.ui8Counter + 1;
	}
	else
	{
		uint8_t i;
		uint8_t max_i = 0;
		for(i = 1; i < NEIGHBOR_TABLE_LENGTH; i++)	// Search for the worth result
		{
			if(g_NeighborsArray.pNeighbors[max_i].distance < g_NeighborsArray.pNeighbors[i].distance)
				max_i = i;
		}
		g_NeighborsArray.pNeighbors[max_i].ID = neighborId;
		g_NeighborsArray.pNeighbors[max_i].distance = distance;

		g_NeighborsArray.ui8Counter = NEIGHBOR_TABLE_LENGTH;
	}
}

void fillNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	uint32_t i;
	uint8_t ui8NeighborsPointer = 0;
	for(i = 0; i < ui32TotalLength; i += 6)
	{
		parse32bitTo4Bytes(&pui8Buffer[i], g_NeighborsArray.pNeighbors[ui8NeighborsPointer].ID);
		pui8Buffer[i + 4] = (uint8_t)(g_NeighborsArray.pNeighbors[ui8NeighborsPointer].distance >> 8);
		pui8Buffer[i + 5] = (uint8_t)g_NeighborsArray.pNeighbors[ui8NeighborsPointer].distance;
		ui8NeighborsPointer++;
	}
}


