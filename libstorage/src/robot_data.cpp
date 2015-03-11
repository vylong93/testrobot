/*
 * robot_data.c
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/robot_data.h"
#include "libstorage/inc/EnhanceLinkedList.h"
#include "libstorage/inc/RobotMeas.h"

#include "libprotocol/inc/network.h"

EnhanceLinkedList<RobotMeas> g_NeighborsTable;

//oneHopMeas_t OneHopNeighborsTable[ONEHOP_NEIGHBORS_TABLE_LENGTH];
//location_t locs[LOCATIONS_TABLE_LENGTH];

void initLinkedList(void)
{
	clearNeighborsTable();
}

void clearNeighborsTable(void)
{
	g_NeighborsTable.clearAll();
}

void addOverrideToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		if(g_NeighborsTable[i].ID == ui32NeighborId)
		{
			g_NeighborsTable[i].Distance = (g_NeighborsTable[i].Distance + ui16Distance) >> 1;
			return;
		}
	}

	// breakout if not found any match which ui32NeighborId:
	addToNeighborsTable(ui32NeighborId, ui16Distance);
}

void addToNeighborsTable(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	RobotMeas robotNeighbor(ui32NeighborId, ui16Distance);

	if(g_NeighborsTable.Count < NEIGHBORS_TABLE_LENGTH)
	{
		g_NeighborsTable.add(robotNeighbor);
	}
	else
	{
		/* replaceTheFarthestNeighborInNeighborsTable(ui32NeighborId, ui16Distance); */

		int i, worst_i = 0;
		for(i = 1; i < g_NeighborsTable.Count; i++)	// Search for the worst (max) distance
		{
			if(g_NeighborsTable[worst_i].Distance < g_NeighborsTable[i].Distance)
				worst_i = i;
		}

		g_NeighborsTable[worst_i] = robotNeighbor;
	}
}

void fillNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	int pointer = 0;
	uint32_t i;
	for(i = 0; i < ui32TotalLength; )
	{
		if(pointer < g_NeighborsTable.Count)
		{
			parse32bitTo4Bytes(&pui8Buffer[i], g_NeighborsTable[pointer].ID);
			pui8Buffer[i + 4] = (uint8_t)(g_NeighborsTable[pointer].Distance >> 8);
			pui8Buffer[i + 5] = (uint8_t)(g_NeighborsTable[pointer].Distance);

			pointer++;
			i += SIZE_OF_ROBOT_MEAS;
		}
		else
		{
			pui8Buffer[i++] = 0;
		}
	}
}


