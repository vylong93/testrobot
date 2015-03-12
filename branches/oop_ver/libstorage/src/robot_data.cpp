/*
 * robot_data.c
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/robot_data.h"
#include "libstorage/inc/OneHopMeas.h"

#include "data_manipulation.h"

EnhanceLinkedList<RobotMeas> g_NeighborsTable;
EnhanceLinkedList<OneHopMeas> g_OneHopNeighborsTable;

//location_t locs[LOCATIONS_TABLE_LENGTH];

void initLinkedList(void)
{
	clearNeighborsTable();
	clearOneHopNeighborsTable();
}

void clearNeighborsTable(void)
{
	g_NeighborsTable.clearAll();
}

int getCurrentNeighborsNumber(void)
{
	return g_NeighborsTable.Count;
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

bool isNeighborsTableContainRobot(uint32_t ui32RobotId)
{
	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		if (g_NeighborsTable[i].ID == ui32RobotId)
			return true;
	}
	return false;
}

void fillNeighborsTableToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	if((ui32TotalLength % SIZE_OF_ROBOT_MEAS) != 0)
		return;

	int pointer = 0;
	uint32_t i;
	for(i = 0; i < ui32TotalLength; )
	{
		if(pointer < g_NeighborsTable.Count)
		{
			parse32bitTo4Bytes(&pui8Buffer[i], g_NeighborsTable[pointer].ID);
			parse16bitTo2Bytes(&pui8Buffer[i + 4], g_NeighborsTable[pointer].Distance);

			pointer++;
			i += SIZE_OF_ROBOT_MEAS;
		}
		else
		{
			pui8Buffer[i++] = 0;
		}
	}
}

void clearOneHopNeighborsTable(void)
{
	g_OneHopNeighborsTable.clearAll();
}

int getCurrentOneHopNeighborsNumber(void)
{
	return g_OneHopNeighborsTable.Count;
}

void addToOneHopNeighborsTable(uint32_t ui32NeighborId, uint8_t* pui8TableBuffer, uint32_t ui32TableSizeInByte)
{
	if((ui32TableSizeInByte % SIZE_OF_ROBOT_MEAS) != 0)
		return;

	EnhanceLinkedList<RobotMeas>* pNeighborsTable = new EnhanceLinkedList<RobotMeas>();
	RobotMeas neighbor(0, 0);

	int i;
	for(i = 0; i < ui32TableSizeInByte; i++)
	{
		neighbor.ID = construct4Byte(&pui8TableBuffer[i]);
		neighbor.Distance = construct2Byte(&pui8TableBuffer[i + 4]);
		pNeighborsTable->add(neighbor);
		i += SIZE_OF_ROBOT_MEAS;
	}

	OneHopMeas OneHopMeas(ui32NeighborId, pNeighborsTable);
	g_OneHopNeighborsTable.add(OneHopMeas);
}
