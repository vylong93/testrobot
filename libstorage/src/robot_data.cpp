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
	NeighborsTable_clear();
	OneHopNeighborsTable_clear();
}

void NeighborsTable_clear(void)
{
	g_NeighborsTable.clearAll();
}

int NeighborsTable_getSize(void)
{
	return g_NeighborsTable.Count;
}

void NeighborsTable_addOverride(uint32_t ui32NeighborId, uint16_t ui16Distance)
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
	NeighborsTable_add(ui32NeighborId, ui16Distance);
}

void NeighborsTable_add(uint32_t ui32NeighborId, uint16_t ui16Distance)
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

bool NeighborsTable_isContainRobot(uint32_t ui32RobotId)
{
	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		if (g_NeighborsTable[i].ID == ui32RobotId)
			return true;
	}
	return false;
}

uint32_t NeighborsTable_getIdAtIndex(uint32_t ui32Index)
{
	return g_NeighborsTable[ui32Index].ID;
}

void NeighborsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
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

void OneHopNeighborsTable_clear(void)
{
	g_OneHopNeighborsTable.clearAll();
}

int OneHopNeighborsTable_getSize(void)
{
	return g_OneHopNeighborsTable.Count;
}

void OneHopNeighborsTable_add(uint32_t ui32NeighborId, uint8_t* pui8TableBuffer, uint32_t ui32TableSizeInByte)
{
	if((ui32TableSizeInByte % SIZE_OF_ROBOT_MEAS) != 0)
		return;

	OneHopMeas oneHopMeas;
	EnhanceLinkedList<RobotMeas> NeighborsTable;
	RobotMeas neighbor(0);

	int i;
	for(i = 0; i < ui32TableSizeInByte; )
	{
		neighbor.ID = construct4Byte(&pui8TableBuffer[i]);
		neighbor.Distance = construct2Byte(&pui8TableBuffer[i + 4]);
		NeighborsTable.add(neighbor);
		i += SIZE_OF_ROBOT_MEAS;
	}

	oneHopMeas.firstHopID = ui32NeighborId;
	oneHopMeas.pNeighborsTable = &NeighborsTable;
	g_OneHopNeighborsTable.add(oneHopMeas);
}

bool OneHopNeighborsTable_isContainRobot(uint32_t ui32RobotId)
{
	int i;
	for(i = 0; i < g_OneHopNeighborsTable.Count; i++)
	{
		if (g_OneHopNeighborsTable[i].firstHopID == ui32RobotId)
			return true;
	}
	return false;
}

void OneHopNeighborsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	if((ui32TotalLength % MAXIMUM_SIZE_OF_ONEHOP_MEAS) != 0)
		return;

	int neighborPointer;
	int oneHopNeighborPointer = 0;

	uint32_t i;
	for(i = 0; i < ui32TotalLength; )
	{
		if(oneHopNeighborPointer < g_OneHopNeighborsTable.Count)
		{
			parse32bitTo4Bytes(&pui8Buffer[i], g_OneHopNeighborsTable[oneHopNeighborPointer].firstHopID);
			i += 4;

			for(neighborPointer = 0; neighborPointer < NEIGHBORS_TABLE_LENGTH; neighborPointer++)
			{
				if(neighborPointer < g_OneHopNeighborsTable[oneHopNeighborPointer].pNeighborsTable->Count)
				{
					parse32bitTo4Bytes(&pui8Buffer[i], g_OneHopNeighborsTable[oneHopNeighborPointer].pNeighborsTable->ElementAt(neighborPointer).ID);
					i += 4;

					parse16bitTo2Bytes(&pui8Buffer[i], g_OneHopNeighborsTable[oneHopNeighborPointer].pNeighborsTable->ElementAt(neighborPointer).Distance);
					i += 2;
				}
				else
				{
					parse32bitTo4Bytes(&pui8Buffer[i], 0);
					i += 4;

					parse16bitTo2Bytes(&pui8Buffer[i], 0);
					i += 2;
				}
			}

			oneHopNeighborPointer++;
		}
		else
		{
			pui8Buffer[i++] = 0;
		}
	}
}
