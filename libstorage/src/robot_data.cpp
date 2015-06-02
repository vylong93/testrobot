/*
 * robot_data.c
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/robot_data.h"
#include "libstorage/inc/CustomLinkedList.h"
#include "libstorage/inc/RobotMeas.h"
#include "libstorage/inc/OneHopMeas.h"
#include "libstorage/inc/RobotLocation.h"
#include "libmath/inc/custom_math.h"
#include "data_manipulation.h"

#include "libalgorithm/inc/GradientDescentNode.h"

#include "libcustom/inc/custom_uart_debug.h"

CustomLinkedList<RobotMeas> g_NeighborsTable;
CustomLinkedList<OneHopMeas> g_OneHopNeighborsTable;
CustomLinkedList<RobotLocation> g_RobotLocationsTable;

void initLinkedList(void)
{
	NeighborsTable_clear();
	OneHopNeighborsTable_clear();
	RobotLocationsTable_clear();
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

void NeighborsTable_remove(uint32_t ui32NeighborId)
{
	RobotMeas robotMeas(ui32NeighborId);

	g_NeighborsTable.remove(robotMeas);
}

void NeighborsTable_updateNewDistanceForNeighbor(uint32_t ui32NeighborId, uint16_t ui16Distance)
{
	int i;
	for(i = 0; i < g_NeighborsTable.Count; i++)
	{
		if(g_NeighborsTable[i].ID == ui32NeighborId)
		{
			g_NeighborsTable[i].Distance = ui16Distance;
			return;
		}
	}

	// breakout if not found any match which ui32NeighborId:
	NeighborsTable_add(ui32NeighborId, ui16Distance);
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

bool NeighborsTable_getDistanceOfRobot(uint32_t ui32RobotID, uint16_t *pui16Distance)
{
	RobotMeas target(ui32RobotID);
	int targetIndex = g_NeighborsTable.isContain(target);
	if(targetIndex < 0)
		return false;
	*pui16Distance = g_NeighborsTable[targetIndex].Distance;
	return true;
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
	CustomLinkedList<RobotMeas> NeighborsTable;
	RobotMeas neighbor(0);

	uint32_t i;
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

uint32_t OneHopNeighborsTable_getFirstHopIdAtIndex(uint32_t ui32Index)
{
	return g_OneHopNeighborsTable[ui32Index].firstHopID;
}

int OneHopNeighborsTable_getIndexOfRobot(uint32_t ui32RobotID)
{
	OneHopMeas target(ui32RobotID, NULL);
	return g_OneHopNeighborsTable.isContain(target);
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

void RobotLocationsTable_clear(void)
{
	g_RobotLocationsTable.clearAll();
}

int RobotLocationsTable_getSize(void)
{
	return g_RobotLocationsTable.Count;
}

void RobotLocationsTable_add(uint32_t id, float x, float y)
{
	RobotLocation robotLocation(id, x, y);

	g_RobotLocationsTable.add(robotLocation);
}

void RobotLocationsTable_remove(uint32_t id)
{
	RobotLocation robotLocation(id, 0);

	g_RobotLocationsTable.remove(robotLocation);
}

void RobotLocationsTable_updateLocation(uint32_t id, float x, float y)
{
	RobotLocation robotLocation(id, 0);

	g_RobotLocationsTable.remove(robotLocation);

	robotLocation.vector.x = x;
	robotLocation.vector.y = y;

	g_RobotLocationsTable.add(robotLocation);
}

int RobotLocationsTable_getIndexOfRobot(uint32_t ui32RobotID)
{
	RobotLocation target(ui32RobotID, NULL);
	return g_RobotLocationsTable.isContain(target);
}

bool RobotLocationsTable_isContainRobot(uint32_t ui32RobotId)
{
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if (g_RobotLocationsTable[i].ID == ui32RobotId)
			return true;
	}
	return false;
}

bool RobotLocationsTable_getVectorOfRobot(uint32_t ui32RobotId, float* pfX, float *pfY)
{
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if (g_RobotLocationsTable[i].ID == ui32RobotId)
		{
			*pfX = g_RobotLocationsTable[i].vector.x;
			*pfY = g_RobotLocationsTable[i].vector.y;
			return true;
		}
	}
	return false;
}

uint32_t RobotLocationsTable_getIdAtIndex(uint32_t ui32Index)
{
	return g_RobotLocationsTable[ui32Index].ID;
}

void RobotLocationsTable_getLocationAtIndex(uint32_t ui32Index, float *pfX, float *pfY)
{
	*pfX = g_RobotLocationsTable[ui32Index].vector.x;
	*pfY = g_RobotLocationsTable[ui32Index].vector.y;
}

bool RobotLocationTable_setVectorOfRobot(uint32_t ui32RobotId, float fX, float fY)
{
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if (g_RobotLocationsTable[i].ID == ui32RobotId)
		{
			g_RobotLocationsTable[i].vector.x = fX;
			g_RobotLocationsTable[i].vector.y = fY;
			return true;
		}
	}
	return false;
}

void RobotLocationsTable_rotate(float fAngle, bool bFlipXaxis)
{
	float x, y;

	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		x = g_RobotLocationsTable[i].vector.x;
		y = g_RobotLocationsTable[i].vector.y;

		g_RobotLocationsTable[i].vector.x = x * cosf(fAngle) - y * sinf(fAngle);
		g_RobotLocationsTable[i].vector.y = x * sinf(fAngle) + y * cosf(fAngle);

		if (bFlipXaxis)
			g_RobotLocationsTable[i].vector.x *= -1;
	}
}

void RobotLocationsTable_transformToWorldFrame(RobotIdentity_t* pRobotIdentity)
{
	Vector2<float> vectRotationHop(pRobotIdentity->RotationHop_x, pRobotIdentity->RotationHop_y);
	Vector2<float> vectTransform;

	RobotLocation target(pRobotIdentity->RotationHop_ID, 0);
	int i = g_RobotLocationsTable.isContain(target);
	if (i >= 0)
	{
		vectTransform = vectRotationHop - g_RobotLocationsTable[i].vector;
		RobotLocationsTable_linearTransform(vectTransform.x, vectTransform.y, pRobotIdentity->Origin_ID);
	}
}

void RobotLocationsTable_linearTransform(float dx, float dy, uint32_t ui32OriginId)
{
	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
//		if(g_RobotLocationsTable[i].ID == ui32OriginId)
//		{
//			g_RobotLocationsTable[i].vector.x = 0;
//			g_RobotLocationsTable[i].vector.y = 0;
//		}
//		else
//		{
			g_RobotLocationsTable[i].vector.x += dx;
			g_RobotLocationsTable[i].vector.y += dy;
//		}
	}
}

void RobotLocationsTable_fillContentToByteBuffer(uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	if((ui32TotalLength % SIZE_OF_ROBOT_LOCATION) != 0)
		return;

	int32_t i32FixedPoint16_16;

	int pointer = 0;
	uint32_t i;
	for(i = 0; i < ui32TotalLength; )
	{
		if(pointer < g_RobotLocationsTable.Count)
		{
			parse32bitTo4Bytes(&pui8Buffer[i], g_RobotLocationsTable[pointer].ID);

			i32FixedPoint16_16 = (int32_t)(g_RobotLocationsTable[pointer].vector.x * 65536);
			parse32bitTo4Bytes(&pui8Buffer[i + 4], i32FixedPoint16_16);

			i32FixedPoint16_16 = (int32_t)(g_RobotLocationsTable[pointer].vector.y * 65536);
			parse32bitTo4Bytes(&pui8Buffer[i + 8], i32FixedPoint16_16);

			pointer++;
			i += SIZE_OF_ROBOT_LOCATION;
		}
		else
		{
			pui8Buffer[i++] = 0;
		}
	}
}

void RobotLocationsTable_fillContentToByteBufferOffsetLocal(uint32_t ui32LocalOriginId, uint8_t* pui8Buffer, uint32_t ui32TotalLength)
{
	if((ui32TotalLength % SIZE_OF_ROBOT_LOCATION) != 0)
		return;

	RobotLocation localOrigin(ui32LocalOriginId, 0);
	int indexOfLocalOrigin = g_RobotLocationsTable.isContain(localOrigin);
	if(indexOfLocalOrigin < 0)
		return;
	Vector2<float> vectorOfLocalOrigin = g_RobotLocationsTable[indexOfLocalOrigin].vector;

	int32_t i32FixedPoint16_16;

	int pointer = 0;
	uint32_t i;
	for(i = 0; i < ui32TotalLength; )
	{
		if(pointer < g_RobotLocationsTable.Count)
		{
			parse32bitTo4Bytes(&pui8Buffer[i], g_RobotLocationsTable[pointer].ID);

			i32FixedPoint16_16 = (int32_t)((g_RobotLocationsTable[pointer].vector.x  - vectorOfLocalOrigin.x) * 65536);
			parse32bitTo4Bytes(&pui8Buffer[i + 4], i32FixedPoint16_16);

			i32FixedPoint16_16 = (int32_t)((g_RobotLocationsTable[pointer].vector.y  - vectorOfLocalOrigin.y) * 65536);
			parse32bitTo4Bytes(&pui8Buffer[i + 8], i32FixedPoint16_16);

			pointer++;
			i += SIZE_OF_ROBOT_LOCATION;
		}
		else
		{
			pui8Buffer[i++] = 0;
		}
	}
}

void RobotLocationsTable_selfCorrectByGradientDescent(uint32_t ui32OriginalID, uint32_t ui32RotationHopID)
{
	DEBUG_PRINT("Entering RobotLocationsTable_selfCorrectByGradientDescent........\n");

	bool isGradienttSearchContinue = true;
	GradientDescentNode* pTarget = NULL;
	CustomLinkedList<GradientDescentNode> listCorrectLocationAlgorithm;

	int i;
	for(i = 0; i < g_RobotLocationsTable.Count; i++)
	{
		if(g_RobotLocationsTable[i].ID == ui32OriginalID || g_RobotLocationsTable[i].ID == ui32RotationHopID)
			continue;
		pTarget = new GradientDescentNode;
		pTarget->init(&g_RobotLocationsTable[i]);
		listCorrectLocationAlgorithm.add(*pTarget);
	}

	// active Gradient descent to calculate new position
	while(isGradienttSearchContinue)
	{
		isGradienttSearchContinue = false;

		for(i = 0; i < listCorrectLocationAlgorithm.Count; i++)
		{
			listCorrectLocationAlgorithm[i].run();
		}

		for(i = 0; i < listCorrectLocationAlgorithm.Count; i++)
		{
			if(listCorrectLocationAlgorithm[i].isGradientSearchStop == false)
			{
				isGradienttSearchContinue = true;
				break;
			}
		}
	}

	//listCorrectLocationAlgorithm.clearAll(); // optimzied

	DEBUG_PRINT("........Returning from RobotLocationsTable_selfCorrectByGradientDescent\n");
}
