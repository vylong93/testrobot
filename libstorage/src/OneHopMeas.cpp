/*
 * OneHopMeas.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: VyLong
 */

#include "libstorage/inc/OneHopMeas.h"

OneHopMeas::OneHopMeas(const OneHopMeas &rhs)
{
	firstHopID = rhs.firstHopID;

	if(rhs.pNeighborsTable != NULL)
	{
		pNeighborsTable = new CustomLinkedList<RobotMeas>();
		int i;
		for(i = 0; i < rhs.pNeighborsTable->Count; i++)
		{
			pNeighborsTable->add(rhs.pNeighborsTable->ElementAt(i));
		}
	}
	else
	{
		pNeighborsTable = NULL;
	}
}

OneHopMeas::OneHopMeas(uint32_t firsthopid, CustomLinkedList<RobotMeas>* pTable)
{
	firstHopID = firsthopid;
	pNeighborsTable = pTable;
}

OneHopMeas::~OneHopMeas()
{
	if(pNeighborsTable != NULL)
	{
		pNeighborsTable->clearAll();
		delete pNeighborsTable;
	}
}


