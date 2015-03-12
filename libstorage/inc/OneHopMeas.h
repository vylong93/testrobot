/*
 * OneHopMeas.h
 *
 *  Created on: Mar 12, 2015
 *      Author: VyLong
 */

#ifndef ONEHOPMEAS_H_
#define ONEHOPMEAS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "libstorage/inc/EnhanceLinkedList.h"
#include "libstorage/inc/RobotMeas.h"

class OneHopMeas
{
public:
	uint32_t firstHopID;
	EnhanceLinkedList<RobotMeas>* pNeighborsTable;

	// Constructor
	OneHopMeas(uint32_t firsthopid = 0, EnhanceLinkedList<RobotMeas>* pTable = NULL);

    // Copy Constructor
	OneHopMeas(const OneHopMeas &rhs);

    // Destructor
    ~OneHopMeas();

    // Operators
    bool operator == (const OneHopMeas &rhs) const { return firstHopID == rhs.firstHopID; }
    bool operator != (const OneHopMeas &rhs) const { return firstHopID != rhs.firstHopID; }
};

OneHopMeas::OneHopMeas(const OneHopMeas &rhs)
{
	firstHopID = rhs.firstHopID;
	pNeighborsTable = new EnhanceLinkedList<RobotMeas>();
	int i;
	for(i = 0; i < rhs.pNeighborsTable->Count; i++)
	{
		RobotMeas value = rhs.pNeighborsTable->ElementAt(i);
		pNeighborsTable->add(value);
	}
}

OneHopMeas::OneHopMeas(uint32_t firsthopid, EnhanceLinkedList<RobotMeas>* pTable)
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
};

///* Test case 2 */
//int i;
//uint8_t* pDynamic = 0;
//
//pDynamic = new uint8_t[700];
//for(i = 0; i < 700; i++)
//	pDynamic[i] = 0xEE;
//delete[] pDynamic;
//
//RobotMeas robot(0, 0);
//EnhanceLinkedList<RobotMeas>* pNeighborsTable;
//OneHopMeas oneHopMeas;
//EnhanceLinkedList<OneHopMeas> OneHopNeighborsTable;
//
//// Node 1
//pNeighborsTable = new EnhanceLinkedList<RobotMeas>();
//robot.ID = 0x11111111; robot.Distance = 0xF11F;
//pNeighborsTable->add(robot); // add Robot 1
//
//robot.ID = 0x22222222; robot.Distance = 0xF22F;
//pNeighborsTable->add(robot); // add Robot 2
//
//robot.ID = 0x33333333; robot.Distance = 0xF33F;
//pNeighborsTable->add(robot); // add Robot 3
//
//robot.ID = 0x44444444; robot.Distance = 0xF44F;
//pNeighborsTable->add(robot); // add Robot 4
//
//oneHopMeas.firstHopID = 0x00123400;
//oneHopMeas.pNeighborsTable = pNeighborsTable;
//OneHopNeighborsTable.add(oneHopMeas);
//delete pNeighborsTable;
//
//// Node 2
//pNeighborsTable = new EnhanceLinkedList<RobotMeas>();
//robot.ID = 0x55555555; robot.Distance = 0xF55F;
//pNeighborsTable->add(robot); // add Robot 5
//
//robot.ID = 0x66666666; robot.Distance = 0xF66F;
//pNeighborsTable->add(robot); // add Robot 6
//
//robot.ID = 0x77777777; robot.Distance = 0xF77F;
//pNeighborsTable->add(robot); // add Robot 7
//
//robot.ID = 0x88888888; robot.Distance = 0xF88F;
//pNeighborsTable->add(robot); // add Robot 8
//
//oneHopMeas.firstHopID = 0x00567800;
//oneHopMeas.pNeighborsTable = pNeighborsTable;
//OneHopNeighborsTable.add(oneHopMeas);
//delete pNeighborsTable;
//
//// Node 3
//pNeighborsTable = new EnhanceLinkedList<RobotMeas>();
//robot.ID = 0x99999999; robot.Distance = 0xF99F;
//pNeighborsTable->add(robot); // add Robot 9
//
//oneHopMeas.firstHopID = 0x00999900;
//oneHopMeas.pNeighborsTable = pNeighborsTable;
//OneHopNeighborsTable.add(oneHopMeas);
//OneHopNeighborsTable.add(oneHopMeas);
//delete pNeighborsTable;
//
//OneHopNeighborsTable.clearAll();
//
//	pDynamic = new uint8_t[700];
//for(i = 0; i < 700; i++)
//	pDynamic[i] = 0xCC;
//delete[] pDynamic;

#endif /* ONEHOPMEAS_H_ */
