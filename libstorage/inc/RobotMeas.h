/*
 * RobotMeas.h
 *
 *  Created on: Mar 10, 2015
 *      Author: VyLong
 */

#ifndef ROBOTMEAS_H_
#define ROBOTMEAS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

class RobotMeas
{
public:
	uint32_t ID;
	uint16_t Distance;

	// Constructor
    RobotMeas(uint32_t id = 0, uint16_t dis = 0) { ID = id; Distance = dis; }

    // Copy Constructor
    RobotMeas(const RobotMeas &rhs) {ID = rhs.ID; Distance = rhs.Distance; }

    // Destructor
    ~RobotMeas() {};

    // Operators
    RobotMeas& operator=(const RobotMeas &rhs) { ID = rhs.ID,  Distance = rhs.Distance; return *this; }
    bool operator == (const RobotMeas &rhs) const { return ID == rhs.ID; }
    bool operator != (const RobotMeas &rhs) const { return ID != rhs.ID; }
};

//	/* Test case */
//
//	int i;
//	uint8_t* pDynamic = 0;
//
//	pDynamic = new uint8_t[100];
//	for(i = 0; i < 100; i++)
//		pDynamic[i] = 0xEE;
//	delete[] pDynamic;
//
//	EnhanceLinkedList<RobotMeas> NeighborsTable;
//	RobotMeas robot(0, 0);
//
//	robot.ID = 0x11111111; robot.Distance = 0xF11F;
//	NeighborsTable.add(robot); // add Robot 1
//
//	robot.ID = 0x22222222; robot.Distance = 0xF22F;
//	NeighborsTable.add(robot); // add Robot 2
//
//	robot.ID = 0x33333333; robot.Distance = 0xF33F;
//	NeighborsTable.add(robot); // add Robot 3
//
//	robot.ID = 0x44444444; robot.Distance = 0xF44F;
//	NeighborsTable.add(robot); // add Robot 4
//
//	robot.ID = 0x55555555; robot.Distance = 0xF55F;
//	NeighborsTable.add(robot); // add Robot 5
//
//	robot.ID = 0x44444444;
//	NeighborsTable.remove(robot); // remove robot 4
//
//	robot.ID = 0x33333333;
//	NeighborsTable.remove(robot); // remove robot 3
//
//	robot.ID = 0x66666666; robot.Distance = 0xF66F;
//	NeighborsTable.add(robot); // add Robot 6
//
//	robot.ID = 0x77777777; robot.Distance = 0xF77F;
//	NeighborsTable.add(robot); // add Robot 7
//
//	robot.ID = 0x88888888; robot.Distance = 0xF88F;
//	NeighborsTable.add(robot); // add Robot 8
//
//	RobotMeas robot_select(0);
//	robot_select = NeighborsTable[4];	// Get element[4] - 7777
//	robot_select.ID = 0x99999999;		// change ID to 0x99999999
//	robot_select = NeighborsTable[5];	// Get element[5] - 88888
//	NeighborsTable[3] = robot_select;	// make element[3] equal to element[5] - 8888
//	NeighborsTable[2].ID = 0x99999999;	// change element[2] 5555 to 9999
//
//	NeighborsTable.remove(NeighborsTable[2]); // remove 9999 (element 2)
//	NeighborsTable.add(NeighborsTable[1]); // add 2222 (element 1)
//
//	NeighborsTable.clearAll();
//
//	pDynamic = new uint8_t[200];
//	for(i = 0; i < 200; i++)
//		pDynamic[i] = 0xCC;
//	delete[] pDynamic;

#endif /* ROBOTMEAS_H_ */
