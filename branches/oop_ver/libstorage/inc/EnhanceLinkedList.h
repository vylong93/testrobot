/*
 * EnhanceLinkedList.h
 *
 *  Created on: Mar 10, 2015
 *      Author: VyLong
 */

#ifndef ENHANCELINKEDLIST_H_
#define ENHANCELINKEDLIST_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Use of templates.
template <class T>
class EnhanceLinkedList
{
private:
	T** ppMap;	// An pointer array store the element's pointer
    int Size;	// Indicate current element in the list

public:
   //Constructor
   EnhanceLinkedList() { ppMap = NULL; Size = 0; }

   // Destructor
   ~EnhanceLinkedList();

   // Operators
   T &operator [](unsigned int i) { return *ppMap[i]; }
   const T &operator [](unsigned int i) const {	return *ppMap[i]; }

   // Behavivours
   bool add(T value);
   void remove(T value);
   void clearAll();
   T& ElementAt(unsigned int i) { if (i < Size) return *ppMap[i]; else return NULL; }
};

//*********************************************
// Adds a new element to the end of the list. *
//*********************************************
template <class T>
bool EnhanceLinkedList<T>::add(T value)
{
	// (1) keep previous Map
	T** ppOldMap = ppMap;

	// (2) allocate new Map
	ppMap = new T*[Size + 1];
	if(ppMap == NULL)
		return false;

	// (3) copy old Map to new Map
	int i;
	for(i = 0; i < Size; i++)
		ppMap[i] = ppOldMap[i];

	// (4) delete old Map
	delete[] ppOldMap;

	// (5) add new value to the map
	ppMap[Size] = new T(value);
	if(ppMap[Size] == NULL)
		return false;

	// (6) update Size
	Size++;

	return true;
}

//***********************************************
// Removes a element from a list. The function  *
// does not assume that the list is sorted.     *
//***********************************************
template <class T>
void EnhanceLinkedList<T>::remove(T value)
{
	int i;
	for(i = 0; i < Size; i++)
	{
		if(*ppMap[i] == value)
		{
			// (1) delete it
			delete ppMap[i];

			// (2) shift up
			int j;
			for(j = i; j < (Size - 1); j++)
				ppMap[j] = ppMap[j + 1];

			// (3) update Size
			Size--;

			return;
		}
	}
}

//*************************************************
// Removes all element from a list. The function  *
// does not assume that the list is sorted.       *
//*************************************************
template <class T>
void EnhanceLinkedList<T>::clearAll()
{
	int i;
	for(i = 0; i < Size; i++)
	{
		delete ppMap[i];
		ppMap[i] = NULL;
	}
	delete[] ppMap;
	ppMap = NULL;

	Size = 0;
}

//******************************************************
// Destructor deallocates the memory used by the list. *
//******************************************************
template <class T>
EnhanceLinkedList<T>::~EnhanceLinkedList()
{
	this->clearAll();
}


//  /* Test case */
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
//	robot.ID = 0x11111111; robot.distance = 0xF11F;
//	NeighborsTable.add(robot); // add Robot 1
//
//	robot.ID = 0x22222222; robot.distance = 0xF22F;
//	NeighborsTable.add(robot); // add Robot 2
//
//	robot.ID = 0x33333333; robot.distance = 0xF33F;
//	NeighborsTable.add(robot); // add Robot 3
//
//	robot.ID = 0x44444444; robot.distance = 0xF44F;
//	NeighborsTable.add(robot); // add Robot 4
//
//	robot.ID = 0x55555555; robot.distance = 0xF55F;
//	NeighborsTable.add(robot); // add Robot 5
//
//	robot.ID = 0x44444444;
//	NeighborsTable.remove(robot); // remove robot 4
//
//	robot.ID = 0x33333333;
//	NeighborsTable.remove(robot); // remove robot 3
//
//	robot.ID = 0x66666666; robot.distance = 0xF66F;
//	NeighborsTable.add(robot); // add Robot 6
//
//	robot.ID = 0x77777777; robot.distance = 0xF77F;
//	NeighborsTable.add(robot); // add Robot 7
//
//	robot.ID = 0x88888888; robot.distance = 0xF88F;
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
//	pDynamic = new uint8_t[100];
//	for(i = 0; i < 100; i++)
//		pDynamic[i] = 0xCC;
//	delete[] pDynamic;

#endif /* ENHANCELINKEDLIST_H_ */
