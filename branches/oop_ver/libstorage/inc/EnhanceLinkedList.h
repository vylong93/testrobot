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
//	int Count;	// Indicate current element in the list

public:
    int Count;	// Indicate current element in the list

   /* Constructor */
   EnhanceLinkedList() { ppMap = NULL; Count = 0; }

   /* Destructor */
   ~EnhanceLinkedList();

   /* Operators */
   T &operator [](unsigned int i) { return *ppMap[i]; }
   const T &operator [](unsigned int i) const {	return *ppMap[i]; }

   /* Behavivours */
//   int getLength() { return Count; }
   T& ElementAt(unsigned int i) { if (i < Count) return *ppMap[i]; else return NULL; }
   bool add(T value);
   void remove(T value);
   void clearAll();
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
	ppMap = new T*[Count + 1];
	if(ppMap == NULL)
	{
		// restore old map
		ppMap = ppOldMap;

		return false;
	}

	// (3) copy old Map to new Map
	int i;
	for(i = 0; i < Count; i++)
		ppMap[i] = ppOldMap[i];

	// (4) delete old Map
	delete[] ppOldMap;

	// (5) add new value to the map
	ppMap[Count] = new T(value);
	if(ppMap[Count] == NULL)
		return false;

	// (6) update Size
	Count++;

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
	int j;

	// (0) keep previous Map
	T** ppOldMap = ppMap;

	for(i = 0; i < Count; i++)
	{
		if(*ppMap[i] == value)
		{
			// (1) delete it
			delete ppMap[i];

			// (2) shift up
			for(j = i; j < (Count - 1); j++)
				ppMap[j] = ppMap[j + 1];

			// (2) allocate new Map
			ppMap = new T*[Count - 1];
			if(ppMap == NULL)
			{
				// restore old Map
				ppMap = ppOldMap;

				// update size
				Count--;

				return;
			}

			// (3) copy old Map to new Map
			for(j = 0; j < (Count - 1); j++)
				ppMap[j] = ppOldMap[j];

			// (4) delete old Map
			delete[] ppOldMap;

			// (5) update size
			Count--;

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
	for(i = 0; i < Count; i++)
	{
		delete ppMap[i];
		ppMap[i] = NULL;
	}
	delete[] ppMap;
	ppMap = NULL;

	Count = 0;
}

//******************************************************
// Destructor deallocates the memory used by the list. *
//******************************************************
template <class T>
EnhanceLinkedList<T>::~EnhanceLinkedList()
{
	this->clearAll();
}


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
#endif /* ENHANCELINKEDLIST_H_ */
