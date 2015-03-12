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
   T& ElementAt(unsigned int i) { return *ppMap[i]; }
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


#endif /* ENHANCELINKEDLIST_H_ */
