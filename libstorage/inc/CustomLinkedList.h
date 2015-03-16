/*
 * CustomLinkedList.h
 *
 *  Created on: Mar 14, 2015
 *      Author: VyLong
 */

#ifndef CUSTOMLINKEDLIST_H_
#define CUSTOMLINKEDLIST_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define MAXIMUM_ELEMENT_SIZE	10

// Use of templates.
template <class T>
class CustomLinkedList
{
private:
	T	*ppMap[MAXIMUM_ELEMENT_SIZE];	// An pointer array store the element's pointer

public:
    int Count;	// Indicate current element in the list

   /* Constructor */
    CustomLinkedList();

   /* Destructor */
   ~CustomLinkedList();

   /* Operators */
   //CustomLinkedList& operator=(const CustomLinkedList &rhs) { ppMap = rhs.ppMap,  Count = rhs.Count; return *this; }
   T &operator [](unsigned int i) { return *ppMap[i]; }
   const T &operator [](unsigned int i) const {	return *ppMap[i]; }

   /* Behavivours */
   T& ElementAt(unsigned int i) { return *ppMap[i]; }
   bool add(T value);
   void remove(T value);
   void clearAll();
   int isContain(T value);
};

//*****************************************************
// Constructor allocates the memory used by the list. *
//*****************************************************
template <class T>
CustomLinkedList<T>::CustomLinkedList()
{
	int i;
	for(i = 0; i < MAXIMUM_ELEMENT_SIZE; i++)
	{
		ppMap[i] = NULL;
	}
	Count = 0;
}

//*********************************************
// Adds a new element to the end of the list. *
//*********************************************
template <class T>
bool CustomLinkedList<T>::add(T value)
{
	// (1) check for OverFlow
	if(Count >= MAXIMUM_ELEMENT_SIZE)
		return false;

	// (2) add new value to the map
	ppMap[Count] = new T(value);
	if(ppMap[Count] == NULL)
		return false;

	// (3) update Size
	Count++;
	return true;
}

//***********************************************
// Removes a element from a list. The function  *
// does not assume that the list is sorted.     *
//***********************************************
template <class T>
void CustomLinkedList<T>::remove(T value)
{
	int i, j;
	for(i = 0; i < Count; i++)
	{
		if(*ppMap[i] == value)
		{
			// (1) delete it
			delete ppMap[i];

			// (2) shift up
			for(j = i; j < (Count - 1); j++)
				ppMap[j] = ppMap[j + 1];

			// (3) update size
			Count--;

			// (4) clear the duplicate pointer at the end
			ppMap[Count] = NULL;

			break;
		}
	}
}

//*************************************************
// Removes all element from a list. The function  *
// does not assume that the list is sorted.       *
//*************************************************
template <class T>
void CustomLinkedList<T>::clearAll()
{
	int i;
	for(i = 0; i < Count; i++)
	{
		if(ppMap[i] != NULL)
		{
			delete ppMap[i];
			ppMap[i] = NULL;
		}
	}
	Count = 0;
}

//********************************************
// check if the element existed in the list. *
//********************************************
template <class T>
int CustomLinkedList<T>::isContain(T value)
{
	int i;
	for(i = 0; i < Count; i++)
	{
		if(*ppMap[i] == value)
			return i;
	}
	return (-1);
}

//******************************************************
// Destructor deallocates the memory used by the list. *
//******************************************************
template <class T>
CustomLinkedList<T>::~CustomLinkedList()
{
	this->clearAll();
}


#endif /* CUSTOMLINKEDLIST_H_ */
