/*
 * LinkedList.h
 *
 *  Created on: Mar 8, 2015
 *      Author: VyLong
 */

#ifndef LINKEDLIST_H_
#define LINKEDLIST_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Use of templates.
template <class T>
class LinkedList
{
private:
   struct ListNode
    {
      T value;
      ListNode * next;
      ListNode(T value1, ListNode * next1 = NULL)
      {
         value = value1;
         next = next1;
      }
    };

    ListNode * head;           // List head pointer

public:
   LinkedList() { head = NULL;  }    //Constructor
   ~LinkedList();                    // Destructor
   void add(T value);
   void remove(T value);
};

//*********************************************
// Adds a new element to the end of the list. *
//*********************************************
template <class T>
void LinkedList<T>::add(T value)
{
   if (head == NULL)
      head = new ListNode(value);
   else
   {
      // The list is not empty.
      // Use nodePtr to traverse the list
      ListNode * nodePtr = head;
      while (nodePtr->next != NULL)
         nodePtr = nodePtr->next;

      // nodePtr->next is NULL so nodePtr points to the last node.
      // Create a new node and put it after the last node.
      nodePtr->next = new ListNode(value);
    }
}

//**********************************************
// Removes a number from a list. The function  *
// does not assume that the list is sorted.    *
//**********************************************
template <class T>
void LinkedList<T>::remove(T value)
{
   ListNode * nodePtr;
   ListNode *previousNodePtr;

   // If the list is empty, do nothing.
   if (!head)  return;

   // Determine if the first node is the one to delete.
   if (head->value == value)
   {
      nodePtr = head;
      head = head->next;
      delete nodePtr;
   }
   else
   {
      // Initialize nodePtr to the head of the list.
      nodePtr = head;

      // Skip nodes whose value member is not num.
      while (nodePtr != NULL && nodePtr->value != value)
      {
         previousNodePtr = nodePtr;
         nodePtr = nodePtr->next;
      }
      // Link the previous node to the node after
      // nodePtr, then delete nodePtr.
      if (nodePtr)
      {
         previousNodePtr->next = nodePtr->next;
         delete nodePtr;
      }
   }
}

////************************************************
//// displayList outputs a sequence of all values  *
//// currently stored in the list.                 *
////************************************************
//template <class T>
//void LinkedList<T>::displayList()
//{
//   ListNode * nodePtr = head; // Start at head of list
//   while (nodePtr)
//   {
//      // Print the value in the current node
//      cout << nodePtr->value << endl;
//      // Move on to the next node
//      nodePtr = nodePtr->next;
//   }
//}

//******************************************************
// Destructor deallocates the memory used by the list. *
//******************************************************
template <class T>
LinkedList<T>::~LinkedList()
{
   ListNode * nodePtr = head;  // Start at head of list
   while (nodePtr != NULL)
   {
      // garbage keeps track of node to be deleted
      ListNode * garbage = nodePtr;
      // Move on to the next node, if any
      nodePtr = nodePtr->next;
      // Delete the "garbage" node
      delete garbage;
   }
}

#endif /* LINKEDLIST_H_ */
