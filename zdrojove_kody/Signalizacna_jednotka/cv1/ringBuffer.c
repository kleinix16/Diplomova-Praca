/*
 * ringBuffer.c
 *
 * Created: 2/3/2019 3:26:08 PM
 * Author: Tomas Klein
 */ 
#include "ringBuffer.h"

/************************************************************************/
/* Initialization ring buffer - all value to 0                          */
/************************************************************************/
void initializeQueue(circularQueue_t *theQueue)
{
	int i;
	theQueue->validItems  =  0;
	theQueue->first       =  0;
	theQueue->last        =  0;
	for(i=0; i<MAX_ITEMS; i++)   //fill all item to 0
	{
		theQueue->data[i] = 0;
	}
	return;
}

/************************************************************************/
/* Ring buffer - return valid items status                              */
/************************************************************************/
int isEmpty(circularQueue_t *theQueue)
{
	if(theQueue->validItems==0)
	return(1);
	else
	return(0);
}

/************************************************************************/
/* Ring buffer - add item to buffer                                     */
/************************************************************************/
int putItem(circularQueue_t *theQueue, int theItemValue)
{
	if(theQueue->validItems>=MAX_ITEMS)  //Control to overflow buffer
	{
		return(-1);
	}
	else
	{
		theQueue->validItems++;						
		theQueue->data[theQueue->last] = theItemValue;
		theQueue->last = (theQueue->last+1)%MAX_ITEMS;   //control to ring
	}
}


/************************************************************************/
/* Ring buffer - get item from buffer                                   */
/************************************************************************/
int getItem(circularQueue_t *theQueue, int *theItemValue)
{
	if(isEmpty(theQueue))
	{
		return(-1);
	}
	else
	{
		*theItemValue=theQueue->data[theQueue->first];
		theQueue->first=(theQueue->first+1)%MAX_ITEMS; //control to ring
		theQueue->validItems--;
		return(0);
	}
}
