/*
 * ringBuffer.c
 *
 * Created: 2/3/2019 3:26:08 PM
 *  Author: klein
 */ 
#include "ringBuffer.h"


void initializeQueue(circularQueue_t *theQueue)
{
	int i;
	theQueue->validItems  =  0;
	theQueue->first       =  0;
	theQueue->last        =  0;
	for(i=0; i<MAX_ITEMS; i++)
	{
		theQueue->data[i] = 0;
	}
	return;
}


int isEmpty(circularQueue_t *theQueue)
{
	if(theQueue->validItems==0)
	return(1);
	else
	return(0);
}

int putItem(circularQueue_t *theQueue, int theItemValue)
{
	if(theQueue->validItems>=MAX_ITEMS)
	{
		return(-1);
	}
	else
	{
		theQueue->validItems++;
		theQueue->data[theQueue->last] = theItemValue;
		theQueue->last = (theQueue->last+1)%MAX_ITEMS;
	}
}

int getItem(circularQueue_t *theQueue, int *theItemValue)
{
	if(isEmpty(theQueue))
	{
		return(-1);
	}
	else
	{
		*theItemValue=theQueue->data[theQueue->first];
		theQueue->first=(theQueue->first+1)%MAX_ITEMS;
		theQueue->validItems--;
		return(0);
	}
}
