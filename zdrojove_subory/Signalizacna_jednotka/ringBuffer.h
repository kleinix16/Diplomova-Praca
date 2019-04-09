/*
 * ringBuffer.h
 *
 * Created: 2/3/2019 3:25:53 PM
 * Author: Tomas Klein
 */ 


#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#define MAX_ITEMS    512
typedef struct circularQueue_s
{
	int     first;
	int     last;
	int     validItems;
	int     data[MAX_ITEMS];
} circularQueue_t;

void initializeQueue(circularQueue_t *theQueue);

int isEmpty(circularQueue_t *theQueue);

int putItem(circularQueue_t *theQueue, int theItemValue);

int getItem(circularQueue_t *theQueue, int *theItemValue);


#endif /* RINGBUFFER_H_ */