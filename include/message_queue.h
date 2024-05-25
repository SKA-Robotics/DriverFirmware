#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#include "roboszpon_message.h"

#define QUEUE_SIZE 32

typedef struct {
    roboszpon_message_t buffer[QUEUE_SIZE];
    unsigned int head;
    unsigned int tail;
} message_queue_t;

void MessageQueue_Init(message_queue_t* queue);

int MessageQueue_IsFull(message_queue_t* queue);

int MessageQueue_IsEmpty(message_queue_t* queue);

// Returns 0 on success or 1 if the queue is already full
int MessageQueue_Enqueue(message_queue_t* queue, roboszpon_message_t data);

// Returns 0 on success or 1 if the queue is empty
int MessageQueue_Dequeue(message_queue_t* queue, roboszpon_message_t* data);

#endif // MESSAGE_QUEUE_H