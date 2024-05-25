#include "message_queue.h"

void MessageQueue_Init(message_queue_t* queue) {
    queue->head = 0;
    queue->tail = 0;
}

int MessageQueue_IsFull(message_queue_t* queue) {
    return ((queue->head + 1) % QUEUE_SIZE) == queue->tail;
}

int MessageQueue_IsEmpty(message_queue_t* queue) {
    return queue->head == queue->tail;
}

// Returns 0 on success or 1 if the queue is already full
int MessageQueue_Enqueue(message_queue_t* queue, roboszpon_message_t data) {
    if (MessageQueue_IsFull(queue)) {
        return 1;
    }
    queue->buffer[queue->head] = data;
    queue->head = (queue->head + 1) % QUEUE_SIZE;
    return 0;
}

// Returns 0 on success or 1 if the queue is empty
int MessageQueue_Dequeue(message_queue_t* queue, roboszpon_message_t* data) {
    if (MessageQueue_IsEmpty(queue)) {
        return 1;
    }
    *data = queue->buffer[queue->tail];
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    return 0;
}