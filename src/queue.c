#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include "read.h"

int message_queue_init(Message_Queue *queue){
  queue->head=0;
  queue->tail=0;
  queue->current=0;
  if(pthread_mutex_init(&queue->mutex,NULL)!=0){
    return -1;
  }
  if(sem_init(&queue->empty,0,QUEUE_CAPACITY)!=0){
    pthread_mutex_destroy(&queue->mutex);
    return -1;
  }
  if(sem_init(&queue->full,0,0)!=0){
    sem_destroy(&queue->empty);
    pthread_mutex_destroy(&queue->mutex);
    return -1;
  }

  return 0;

};

void message_destroy(Message_Queue *queue){
  pthread_mutex_destroy(&queue->mutex);
  sem_destroy(&queue->full);
  sem_destroy(&queue->empty);

};

/*
 * Push is the mirror image of pop.
 *
 * "empty" counts the free slots and "full" counts the queued messages, so
 * waiting on "empty" is what blocks a producer while the queue is full, and
 * posting "full" is what releases a consumer parked in pop.
 *
 * The semaphores do the waiting; the mutex only guards the short critical
 * section that touches head/tail/current, so it is never held across a block.
 * The buffer is the fixed array already declared in read.h and the indices
 * wrap around it, so no slot is ever allocated or copied twice.
 */
int message_queue_push(Message_Queue *queue,const Message *msg){
  if(queue==NULL || msg==NULL){
    return -1;
  }

  sem_wait(&queue->empty);
  pthread_mutex_lock(&queue->mutex);

  queue->buffer[queue->tail]=*msg;
  queue->tail=(queue->tail + 1)%QUEUE_CAPACITY;
  queue->current++;

  pthread_mutex_unlock(&queue->mutex);
  sem_post(&queue->full);

  return 0;
};

int message_queue_pop(Message_Queue *queue,Message *msg){
  if(queue==NULL || msg==NULL){
    return -1;
  }

  sem_wait(&queue->full);
  pthread_mutex_lock(&queue->mutex);

  *msg=queue->buffer[queue->head];
  queue->head=(queue->head +1)%QUEUE_CAPACITY;
  queue->current--;

  pthread_mutex_unlock(&queue->mutex);
  sem_post(&queue->empty);

  return 0;
};
