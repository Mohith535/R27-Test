#ifndef READ_H
#define READ_H

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stddef.h>

#define max_size 256
#define QUEUE_CAPACITY 50
/*Incoming message struct 
 * data -> the data sent 
 * length -> the size of the data sent 
*/ 
typedef struct{
  uint8_t data[max_size];
  size_t length;
}Message;

typedef struct{
  uint8_t data[max_size];
  size_t length;
}Shared_Buffer;

typedef struct{
  Message buffer[QUEUE_CAPACITY];
  int head;
  int tail;
  int current;
  pthread_mutex_t mutex;
  sem_t empty;
  sem_t full;

}Message_Queue;

/* reader writer syncronization */ 

typedef struct {
  int reader;
  pthread_mutex_t reader_count;
  pthread_mutex_t writer_count;
  sem_t resource;
}ReadWrite_Lock;

typedef struct {
  int reader_id;
  const char *input_file;
  Message_Queue *queue;
  ReadWrite_Lock *rw_lock;
  int running;
}Reader;

int message_queue_init(Message_Queue *queue);
void message_destroy(Message_Queue *queue);
int message_queue_push(Message_Queue *queue, const Message *msg);
int message_queue_pop(Message_Queue *queue, Message *msg);

int rwlock_init(ReadWrite_Lock *rw);
void rwlock_destroy(ReadWrite_Lock *rw);

void reader_enter(ReadWrite_Lock *lock);
void reader_exit(ReadWrite_Lock *lock);

void writer_enter(ReadWrite_Lock *lock);
void writer_exit(ReadWrite_Lock *lock);

void *reader_thread(void *arg);

#endif



