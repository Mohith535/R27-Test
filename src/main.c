#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <math.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include "read.h"
#include "en_dc.h"
#include "read_file.h"
#include "drive.h"
#define NUM_PRODUCERS 1
#define NUM_CONSUMERS 3

/* One coordinate on the wire is two floats: latitude then longitude. */
#define PAYLOAD_BYTES (2u * sizeof(float))

Message_Queue queue;
Shared_Buffer shared_buffer;
ReadWrite_Lock lock;
pthread_mutex_t message_mutex;
pthread_cond_t message_available;
unsigned long message_generation=0;
int producer_finished=0;

/*
 * The producer publishes into a single shared buffer, so a message is only
 * safe to overwrite once a consumer has taken it. Three counters, all guarded
 * by message_mutex, express that hand-off:
 *
 *   message_generation   incremented by the producer once a message is in the
 *                        shared buffer and ready to be picked up;
 *   claimed_generation   raised by the one consumer that takes ownership of
 *                        that message, so the other two leave it alone;
 *   consumed_generation  raised by that same consumer once it has finished
 *                        reading the buffer, which is what frees the producer
 *                        to write the next message.
 *
 * Claiming is what stops the same coordinate being queued three times, and
 * waiting for consumed_generation is what keeps the results in input order.
 */
static unsigned long claimed_generation=0;
static unsigned long consumed_generation=0;
static int end_of_stream_pushed=0;

/*
 * A zero-length message is the end-of-stream marker for the drive thread. A
 * real coordinate always encodes to at least one byte, so the two can never be
 * confused.
 */
static int message_is_end_of_stream(const Message *msg)
{
    return msg->length == 0u;
}

void *producer(void *arg)
{
    InputFile input;
    FileArgs *args = (FileArgs *)arg;

    if (input_file_open(&input, args->filename) != 0) {
        /* Nothing will ever be published, so release the consumers instead of
         * leaving them parked on the condition variable. */
        pthread_mutex_lock(&message_mutex);
        producer_finished = 1;
        pthread_cond_broadcast(&message_available);
        pthread_mutex_unlock(&message_mutex);
        return NULL;
    }

    float x_coord;
    float y_coord;

    while (input_file_read(&input, &x_coord, &y_coord)) {

        Message msg = {0};

        /*
         * Convert the coordinates into a transport message.
         *
         * The two floats are laid out as raw bytes and then framed, so the
         * frame carries no 0x00 of its own and a zero byte stays usable as a
         * delimiter. A coordinate of 0.0 is eight zero bytes, which is exactly
         * the case the framing exists to handle.
         */
        uint8_t payload[PAYLOAD_BYTES];

        memcpy(payload, &x_coord, sizeof x_coord);
        memcpy(payload + sizeof x_coord, &y_coord, sizeof y_coord);

        encode_result encoded =
            frame_encode(msg.data, sizeof msg.data, payload, sizeof payload);

        if (encoded.status != ENCODE_OK) {
            printf("Producer %d: encode failed (status 0x%02X)\n", args->id,
                   (unsigned)encoded.status);
            continue;
        }

        msg.length = encoded.out_len;

        /*
         * Store the message in the shared buffer safely.
         *
         * The writer side of the reader-writer lock gives exclusive access, so
         * no consumer can read a half-written buffer.
         */
        writer_enter(&lock);
        memcpy(shared_buffer.data, msg.data, msg.length);
        shared_buffer.length = msg.length;
        writer_exit(&lock);

        /*
         * Notify waiting consumers, then wait until exactly one of them has
         * taken this message before reusing the buffer. Without that wait the
         * next coordinate could overwrite this one before it was read.
         */
        pthread_mutex_lock(&message_mutex);
        message_generation++;
        pthread_cond_broadcast(&message_available);
        while (consumed_generation != message_generation) {
            pthread_cond_wait(&message_available, &message_mutex);
        }
        pthread_mutex_unlock(&message_mutex);
    }

    input_file_close(&input);

    /*
     * Notify consumers that production has finished so they can drain and
     * exit rather than blocking forever.
     */
    pthread_mutex_lock(&message_mutex);
    producer_finished = 1;
    pthread_cond_broadcast(&message_available);
    pthread_mutex_unlock(&message_mutex);

    return NULL;
}

void *consumer(void *arg)
{
    int id = *(int *)arg;

    for (;;) {
        Message encoded = {0};
        Message decoded = {0};
        unsigned long my_generation;

        /*
         * Wait for a new message.
         *
         * The predicate is "a generation exists that nobody has claimed", so a
         * spurious or broadcast wake-up simply goes back to waiting.
         */
        pthread_mutex_lock(&message_mutex);
        while ((claimed_generation == message_generation) && !producer_finished) {
            pthread_cond_wait(&message_available, &message_mutex);
        }

        if (claimed_generation == message_generation) {
            /* Producer is done and nothing is outstanding. The first consumer
             * to notice posts a single end-of-stream marker for the drive
             * thread; all three then exit. */
            if (!end_of_stream_pushed) {
                Message end_of_stream = {0};

                end_of_stream_pushed = 1;
                pthread_mutex_unlock(&message_mutex);
                message_queue_push(&queue, &end_of_stream);
                pthread_mutex_lock(&message_mutex);
            }
            pthread_mutex_unlock(&message_mutex);
            break;
        }

        /* Claim it, so the other consumers skip this generation. */
        claimed_generation = message_generation;
        my_generation = claimed_generation;
        pthread_mutex_unlock(&message_mutex);

        /*
         * Safely retrieve the message from the shared buffer, on the reader
         * side of the lock.
         */
        reader_enter(&lock);
        encoded.length = shared_buffer.length;
        if (encoded.length > sizeof encoded.data) {
            encoded.length = sizeof encoded.data;
        }
        memcpy(encoded.data, shared_buffer.data, encoded.length);
        reader_exit(&lock);

        /*
         * Decode the message.
         */
        decode_result result = frame_decode(decoded.data, sizeof decoded.data,
                                            encoded.data, encoded.length);
        decoded.length = result.out_len;

        /*
         * Forward the message to the drive queue. Only the consumer that
         * claimed this generation gets here, so each coordinate is queued
         * exactly once and the queue stays in input order.
         */
        if ((result.status == DECODE_OK) && (decoded.length == PAYLOAD_BYTES)) {
            message_queue_push(&queue, &decoded);
        } else {
            printf("Consumer %d: decode failed (status 0x%02X, %lu bytes)\n", id,
                   (unsigned)result.status, (unsigned long)decoded.length);
        }

        /* Release the producer to publish the next coordinate. */
        pthread_mutex_lock(&message_mutex);
        consumed_generation = my_generation;
        pthread_cond_broadcast(&message_available);
        pthread_mutex_unlock(&message_mutex);
    }

    return NULL;
}

void *drive_write(void *arg){
  InputFile input;
  FileArgs *args=(FileArgs *) arg;
  int failed=0;
  int written=0;

  if (input_file_open_write(&input, args->result_filename) != 0) {
    printf("Failed to open %s\n", args->result_filename);
    return NULL;
  }

  for(;;){
    Message msg;
    /*
     * Receive the coordinate message. The pop blocks on the queue's semaphore,
     * so this thread costs nothing while it waits.
     */
    if (message_queue_pop(&queue, &msg) != 0) {
      break;
    }
    if (message_is_end_of_stream(&msg)) {
      break;
    }
    if (msg.length != PAYLOAD_BYTES) {
      printf("Drive %d: unexpected payload of %lu bytes\n", args->id,
             (unsigned long)msg.length);
      continue;
    }

    /* Construct the target coordinate. */
    float latitude;
    float longitude;

    memcpy(&latitude, msg.data, sizeof latitude);
    memcpy(&longitude, msg.data + sizeof latitude, sizeof longitude);

    struct coordinate coordinate_target = {latitude, longitude, 0.0f};

    /* Maintain the rover state. Every coordinate is driven from the same
     * origin and heading, so each line of the result file is independent. */
    struct rover_state rover = {{0.0f, 0.0f, 0.0f}, 0.0f};

    /* Invoke the drive_to_target function in drive.c. */
    enum drive_status result_status = drive_to_target(&rover, &coordinate_target);

    float dx=coordinate_target.latitude-rover.position.latitude;
    float dy=coordinate_target.longitude-rover.position.longitude;
    float error =hypotf(dx,dy);
    int status=1;
    if (result_status==DRIVE_REACHED_TARGET && error<=TARGET_TOLERANCE){
      status=0;
    }

    input_file_write(&input,&rover.position.latitude,&rover.position.longitude,&error,&status);
    written++;

    if (status != 0) {
      failed=1;
      break;
    }
  }

  input_file_close(&input);

  if(failed==0 && written>0){
      printf("Success \n");
  }else {
      printf("Failed try again \n");
  }
  return NULL;
}


int main(){
  pthread_t producers[NUM_PRODUCERS];
  pthread_t consumers[NUM_CONSUMERS];
  pthread_t drive_writers[NUM_PRODUCERS];
  int consumer_id[NUM_CONSUMERS]={1,2,3};
  const char *testcases[]={
    "input/testcase1.txt",
    "input/testcase2.txt",
    "input/testcase3.txt",
    "input/testcase4.txt"
  };
  const char *result_tc[]={
    "result/result1.txt",
    "result/result2.txt",
    "result/result3.txt",
    "result/result4.txt"
  };
  if(rwlock_init(&lock) !=0 ){
    printf("Reader writer synchrnization failed \n");
    return 1;
  }
  if(message_queue_init(&queue)!=0){
    printf("Queue Initialization failed \n");
    return 1;
  }
  if(pthread_mutex_init(&message_mutex,NULL)!=0){
    printf("Message mutex Initialization failed\n");
    return 1;
  }
  if(pthread_cond_init(&message_available,NULL)!=0){
    printf("Condition mutex Initialization failed\n");
    return 1;

  }
  for(int i=0;i<4;i++){
    printf("Input : %d \n",i+1);
    printf("\n");
    printf("\n");
    FileArgs file_args={
      .id=1,
      .filename=testcases[i],
      .result_filename=result_tc[i]
    };
    /* Every testcase starts from a clean hand-off state. */
    message_generation=0;
    producer_finished=0;
    claimed_generation=0;
    consumed_generation=0;
    end_of_stream_pushed=0;

    for(int p=0;p<NUM_PRODUCERS;p++){
      pthread_create(&producers[p],NULL,producer,&file_args);
    };
    for(int c=0;c<NUM_CONSUMERS;c++){
      pthread_create(&consumers[c],NULL,consumer,&consumer_id[c]);

    };

    for(int w = 0; w < NUM_PRODUCERS; w++) {
      pthread_create(
        &drive_writers[w],
        NULL,
        drive_write,
        &file_args
      );
    };
    for(int p=0;p<NUM_PRODUCERS;p++){
      pthread_join(producers[p],NULL);
    };
    for(int c=0;c<NUM_CONSUMERS;c++){
      pthread_join(consumers[c],NULL);
    };
    for (int w = 0; w < NUM_PRODUCERS; w++) {
      pthread_join(drive_writers[w], NULL);
    };


  };
  pthread_cond_destroy(&message_available);
  pthread_mutex_destroy(&message_mutex);
  rwlock_destroy(&lock);
  message_destroy(&queue);

  return 0;
}
