#include <stdio.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>

#include "read.h"

int rwlock_init(ReadWrite_Lock *rw){
    rw->reader = 0;

    if (pthread_mutex_init(&rw->reader_count, NULL) != 0)
        return -1;

    if (pthread_mutex_init(&rw->writer_count, NULL) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        return -1;
    }

    if (sem_init(&rw->resource, 0, 1) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        pthread_mutex_destroy(&rw->writer_count);
        return -1;
    }

    return 0;
}

/*
 * Reader Entry
 *
 * writer_count is used here as a turnstile rather than as a lock over the
 * data: a reader passes straight through it, but a waiting writer holds it,
 * so a steady stream of readers can no longer starve that writer. It is
 * released again immediately, which is what lets readers overlap each other.
 *
 * reader_count guards the counter itself, and only the reader that takes the
 * count from 0 to 1 acquires "resource" on behalf of the whole group. Later
 * readers inherit that acquisition instead of taking the semaphore again,
 * which would deadlock them against each other.
 */
void reader_enter(ReadWrite_Lock *lock){
    pthread_mutex_lock(&lock->writer_count);

    pthread_mutex_lock(&lock->reader_count);

    lock->reader++;

    if (lock->reader == 1) {
        /* First reader in: shut writers out for as long as any reader is
         * active. reader_exit() releases this when the last one leaves. */
        sem_wait(&lock->resource);
    }

    pthread_mutex_unlock(&lock->reader_count);

    pthread_mutex_unlock(&lock->writer_count);
}

/*
 * Reader Exit
 *
 * The last reader out hands "resource" back, which is the only point at which
 * a writer can be admitted. The decrement and the test have to happen under
 * reader_count together, otherwise two readers can both observe zero and post
 * the semaphore twice.
 */
void reader_exit(ReadWrite_Lock *rw){
    pthread_mutex_lock(&rw->reader_count);

    rw->reader--;

    if(rw->reader == 0){
        sem_post(&rw->resource);
    }

    pthread_mutex_unlock(&rw->reader_count);
}

/*
 * Writer Entry
 *
 * Holding writer_count for the whole write both serialises writers against
 * each other and blocks the reader turnstile, so readers queue up behind a
 * writer instead of overtaking it. "resource" is then what actually waits for
 * the readers already inside to drain.
 *
 * Both are taken in the same order as reader_enter() takes them
 * (writer_count -> resource), so the two paths cannot deadlock.
 */
void writer_enter(ReadWrite_Lock *lock){
    pthread_mutex_lock(&lock->writer_count);
    sem_wait(&lock->resource);
}

/*
 * Writer Exit
 *
 * Released in the reverse order of acquisition: the resource first, then the
 * turnstile that readers and other writers are queued on.
 */
void writer_exit(ReadWrite_Lock *lock){
    sem_post(&lock->resource);
    pthread_mutex_unlock(&lock->writer_count);
}

void rwlock_destroy(ReadWrite_Lock *rw)
{
    pthread_mutex_destroy(&rw->reader_count);
    pthread_mutex_destroy(&rw->writer_count);
    sem_destroy(&rw->resource);
}
