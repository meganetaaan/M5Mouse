#ifndef M5_COMMON_QUEUE_H_
#define M5_COMMON_QUEUE_H_
#include <stm32f4xx_hal.h>

#define M5_QUEUE_SIZE 256

typedef struct {
  size_t size;
  size_t idx_first;
  size_t idx_last;
  void *array[M5_QUEUE_SIZE];
} m5QueueRecord, *m5Queue;

// queue
m5Queue m5queue_constructor();
void m5queue_destructor(m5Queue queue);
void m5queue_clear(m5Queue queue);
uint8_t m5queue_enqueue(m5Queue queue, void *item);
void *m5queue_dequeue(m5Queue queue);
uint8_t m5queue_is_empty(m5Queue queue);

#endif
