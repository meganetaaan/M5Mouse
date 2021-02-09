#include "common/queue.h"
#include <stdlib.h>

m5Queue m5queue_constructor() {
  m5Queue queue = malloc(sizeof(m5QueueRecord));
  queue->size = M5_QUEUE_SIZE;
  queue->idx_first = queue->idx_last = 0;
  return queue;
}
void m5queue_destructor(m5Queue queue) {
  m5queue_clear(queue);
  free(queue);
}

void m5queue_clear(m5Queue queue) {
  while (!m5queue_is_empty(queue)) {
    free(m5queue_dequeue(queue));
  }
  queue->idx_first = queue->idx_last;
}

uint8_t m5queue_enqueue(m5Queue queue, void* value) {
  size_t idx_next = (queue->idx_last + 1) % queue->size;
  if (queue->idx_first == idx_next) {
    // queue full
    return -1;
  }
  queue->array[queue->idx_last] = value;
  queue->idx_last = idx_next;
  return 0;
};

void* m5queue_dequeue(m5Queue queue) {
  if (m5queue_is_empty(queue)) {
    // queue empty
    return NULL;
  }
  void* ptr = queue->array[queue->idx_first];
  queue->array[queue->idx_first] = NULL;
  queue->idx_first = (queue->idx_first + 1) % queue->size;
  return ptr;
};
uint8_t m5queue_is_empty(m5Queue queue) {
  return queue->idx_first == queue->idx_last;
};