#ifndef M5_MAZE_MAZE_H
#define M5_MAZE_MAZE_H
#include <stm32f4xx_hal.h>

#define MAZE_SIZE 16

typedef union __attribute__((__packed__)) {
  uint8_t byte;
  struct __attribute__((__packed__)) {
    uint8_t North : 1;      // bit0 LSB
    uint8_t East : 1;       // bit1
    uint8_t South : 1;      // bit2
    uint8_t West : 1;       // bit3
    uint8_t DoneNorth : 1;  // bit4
    uint8_t DoneEast : 1;   // bit5
    uint8_t DoneSouth : 1;  // bit6
    uint8_t DoneWest : 1;   // bit7 MSB
  } bits;
} m5Direction;

typedef struct {
  int x;
  int y;
} m5Index;

typedef struct {
  m5Direction wall[MAZE_SIZE][MAZE_SIZE];
  uint8_t step_map[MAZE_SIZE][MAZE_SIZE];
  uint8_t dirty;
} m5MazeRecord, *m5Maze;

typedef struct {
  size_t size;
  size_t idx_first;
  size_t idx_last;
  m5Index array[MAZE_SIZE * MAZE_SIZE];
} m5QueueRecord, *m5Queue;

// direction
uint8_t m5direction_is_wall(m5Direction d, uint8_t i);
uint8_t m5direction_num_walls(m5Direction d);

// queue
m5Queue m5queue_constructor();
void m5queue_destructor(m5Queue queue);
void m5queue_clear(m5Queue queue);
uint8_t m5queue_enqueue(m5Queue queue, m5Index value);
m5Index m5queue_dequeue(m5Queue queue);
uint8_t m5queue_is_empty(m5Queue queue);

// maze
m5Maze m5maze_constructor(void);
void m5maze_initialize(m5Maze maze);
void m5maze_load(m5Maze maze, const char *filename);
void m5maze_save(m5Maze maze, const char *filename);
void m5maze_set_wall(m5Maze maze, m5Index index,
                     const m5Direction *newState);
void m5maze_update_step_map(m5Maze maze, m5Index destination);
void m5maze_print_wall(m5Maze maze, uint8_t values[MAZE_SIZE][MAZE_SIZE]);
void m5maze_print_step_map(m5Maze maze);
#endif