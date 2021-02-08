#ifndef M5_MAZE_MAZE_H_
#define M5_MAZE_MAZE_H_
#include <stm32f4xx_hal.h>

#define M5_MAZE_SIZE 16

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
} m5Cell;

typedef struct {
  int x;
  int y;
} m5Index;

typedef struct {
  m5Cell wall[M5_MAZE_SIZE][M5_MAZE_SIZE];
  uint8_t step_map[M5_MAZE_SIZE][M5_MAZE_SIZE];
  uint8_t dirty;
} m5MazeRecord, *m5Maze;

typedef struct {
  size_t size;
  size_t idx_first;
  size_t idx_last;
  m5Index array[M5_MAZE_SIZE * M5_MAZE_SIZE];
} m5IdxQueueRecord, *m5IdxQueue;

// direction
uint8_t m5cell_is_wall(m5Cell d, uint8_t i);
uint8_t m5cell_num_walls(m5Cell d);

// queue
m5IdxQueue m5idxqueue_constructor();
void m5idxqueue_destructor(m5IdxQueue queue);
void m5idxqueue_clear(m5IdxQueue queue);
uint8_t m5idxqueue_enqueue(m5IdxQueue queue, m5Index value);
m5Index m5idxqueue_dequeue(m5IdxQueue queue);
uint8_t m5idxqueue_is_empty(m5IdxQueue queue);

// maze
m5Maze m5maze_constructor(void);
void m5maze_initialize(m5Maze maze);
void m5maze_load_from_array(m5Maze maze, const char ascii_array[M5_MAZE_SIZE + 1][M5_MAZE_SIZE + 1]);
void m5maze_load_from_file(m5Maze maze, const char *filename);
void m5maze_save_to_file(m5Maze maze, const char *filename);
void m5maze_set_wall(m5Maze maze, m5Index index,
                     const m5Cell *newState);
void m5maze_update_step_map(m5Maze maze, m5Index destination);
void m5maze_print_wall(m5Maze maze, uint8_t values[M5_MAZE_SIZE][M5_MAZE_SIZE]);
void m5maze_print_step_map(m5Maze maze);
#endif