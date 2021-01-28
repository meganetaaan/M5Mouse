#include "maze/maze.h"
#include <stdlib.h>
#include <stdio.h>

const uint8_t M5_NORTH = 0x01;
const uint8_t M5_EAST = 0x02;
const uint8_t M5_SOUTH = 0x04;
const uint8_t M5_WEST = 0x08;
const uint8_t M5_DONE_NORTH = 0x10;
const uint8_t M5_DONE_EAST = 0x20;
const uint8_t M5_DONE_SOUTH = 0x40;
const uint8_t M5_DONE_WEST = 0x80;
#define M5_VEC_NORTH \
  { 0, 1 }
#define M5_VEC_EAST \
  { 1, 0 }
#define M5_VEC_SOUTH \
  { 0, -1 }
#define M5_VEC_WEST \
  { -1, 0 }
const m5Index M5_DIRECTIONS[4] = {M5_VEC_NORTH, M5_VEC_EAST, M5_VEC_SOUTH, M5_VEC_WEST};

uint8_t m5Cell_is_wall(m5Cell d, uint8_t i) {
  return d.byte & (0x01 << i);
}
uint8_t m5Cell_num_walls(m5Cell d) {
  return d.bits.North + d.bits.East + d.bits.South + d.bits.West;
}

m5Maze m5maze_constructor() {
  m5Maze maze = malloc(sizeof(m5MazeRecord));
  m5maze_initialize(maze);
  return maze;
}

void m5maze_initialize(m5Maze maze) {
  for (size_t i = 0; i < M5_MAZE_SIZE; i++) {
    for (size_t j = 0; j < M5_MAZE_SIZE; j++) {
      maze->wall[i][j].byte = 0x00;
      maze->step_map[i][j] = 0x00;
    }
  }
  for (size_t i = 0; i < M5_MAZE_SIZE; i++) {
    maze->wall[M5_MAZE_SIZE - 1][i].byte |= M5_NORTH | M5_DONE_NORTH;
    maze->wall[i][M5_MAZE_SIZE - 1].byte |= M5_EAST | M5_DONE_EAST;
    maze->wall[0][i].byte |= M5_SOUTH | M5_DONE_SOUTH;
    maze->wall[i][0].byte |= M5_WEST | M5_DONE_WEST;
  }
  maze->dirty = 1;
}

void m5maze_load(m5Maze maze, const char *filename) {
  // TODO: load
}

void m5maze_save(m5Maze maze, const char *filename) {
  // TODO: save
}

void m5maze_print_step_map(m5Maze maze) {
  m5maze_print_wall(maze, maze->step_map);
}

void m5maze_print_wall(m5Maze maze, uint8_t values[M5_MAZE_SIZE][M5_MAZE_SIZE]) {
  uint8_t has_value = 0;
  if (values) {
    has_value = 1;
  }
  for (int y = M5_MAZE_SIZE - 1; y >= 0; y--) {
    // north side walls
    for (int x = 0; x < M5_MAZE_SIZE; x++) {
      printf("+");
      if (maze->wall[y][x].bits.North) {
        printf("----");
      } else {
        printf("    ");
      }
    }
    printf("+\n");
    // west side walls
    for (int x = 0; x < M5_MAZE_SIZE; x++) {
      if (maze->wall[y][x].bits.West) {
        printf("|");
      } else {
        printf(" ");
      }
      if (has_value) {
        printf("%3u ", values[y][x]);
      } else {
        printf("    ");
      }
    }
    printf("|\n");
    // east end wall
  }
  // south end wall
  for (int x = 0; x < M5_MAZE_SIZE; x++) {
    printf("-----");
  }
  printf("+\n");
}
void m5maze_set_wall(m5Maze maze, m5Index index,
                     const m5Cell *newState) {
  uint8_t y = index.y;
  uint8_t x = index.x;
  maze->wall[y][x].byte |= (uint8_t)newState->byte | (uint8_t)0xf0;
  m5Cell d = maze->wall[y][x];
  // 周囲のセルのwallを同時に更新する
  if (y + 1 < M5_MAZE_SIZE) {
    maze->wall[y + 1][x].byte |=
        ((d.bits.North << 2) | (d.bits.DoneNorth << 6));
  }
  if (y - 1 > 0) {
    maze->wall[y - 1][x].byte |= ((d.bits.South) | (d.bits.DoneSouth << 4));
  }
  if (x + 1 < M5_MAZE_SIZE) {
    maze->wall[y][x + 1].byte |= ((d.bits.East << 3) | (d.bits.DoneEast << 7));
  }
  if (y - 1 > 0) {
    maze->wall[y][x - 1].byte |= ((d.bits.West << 1) | (d.bits.DoneWest << 5));
  }
  maze->dirty = 1;
}

m5Queue m5queue_constructor() {
  m5Queue queue = malloc(sizeof(m5QueueRecord));
  queue->size = M5_MAZE_SIZE * M5_MAZE_SIZE;
  queue->idx_first = queue->idx_last = 0;
  return queue;
}
void m5queue_destructor(m5Queue queue) { free(queue); }

void m5queue_clear(m5Queue queue) { queue->idx_first = queue->idx_last; }

uint8_t m5queue_enqueue(m5Queue queue, m5Index value) {
  size_t idx_next = (queue->idx_last + 1) % queue->size;
  if (queue->idx_first == idx_next) {
    // queue full
    return -1;
  }
  queue->array[queue->idx_last] = value;
  queue->idx_last = idx_next;
  return 0;
};

m5Index m5queue_dequeue(m5Queue queue) {
  if (m5queue_is_empty(queue)) {
    // queue empty
    return (m5Index){0xff, 0xff};
  }
  m5Index value = queue->array[queue->idx_first];
  queue->idx_first = (queue->idx_first + 1) % queue->size;
  return value;
};
uint8_t m5queue_is_empty(m5Queue queue) {
  return queue->idx_first == queue->idx_last;
};

void m5maze_update_step_map(m5Maze maze, m5Index destination) {
  if (!maze->dirty) {
    return;
  }
  maze->dirty = 0;

  for (size_t i = 0; i < M5_MAZE_SIZE; i++) {
    for (size_t j = 0; j < M5_MAZE_SIZE; j++) {
      maze->step_map[i][j] = 0xff;
    }
  }
  maze->step_map[destination.y][destination.x] = 0;

  m5Queue queue = m5queue_constructor();
  m5queue_enqueue(queue, destination);

  while (!m5queue_is_empty(queue)) {
    m5Index current_index = m5queue_dequeue(queue);
    m5Cell current_wall = maze->wall[current_index.y][current_index.x];
    for (uint8_t i = 0; i < 4; i++) {
      const m5Index scan_index = {current_index.x + M5_DIRECTIONS[i].x,
                                  current_index.y + M5_DIRECTIONS[i].y};
      const uint8_t current_step =
          maze->step_map[current_index.y][current_index.x];
      if (!m5Cell_is_wall(current_wall, i) &&
          maze->step_map[scan_index.y][scan_index.x] > current_step + 1) {
        maze->step_map[scan_index.y][scan_index.x] = current_step + 1;
        if (m5Cell_num_walls(maze->wall[scan_index.y][scan_index.x]) !=
            3) {
          m5queue_enqueue(queue, scan_index);
        }
      }
    }
  }
  m5queue_destructor(queue);
}
