#ifndef M5_MAZE_AGENT_H
#define M5_MAZE_AGENT_H

#include "maze.h"
#include "mouse.h"
#define M5_WALL_THRESH_L (300.0f)
#define M5_WALL_THRESH_F (300.0f)
#define M5_WALL_THRESH_F (300.0f)
#define M5_WALL_THRESH_R (300.0f)
#define M5_MAZE_WIDTH (180.0f)

typedef enum {
  M5_DIR_NORTH = 0x00U,
  M5_DIR_EAST = 0x01U,
  M5_DIR_SOUTH = 0x02U,
  M5_DIR_WEST = 0x03U
} m5Direction;

typedef enum {
  M5_DIR_FORWARD = 0x00U,
  M5_DIR_RIGHT = 0x01U,
  M5_DIR_BACKWARD = 0x02U,
  M5_DIR_LEFT = 0x03U,
} m5RelativeDirection;

typedef enum {
  M5_AGENT_STATE_GOAL = 0x00U,
  M5_AGENT_STATE_SEARCHING = 0x01U,
  M5_AGENT_STATE_RUNNING = 0x02U,
  M5_AGENT_STATE_GOING_HOME = 0x03U
} m5AgentState;

typedef struct {
  m5Maze maze;
  m5Mouse mouse;
  m5AgentState state;
  m5Index position;
  m5Direction direction;
  m5Index goal;
} m5MazeAgentRecord, *m5MazeAgent;

void search_run(m5MazeAgent agent, m5Index goal);
void fast_run(m5MazeAgent agent, m5Index goal);

#endif
