#include "maze/agent.h"
#include <stdlib.h>
#include <stdio.h>
#include "maze/maze_data.h"
#define M5_SIMULATION

extern const m5Index M5_DIRECTIONS[4];

m5MazeAgent m5mazeagent_constructor(m5Maze maze, m5Mouse mouse) {
  m5MazeAgent agent = malloc(sizeof(m5MazeAgentRecord));
  agent->maze = maze;
#ifdef M5_SIMULATION
  agent->mouse = NULL;
#else
  agent->mouse = mouse;
#endif
  m5Maze test_maze = m5maze_constructor();
  m5maze_load_from_array(test_maze, mazeData_maze);
  agent->_maze = test_maze;
  agent->position = (m5Index){0, 0};
  agent->direction = M5_DIR_NORTH;
  return agent;
}

void m5agent_advance(m5MazeAgent agent) {
  m5Index dir = M5_DIRECTIONS[agent->direction];
  agent->position.x = agent->position.x + dir.x;
  agent->position.y = agent->position.y + dir.y;
}

void m5agent_straight(m5MazeAgent agent) {
  if (agent->mouse) {
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH, agent->mouse->cap_motion->vel, 0);
  }
  m5agent_advance(agent);
}

void m5agent_turn_left(m5MazeAgent agent) {
  if (agent->mouse) {
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
    m5mouse_spin(agent->mouse, -90);
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  }
  agent->direction = (agent->direction + 3) % 4;
  m5agent_advance(agent);
}

void m5agent_turn_right(m5MazeAgent agent) {
  if (agent->mouse) {
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
    m5mouse_spin(agent->mouse, -90);
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  }
  agent->direction = (agent->direction + 1) % 4;
  m5agent_advance(agent);
}

void m5agent_turn_back(m5MazeAgent agent, uint8_t adjust) {
  if (agent->mouse) {
    if (adjust) {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
      m5mouse_spin(agent->mouse, 180);
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
    } else {
      m5mouse_spin(agent->mouse, 180);
    }
  }
  agent->direction = (agent->direction + 2) % 4;
  m5agent_advance(agent);
}

m5Cell m5agent_get_wall(m5MazeAgent agent) {
#ifdef M5_SIMULATION
  m5Cell cell = agent->_maze->wall[agent->position.y][agent->position.x];
  return cell;
#else
  m5Mouse mouse = agent->mouse;
  uint8_t walls[4];
  walls[0] = (mouse->sensor_fr->value + mouse->sensor_fr->value) / 2.0 > M5_WALL_THRESH_F;
  walls[1] = mouse->sensor_r->value > M5_WALL_THRESH_R;
  walls[2] = 0; // 進んできた方向なので壁は無い
  walls[3] = mouse->sensor_l->value > M5_WALL_THRESH_L;
  m5Cell cell = {0x00};
  m5Direction dir = agent->direction;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t shift = (dir + i) % 4;
    cell.byte |= (0x01 << shift);
  }
  return cell;
#endif
}

m5Direction m5agent_get_next_direction(m5MazeAgent agent) {
  uint8_t dir = 0;
  uint8_t step = 0xff;
  m5Cell current_cell = agent->maze->wall[agent->position.y][agent->position.x];
  // 同じスコアのセルがある場合は前>右>後>左の順に優先される
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (agent->direction + i) % 4;
    m5Index idx = M5_DIRECTIONS[d];
    if (m5cell_is_wall(current_cell, d)) {
      printf("%d is wall, ", d);
      continue;
    }
    uint8_t s = agent->maze->step_map[agent->position.y + idx.y][agent->position.x + idx.x];
    if (s < step){
      dir = i;
      step = s;
    }
  }
  printf("->");
  return (m5Direction)dir;
}

void m5agent_search_run(m5MazeAgent agent, m5Index goal) {
  m5Mouse mouse = agent->mouse;
  m5Maze maze = agent->maze;

  // ゴール設定
  agent->goal = goal;

  // 半歩進む
  if (mouse) {
    m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, mouse->cap_motion->vel / 2, 0);
  }
  m5agent_advance(agent);

  // ゴールまで繰り返し
  uint16_t count = 0;
  while(agent->state != M5_AGENT_STATE_GOAL) {
    printf("count: %u, position: (%u, %u), \n", count, agent->position.x, agent->position.y);
    m5maze_print_step_map(maze);
    // 壁を測定
    m5Cell wall = m5agent_get_wall(agent);
    // 迷路をアップデート
    m5maze_set_wall(maze, agent->position, &wall);
    m5maze_update_step_map(maze, agent->goal);
    // 次の進路を決定
    m5Direction next_dir = m5agent_get_next_direction(agent);
    uint8_t adjust;
    // 進む
    switch (next_dir) {
      case M5_DIR_FORWARD:
        printf("going: forward...\n");
        m5agent_straight(agent);
        break;
      case M5_DIR_RIGHT:
        printf("going: right...\n");
        m5agent_turn_right(agent);
        break;
      case M5_DIR_BACKWARD:
        printf("going: backward...\n");
        // 進行方向に壁があれば背面調整をする
        adjust = m5cell_is_wall(wall, agent->direction);
        m5agent_turn_back(agent, adjust);
        break;
      case M5_DIR_LEFT:
        printf("going: left...\n");
        m5agent_turn_left(agent);
        break;
    }
    if (agent->position.x == agent->goal.x && agent->position.y == agent->goal.y) {
      agent->state = M5_AGENT_STATE_GOAL;
    }
    count++;
  }
  // セルの中央まで進む
  if (mouse) {
    m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, mouse->cap_motion->vel / 2, 0);
  }
  // ゴール！
  return;
}

void m5agent_fast_run(m5MazeAgent agent, m5Index goal) {
}
