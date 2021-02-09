#include "maze/agent.h"
#include "mouse.h"
#include <stdlib.h>
#include <stdio.h>
#include "maze/maze_data.h"
// #define M5_SIMULATION

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
  // agent->curve_mode = M5_CURVE_SLALOM;
  agent->curve_mode = M5_CURVE_SPIN;
  agent->curving_velocity = mouse->cap_velocity;
  agent->max_velocity = mouse->cap_velocity;
  agent->straight_count = 0;
  return agent;
}

void m5agent_advance(m5MazeAgent agent) {
  m5Index dir = M5_DIRECTIONS[agent->direction];
  agent->position.x = agent->position.x + dir.x;
  agent->position.y = agent->position.y + dir.y;
}

void m5agent_go_forward(m5MazeAgent agent, uint8_t force) {
  /*
  if (force) {
    float v = agent->mouse->cap_velocity.v;
    if (agent->mouse) {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH, v, v, v);
    }
  } else {
    agent->straight_count += 1;
  }
  */
  float v = agent->mouse->cap_velocity.v;
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH, v, v, v);
  m5agent_advance(agent);
}

void m5agent_turn_left(m5MazeAgent agent) {
  float v = agent->curving_velocity.v;
  float alpha = agent->mouse->cap_accel.alpha;
  float omega = agent->mouse->cap_velocity.omega;
  if (agent->mouse) {
    // m5agent_flush_straight(agent);
    if (agent->curve_mode == M5_CURVE_SLALOM) {
      m5mouse_slalom(agent->mouse, -90, M5_MAZE_WIDTH / 2, v, alpha, omega);
    } else {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, v, v, 0);
      m5mouse_spin(agent->mouse, -90);
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, 0, v, v);
    }
  }
  agent->direction = (agent->direction + 3) % 4;
  m5agent_advance(agent);
}

void m5agent_turn_right(m5MazeAgent agent) {
  float v = agent->curving_velocity.v;
  float alpha = agent->mouse->cap_accel.alpha;
  float omega = agent->mouse->cap_velocity.omega;
  if (agent->mouse) {
    // m5agent_flush_straight(agent);
    if (agent->curve_mode == M5_CURVE_SLALOM) {
      m5mouse_slalom(agent->mouse, 90, M5_MAZE_WIDTH / 2, v, alpha, omega);
    } else {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, v, v, 0);
      m5mouse_spin(agent->mouse, 90);
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, 0, v, v);
    }
  }
  agent->direction = (agent->direction + 1) % 4;
  m5agent_advance(agent);
}

void m5agent_flush_straight(m5MazeAgent agent) {
  float v = agent->curving_velocity.v;
  float vmax = agent->max_velocity.v;
  if (agent->straight_count > 0) {
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH * agent->straight_count, v, vmax, v);
    agent->straight_count = 0;
  }
}

void m5agent_go_backward(m5MazeAgent agent, uint8_t adjust) {
  float v = agent->curving_velocity.v;
  if (agent->mouse) {
    // m5agent_flush_straight(agent);
    if (0) {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, v, v, 0);
      m5mouse_spin(agent->mouse, 180);
      m5mouse_straight(agent->mouse, -M5_MAZE_WIDTH / 2 + 5, 0, v, 0);
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH - (M5_BODY_WIDTH + 1.2) / 2, 0, v, v);
    } else {
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, v, v, 0);
      m5mouse_spin(agent->mouse, 180);
      m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, 0, v, v);
    }
  }
  agent->direction = (agent->direction + 2) % 4;
  m5agent_advance(agent);
}

m5Cell m5agent_get_current_cell(m5MazeAgent agent) {
  return agent->maze->wall[agent->position.y][agent->position.x];
}

m5Cell m5agent_get_wall(m5MazeAgent agent) {
#ifdef M5_SIMULATION
  m5Cell cell = agent->_maze->wall[agent->position.y][agent->position.x];
  return cell;
#else
  m5Mouse mouse = agent->mouse;
  uint8_t walls[4];
  m5WallInfo wall = mouse->wall;
  walls[0] = wall.front;
  walls[1] = wall.right;
  walls[2] = 0; // 進んできた方向なので壁は無いとみなす
  walls[3] = wall.left;
  m5Cell cell = {0x00};
  m5Direction dir = agent->direction;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t shift = (dir + i) % 4;
    cell.byte |= (walls[i] << shift);
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
      continue;
    }
    uint8_t s = agent->maze->step_map[agent->position.y + idx.y][agent->position.x + idx.x];
    if (s < step){
      dir = i;
      step = s;
    }
  }
  return (m5Direction)dir;
}

void m5agent_search_run(m5MazeAgent agent, m5Index goal) {
  m5Mouse mouse = agent->mouse;
  m5Maze maze = agent->maze;
  float v = mouse->cap_velocity.v;

  // ゴール設定
  agent->goal = goal;

  // 半歩進む
  if (mouse) {
    m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, 0, v, v);
  }
  m5agent_advance(agent);

  // ゴールまで繰り返し
  uint16_t count = 0;
  agent->straight_count = 0;
  while(agent->state != M5_AGENT_STATE_GOAL) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
    if (!m5cell_is_visited(m5agent_get_current_cell(agent))) {
      // m5agent_flush_straight(agent);
      // 壁を測定
      m5Cell wall = m5agent_get_wall(agent);
      // 迷路をアップデート
      m5maze_set_wall(maze, agent->position, &wall);
    }
    // 次の進路を決定
    m5maze_update_step_map(maze, agent->goal);
    m5Direction next_dir = m5agent_get_next_direction(agent);
    uint8_t adjust;
    // 進む
    switch (next_dir) {
      case M5_DIR_FORWARD:
        m5agent_go_forward(agent, 0);
        break;
      case M5_DIR_RIGHT:
        m5agent_turn_right(agent);
        break;
      case M5_DIR_BACKWARD:
        // 進行方向に壁があれば背面調整をする
        adjust = m5cell_is_wall(m5agent_get_current_cell(agent), agent->direction);
        m5agent_go_backward(agent, adjust);
        break;
      case M5_DIR_LEFT:
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
    m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, v, v, 0);
  }
  // ゴール！
  return;
}

void m5agent_fast_run(m5MazeAgent agent, m5Index goal) {
}
