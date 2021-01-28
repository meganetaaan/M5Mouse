#include "maze/agent.h"

extern const m5Index M5_DIRECTIONS[4];

m5MazeAgent m5mazeagent_constructor(m5Maze maze, m5Mouse mouse) {
  m5MazeAgent agent = malloc(sizeof(m5MazeAgentRecord));
  agent->maze = maze;
  agent->mouse = mouse;
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
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH, agent->mouse->cap_motion->vel, 0);
  m5agent_advance(agent);
}

void m5agent_turn_left(m5MazeAgent agent) {
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  m5mouse_spin(agent->mouse, -90);
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  agent->direction = (agent->direction + 3) % 4;
  m5agent_advance(agent);
}

void m5agent_turn_right(m5MazeAgent agent) {
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  m5mouse_spin(agent->mouse, -90);
  m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  agent->direction = (agent->direction + 1) % 4;
  m5agent_advance(agent);
}

void m5agent_turn_back(m5MazeAgent agent, uint8_t adjust) {
  if (adjust) {
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
    m5mouse_spin(agent->mouse, 180);
    m5mouse_straight(agent->mouse, M5_MAZE_WIDTH / 2, agent->mouse->cap_motion->vel, 0);
  } else {
    m5mouse_spin(agent->mouse, 180);
  }
  agent->direction = (agent->direction + 2) % 4;
  m5agent_advance(agent);
}

m5Cell m5agent_get_wall(m5MazeAgent agent) {
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
}

m5Direction m5agent_get_next_direction(m5MazeAgent agent) {
  uint8_t dir = 0;
  uint8_t step = 0xff;
  m5Cell current_cell = agent->maze->wall[agent->position.y][agent->position.y];
  // 同じスコアのセルがある場合は前>右>後>左の順に優先される
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t d = (agent->direction + i) % 4;
    m5Index idx = M5_DIRECTIONS[d];
    if (m5Cell_is_wall(current_cell, d)) {
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

void search_run(m5MazeAgent agent, m5Index goal) {
  m5Mouse mouse = agent->mouse;
  m5Maze maze = agent->maze;

  // ゴール設定
  agent->goal = goal;

  // 半歩進む
  m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, mouse->cap_motion->vel / 2, 0);
  m5agent_advance(agent);

  // ゴールまで繰り返し
  while(agent->state != M5_AGENT_STATE_GOAL) {
    // 壁を測定
    m5Cell wall = m5agent_get_wall(agent);
    // 迷路をアップデート
    m5maze_set_wall(agent->maze, agent->position, &wall);
    m5maze_update_step_map(agent->maze, agent->goal);
    // 次の進路を決定
    m5Direction next_dir = m5agent_get_next_direction(agent);
    uint8_t adjust;
    // 進む
    switch (next_dir) {
      case M5_DIR_FORWARD:
        m5agent_straight(agent);
        break;
      case M5_DIR_RIGHT:
        m5agent_turn_right(agent);
        break;
      case M5_DIR_BACKWARD:
        adjust = m5cell_is_wall(agent->maze->wall, agent->direction);
        m5agent_turn_back(agent, adjust);
        break;
      case M5_DIR_LEFT:
        m5agent_turn_left(agent);
        break;
    }
    if (agent->position.x == agent->goal.x && agent->position.y == agent->goal.y) {
      agent->state = M5_AGENT_STATE_GOAL;
    }
  }
  // セルの中央まで進む
  m5mouse_straight(mouse, M5_MAZE_WIDTH / 2, mouse->cap_motion->vel / 2, 0);
  // ゴール！
  return;
}

void fast_run(m5MazeAgent agent, m5Index goal) {
}
