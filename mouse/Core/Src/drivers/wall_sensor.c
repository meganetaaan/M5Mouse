#include "drivers/wall_sensor.h"
#include <stdlib.h>

m5WallSensor m5wallsensor_constructor(m5Sensor sensor_l, m5Sensor sensor_fl, m5Sensor sensor_fr, m5Sensor sensor_r) {
  m5WallSensor wall_sensor = malloc(sizeof(m5WallSensorRecord));
  wall_sensor->sensor_l = sensor_l;
  wall_sensor->sensor_fl = sensor_fl;
  wall_sensor->sensor_fr = sensor_fr;
  wall_sensor->sensor_r = sensor_r;
  wall_sensor->detected_frame_count_f = 0;
  wall_sensor->detected_frame_count_l = 0;
  wall_sensor->detected_frame_count_r = 0;
  wall_sensor->offset_l = 0;
  wall_sensor->offset_r = 0;
  wall_sensor->offset_sum_l = 0;
  wall_sensor->offset_sum_r = 0;
  wall_sensor->offset_count = 0;
  wall_sensor->threshold_l = M5_WALL_THRESH_L;
  wall_sensor->threshold_f = M5_WALL_THRESH_F;
  wall_sensor->threshold_r = M5_WALL_THRESH_R;
  return wall_sensor;
}

void m5wallsensor_calibrate(m5WallSensor ws) {
  ws->offset_sum_l = 0;
  ws->offset_sum_r = 0;
  ws->offset_count = M5_WALL_OFFSET_COUNT;
}

void m5wallsensor_start(m5WallSensor ws) {
  m5sensor_start(ws->sensor_l);
  m5sensor_start(ws->sensor_fl);
  m5sensor_start(ws->sensor_fr);
  m5sensor_start(ws->sensor_r);
}

m5WallInfo m5wallsensor_update(m5WallSensor ws) {
  if (ws->offset_count > 0) {
    ws->offset_count--;
    ws->offset_sum_l += ws->sensor_l->value;
    ws->offset_sum_r += ws->sensor_r->value;
    if(ws->offset_count == 0) {
      ws->offset_l = ws->offset_sum_l / M5_WALL_OFFSET_COUNT;
      ws->offset_r = ws->offset_sum_r / M5_WALL_OFFSET_COUNT;
    }
  }
  uint16_t fs = (ws->sensor_fr->value + ws->sensor_fr->value) / 2;
  if (fs < ws->threshold_f) {
    ws->detected_frame_count_f = 0;
  } else {
    ws->detected_frame_count_f++;
    if (ws->detected_frame_count_f > M5_WALL_COUNT) {
      ws->detected_frame_count_f = M5_WALL_COUNT;
    }
  }
  uint16_t ls = ws->sensor_l->value;
  if (ls < ws->threshold_l) {
    ws->detected_frame_count_l = 0;
  } else {
    ws->detected_frame_count_l++;
    if (ws->detected_frame_count_l > M5_WALL_COUNT) {
      ws->detected_frame_count_l = M5_WALL_COUNT;
    }
  }
  uint16_t rs = ws->sensor_r->value;
  if (rs < ws->threshold_r) {
    ws->detected_frame_count_r = 0;
  } else {
    ws->detected_frame_count_r++;
    if (ws->detected_frame_count_r > M5_WALL_COUNT) {
      ws->detected_frame_count_r = M5_WALL_COUNT;
    }
  }
  m5WallInfo info = (m5WallInfo){0, 0, 0, 0, 0};
  info.front = ws->detected_frame_count_f == M5_WALL_COUNT;
  info.left = ws->detected_frame_count_l == M5_WALL_COUNT;
  info.right = ws->detected_frame_count_r == M5_WALL_COUNT;
  info.left_error = info.left ? ls - ws->offset_l : 0;
  info.right_error = info.right ? rs - ws->offset_r : 0;
  return info;
}
