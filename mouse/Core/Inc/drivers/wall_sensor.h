#ifndef _M5_DRIVER_WALL_SENSOR_H_
#define _M5_DRIVER_WALL_SENSOR_H_
#include "sensor.h"

// #define M5_WALL_THRESH_L (780)
// #define M5_WALL_THRESH_F (550)
// #define M5_WALL_THRESH_R (1050)
#define M5_WALL_THRESH_L (740)
#define M5_WALL_THRESH_F (570)
#define M5_WALL_THRESH_R (1010)
#define M5_WALL_COUNT (2)
#define M5_WALL_OFFSET_COUNT (100)

typedef struct {
  m5Sensor sensor_l;
  m5Sensor sensor_fl;
  m5Sensor sensor_fr;
  m5Sensor sensor_r;
  uint16_t threshold_l;
  uint16_t threshold_f;
  uint16_t threshold_r;
  uint16_t detected_frame_count_l;
  uint16_t detected_frame_count_f;
  uint16_t detected_frame_count_r;
  uint16_t offset_l;
  uint16_t offset_r;
  uint32_t offset_sum_l;
  uint32_t offset_sum_r;
  uint16_t offset_count;
} m5WallSensorRecord, *m5WallSensor;

typedef struct {
  uint8_t left;
  uint8_t right;
  uint8_t front;
  int32_t left_error;
  int32_t right_error;
} m5WallInfo;

m5WallSensor m5wallsensor_constructor(m5Sensor sensor_l, m5Sensor sensor_fl, m5Sensor sensor_fr, m5Sensor sensor_r);
void m5wallsensor_start(m5WallSensor ws);
void m5wallsensor_calibrate(m5WallSensor ws);
m5WallInfo m5wallsensor_update(m5WallSensor ws);

#endif
