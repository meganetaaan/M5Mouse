#ifndef _M5_MOUSE_H
#define _M5_MOUSE_H
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>

#define M5_PULSE_PER_ROTATE (4096 * 4 * 5)

typedef float m5MotionValue;

typedef struct {
  m5MotionValue distance;
  m5MotionValue angle;
} m5CoordinateRecord, m5Coordinate;

typedef struct {
  m5MotionValue vel;
  m5MotionValue ang_vel;
  m5MotionValue accel;
  m5MotionValue ang_accel;
} m5MotionRecord, *m5Motion;

typedef struct {
  m5Motion current_motion;
  m5Motion target_motion;
  m5Motion cap_motion;
  m5Coordinate current_coordinate;
  m5Coordinate target_coordinate;
  m5Gyro gyro;
  m5Encoder encoderL;
  m5Encoder encoderR;
  m5Motor motorL;
  m5Motor motorR;
  m5Sensor sensorL;
  m5Sensor sensorFL;
  m5Sensor sensorFR;
  m5Sensor sensorR;
} m5MouseRecord, *m5Mouse;

m5Mouse m5mouse(void);
void m5mouse_enqueue(m5Mouse mouse);
void m5mouse_run(m5Mouse mouse);
void m5mouse_update(m5Mouse mouse);
#endif
