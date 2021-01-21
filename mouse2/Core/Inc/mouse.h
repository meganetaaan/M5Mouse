#ifndef _M5_MOUSE_H
#define _M5_MOUSE_H
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>
#include <controllers/pid.h>
// #include <controllers/command.h>

#define M5_PULSE_PER_ROTATE (4096 * 4 * 5)

typedef enum {
  M5_BRAKE = 0x00U,
  M5_ACCEL = 0x01U,
  M5_CONST = 0x02U,
  M5_DECEL = 0x03U
} M5ACCEL_OP;

typedef float m5MotionValue;

typedef struct {
  m5MotionValue distance;
  m5MotionValue angle;
} m5CoordinateRecord, *m5Coordinate;

typedef struct {
  m5MotionValue vel;
  m5MotionValue ang_vel;
  m5MotionValue accel;
  m5MotionValue ang_accel;
} m5MotionRecord, *m5Motion;

m5Motion m5motion_constructor(m5MotionValue vel, m5MotionValue ang_vel, m5MotionValue accel, m5MotionValue ang_accel);
m5Coordinate m5coordinate_constructor(m5MotionValue distance, m5MotionValue angle);

typedef struct {
  m5Motion current_motion;
  m5Motion target_motion;
  m5Motion cap_motion;
  m5Coordinate current_coordinate;
  m5Coordinate target_coordinate;
  m5Gyro gyro;
  m5Encoder encoder_l;
  m5Encoder encoder_r;
  m5Motor motor_l;
  m5Motor motor_r;
  m5Sensor sensor_l;
  m5Sensor sensor_fl;
  m5Sensor sensor_fr;
  m5Sensor sensor_r;
  m5PIDController controller_l;
  m5PIDController controller_r;
  M5ACCEL_OP current_op;
  uint8_t active;
} m5MouseRecord, *m5Mouse;

m5Mouse m5mouse(void);
void m5mouse_start(m5Mouse mouse);
void m5mouse_update(m5Mouse mouse);
void m5mouse_update_current_accel(m5Mouse mouse);
void m5mouse_update_current_orientation(m5Mouse mouse);
void m5mouse_update_target_velocity(m5Mouse mouse);
void m5mouse_update_current_velocity(m5Mouse mouse);
void m5mouse_apply_velocity(m5Mouse mouse);
void m5mouse_set_operation(m5Mouse mouse);
void m5mouse_straight(m5Mouse mouse, float distance);
void m5mouse_spin(m5Mouse mouse, float angle);
M5ACCEL_OP m5command_next_operation(M5ACCEL_OP current_op, m5Motion current_motion,
                      m5Motion target_motion, m5Coordinate current_coordinate,
                      m5Coordinate target, float delta);
#endif
