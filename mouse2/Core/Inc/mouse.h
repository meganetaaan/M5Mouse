#ifndef _M5_MOUSE_H_
#define _M5_MOUSE_H_
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>
#include <drivers/wall_sensor.h>
#include <controllers/pid.h>
#include <controllers/motion.h>
// #include <controllers/command.h>

#define M5_PULSE_PER_ROTATE (4096 * 4 * 5)

typedef enum {
  M5_BRAKE = 0x00U,
  M5_ACCEL = 0x01U,
  M5_CONST = 0x02U,
  M5_DECEL = 0x03U
} M5ACCEL_OP;

typedef enum {
  M5_STRAIGHT = 0x00U,
  M5_SLALOM = 0x01U,
  M5_SPIN = 0x02U,
} M5RUN_OP;

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
  m5Motion2 motion;
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
  m5WallSensor sensor;
  m5WallInfo wall;
  m5PIDController controller_l;
  m5PIDController controller_r;
  M5RUN_OP current_run_op;
  uint8_t active;
} m5MouseRecord, *m5Mouse;

typedef struct {
  uint32_t count;
  uint32_t duration;
  float distance;
  float start_velocity;
  float max_velocity;
  float end_velocity;
} m5MotionOpRecord, *m5MotionOp;

m5MotionOp m5move_straight_constructor(float distance, float start_velocity, float max_velocity, float end_velocity);
m5Motion m5move_next(m5MotionOp op);
m5Motion m5move_is_end(m5MotionOp op);

m5Mouse m5mouse(void);
void m5mouse_start(m5Mouse mouse);
void m5mouse_update(m5Mouse mouse);
void m5mouse_update_wallinfo(m5Mouse mouse);
void m5mouse_update_current_orientation(m5Mouse mouse);
void m5mouse_update_target_velocity(m5Mouse mouse);
void m5mouse_update_current_velocity(m5Mouse mouse);
void m5mouse_apply_velocity(m5Mouse mouse);
void m5mouse_set_operation(m5Mouse mouse);
void m5mouse_straight(m5Mouse mouse, float distance, float max_velocity, float end_velocity);
void m5mouse_spin(m5Mouse mouse, float angle);
#endif
