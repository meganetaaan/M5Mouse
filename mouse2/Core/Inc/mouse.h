#ifndef _M5_MOUSE_H_
#define _M5_MOUSE_H_
#include <common/geometry.h>
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>
#include <drivers/wall_sensor.h>
#include <controllers/pid.h>
#include <controllers/motion.h>

#define M5_PULSE_PER_ROTATE (4096 * 4 * 5)
#define M5_BODY_WIDTH 54
#define M5_WALL_ADJUST_GAIN (0.001f)

typedef struct {
  m5Motion motion;
  m5Odometry odometry;
  m5MotionQueue motion_queue;
  m5Velocity current_velocity;
  m5Velocity target_velocity;
  m5Velocity cap_velocity;
  m5Accel cap_accel;
  m5Position position;
  m5Gyro gyro;
  m5Encoder encoder_l;
  m5Encoder encoder_r;
  m5Motor motor_l;
  m5Motor motor_r;
  m5WallSensor sensor;
  m5WallInfo wall;
  m5PIDController controller_l;
  m5PIDController controller_r;
  m5PIDController controller_wall;
  uint8_t is_wall_adjust_enabled;
  uint8_t active;
  uint8_t count;
  m5TrackTarget track_target;
} m5MouseRecord, *m5Mouse;

typedef struct {
  uint32_t count;
  uint32_t duration;
  m5Position position;
  m5Velocity start_velocity;
  m5Velocity max_velocity;
  m5Velocity end_velocity;
} m5MotionOpRecord, *m5MotionOp;

m5MotionOp m5move_straight_constructor(float distance, float start_velocity, float max_velocity, float end_velocity);
m5Velocity m5move_next(m5MotionOp op);
uint8_t m5move_is_end(m5MotionOp op);
m5Mouse m5mouse(void);
void m5mouse_start(m5Mouse mouse);
void m5mouse_update(m5Mouse mouse);
void m5mouse_update_wallinfo(m5Mouse mouse);
void m5mouse_update_position(m5Mouse mouse);
void m5mouse_update_target_velocity(m5Mouse mouse);
void m5mouse_update_velocity(m5Mouse mouse);
void m5mouse_apply_velocity(m5Mouse mouse);
void m5mouse_set_operation(m5Mouse mouse);
void m5mouse_straight(m5Mouse mouse, float distance, float start_velocity, float max_velocity, float end_velocity);
void m5mouse_slalom(m5Mouse mouse, float degrees, float d, float const_velocity, float ang_accel, float max_omega);
void m5mouse_spin(m5Mouse mouse, float degrees);
#endif
