#ifndef M5_CONTROLLERS_COMMAND_H
#define M5_CONTROLLERS_COMMAND_H
#include <stm32f4xx_hal.h>

#define MAX_VEL 200
#define END_VEL 20
#define ACCEL (20.0)

typedef struct {
  int distance;
  int angle;
} m5CoordinateRecord, *m5Coordinate;

typedef struct {
  int velocity;
  int anglar_velocity;
} m5VelocityRecord, *m5Velocity;

typedef enum {
  M5_BRAKE = 0x00U,
  M5_ACCEL = 0x01U,
  M5_CONST = 0x02U,
  M5_DECEL = 0x03U
} M5ACCEL_OP;

#endif
