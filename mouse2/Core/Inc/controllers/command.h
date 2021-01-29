#ifndef M5_CONTROLLERS_COMMAND_H_
#define M5_CONTROLLERS_COMMAND_H_
#include <stm32f4xx_hal.h>

#define MAX_VEL 200
#define END_VEL 20
#define ACCEL (20.0)

M5ACCEL_OP m5command_next_operation(M5ACCEL_OP current_op, m5Velocity current_velocity,
                      m5Velocity target_velocity, m5Coordinate current,
                      m5Coordinate target, uint16_t delta);
#endif
