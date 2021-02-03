#ifndef M5_CONTROLLERS_PID_H_
#define M5_CONTROLLERS_PID_H_
#include <stm32f4xx_hal.h>

#define M5DEFAULT_PGAIN (0.020f)
#define M5DEFAULT_IGAIN (0.0020f)
#define M5DEFAULT_DGAIN (0.0020f)

#define M5DEFAULT_WALL_PGAIN (0.005f)
#define M5DEFAULT_WALL_IGAIN (0.0005f)
#define M5DEFAULT_WALL_DGAIN (0.0005f)

typedef float m5Value;
typedef struct {
  float p_gain;
  float i_gain;
  float d_gain;
  float i_saturation;
} m5PIDConfigurationRecord, *m5PIDConfiguration;

typedef struct {
  uint8_t initialized;
  m5Value last_value;
  m5Value integrated_value;
} m5PIDContextRecord, *m5PIDContext;

typedef struct {
  m5PIDConfiguration config;
  m5PIDContext context;
  m5Value value;
} m5PIDControllerRecord, *m5PIDController;

m5PIDController m5pid_constructor(float p, float i, float d, float saturation);
void m5pid_reset(m5PIDController pid);
m5Value m5pid_update(m5PIDController pid, m5Value target_value, m5Value current_value);

#endif
