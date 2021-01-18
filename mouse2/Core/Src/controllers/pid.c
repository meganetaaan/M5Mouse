#include <stm32f4xx_hal.h>
#define M5DEFAULT_PGAIN (20.0f)
#define M5DEFAULT_IGAIN (0.20f)
#define M5DEFAULT_DGAIN (0.0f)
typedef float m5Value;

struct m5PIDGainRecord {
  float P;
  float I;
  float D;
};
typedef m5PIDGainRecord m5PIDGainRecord;
typedef m5PIDGainRecord *m5PIDGain;

struct m5PIDContextRecord {
  uint8_t initialized;
  m5Value last_value;
  m5Value integrated_value;
};
typedef m5PIDContextRecord m5PIDContextRecord;
typedef m5PIDContextRecord *m5PIDContext;

struct m5PIDControllerRecord {
  m5PIDGain gain;
  m5PIDContext context;
};
typedef m5PIDControllerRecord m5PIDControllerRecord;
typedef m5PIDControllerRecord *m5PIDController;

m5PIDController m5PIDCOntroller() {
  m5PIDGainRecord gain = {
    M5DEFAULT_PGAIN,
    M5DEFAULT_IGAIN,
    M5DEFAULT_DGAIN
  };
  m5PIDContextRecord ctx = {
    0,
    (m5Value)0,
    (m5Value)0
  };
  m5PIDControllerRecord record = {
    &gain,
    &ctx
  };
  return &record;
}

m5pid_update(m5PIDController pid, m5Value target_value, m5Value current_value) {
  if (pid->context->activated) {
    pid->context->last_value = current_value;
  }
  m5Value diff = target_value - current_value;

  // update_context
  m5Value integrated = pid->context->integrated_value =
      pid->context->integrated_value += diff;
  m5Value last_value = pid->context->last_value;
  float p = diff * pid->gain->P;
  float i = integrated * pid->gain->I;
  float d = (current_value - last_value) * pid->gain->D;
  pid->value = p + i + d;
}
