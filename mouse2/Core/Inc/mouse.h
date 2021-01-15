#ifndef _M5_MOUSE_H
#define _M5_MOUSE_H
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>
#include <drivers/sensor.h>

struct m5MouseRecord {
  m5Gyro gyro;
  m5Encoder encoderL;
  m5Encoder encoderR;
  m5Motor motorL;
  m5Motor motorR;
  m5Sensor sensorL;
  m5Sensor sensorFL;
  m5Sensor sensorFR;
  m5Sensor sensorR;
};
typedef struct m5MouseRecord m5MouseRecord;
typedef struct m5MouseRecord *m5Mouse;

m5Mouse m5mouse(void);
void m5mouse_update(m5Mouse mouse);
void m5mouse_set_operation(m5Mouse mouse);
#endif
