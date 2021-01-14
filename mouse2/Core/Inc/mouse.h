#ifndef _M5_MOUSE_H
#define _M5_MOUSE_H
#include <drivers/gyro.h>
#include <drivers/encoder.h>
#include <drivers/motor.h>

struct m5MouseRecord {
  m5Gyro gyro;
  m5Encoder encoderL;
  m5Encoder encoderR;
  m5Motor motorL;
  m5Motor motorR;
};
typedef struct m5MouseRecord m5MouseRecord;
typedef struct m5MouseRecord *m5Mouse;
#endif
