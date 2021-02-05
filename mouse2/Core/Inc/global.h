#ifndef _M5_GLOBAL_H_
#define _M5_GLOBAL_H_
#include <mouse.h>

#define M5_REGISTER_WHO_AM_I 0x68
#define M5_REGISTER_TEST 0xff
#define M5_REGISTER_CALIBRATE 0xf0
#define M5_VALUE_WHO_AM_I 0x64
#define M5_FREQUENCY (1000.0f)
#define M5_TARGET_FREQUENCY (1000.0f)
#define M5_DELTA (1.0f / M5_FREQUENCY)
#define M5_CELL_WIDTH (180.0f)

extern m5MouseRecord m5mouseRecord;
extern m5Mouse mouse;
extern uint32_t m5timerCount;
extern uint8_t m5transferRequested;
extern uint8_t m5transferDirection;
extern uint8_t m5i2cbuffer[256];
extern uint16_t m5i2c_count;
extern uint16_t m5sensor_count;

#endif
