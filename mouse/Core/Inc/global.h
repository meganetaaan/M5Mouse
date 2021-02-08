#ifndef _M5_GLOBAL_H_
#define _M5_GLOBAL_H_
#include <mouse.h>

#define M5_REGISTER_WHO_AM_I 0x68
#define M5_REGISTER_RUN_SEARCH 0x40
#define M5_REGISTER_RUN_FAST 0x41
#define M5_REGISTER_SHOW_MAZE 0x50
#define M5_REGISTER_TEST_STRAIGHT 0xf0
#define M5_REGISTER_TEST_SPIN 0xf1
#define M5_REGISTER_TEST_SLALOM 0xf2
#define M5_REGISTER_TEST_ZIG_ZAG 0xf3
#define M5_REGISTER_TEST_CRANK 0xf4
#define M5_REGISTER_TEST_FAST 0xf5
#define M5_REGISTER_CALIBRATE 0xe0
#define M5_REGISTER_RESET 0xe1
#define M5_VALUE_WHO_AM_I 0x64
#define M5_FREQUENCY (1000.0f)
#define M5_TARGET_FREQUENCY (1000.0f)
#define M5_DELTA (1.0f / M5_FREQUENCY)
#define M5_CELL_WIDTH (180.0f)
#define M5_VELOCITY_SEARCH (280)
#define M5_ACCEL_SEARCH (900)
#define M5_ANG_VELOCITY (6 * PI)
#define M5_ANG_ACCEL (9 * PI)

extern m5MouseRecord m5mouseRecord;
extern m5Mouse mouse;
extern uint32_t m5timerCount;
extern uint8_t m5transferRequested;
extern uint8_t m5transferDirection;
extern uint8_t m5i2cbuffer[256];
extern uint16_t m5i2c_count;
extern uint16_t m5sensor_count;

#endif
