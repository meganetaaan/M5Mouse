#ifndef _M5_GLOBAL_H
#define _M5_GLOBAL_H
#include <mouse.h>

#define M5_REGISTER_WHO_AM_I 0x68
#define M5_REGISTER_TEST 0xff
#define M5_VALUE_WHO_AM_I 0x64
extern m5MouseRecord m5mouseRecord;
extern m5Mouse mouse;
extern uint32_t m5timerCount;
extern uint8_t m5transferRequested;
extern uint8_t m5transferDirection;
extern uint8_t m5i2cbuffer[256];
extern uint16_t i2cCount;

#endif
