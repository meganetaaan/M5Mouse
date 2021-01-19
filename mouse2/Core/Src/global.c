#include <global.h>

m5MouseRecord m5mouseRecord = {0};
m5Mouse mouse = &m5mouseRecord;
uint32_t m5timerCount = 0;
uint8_t m5transferRequested = 0;
uint8_t m5transferDirection = 0;
uint8_t m5i2cbuffer[256] = {0};
uint16_t i2cCount = 0;
