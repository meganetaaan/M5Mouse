#include <stm32f4xx_hal.h>
void delay_us(int us) {
  long int loop = us * SystemCoreClock / 1000000;
  while (loop-- > 0) {
  }
  return;
}
