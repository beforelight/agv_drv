#include "drv_beep.h"
#include "main.h"
#include "tim.h"

int tick;
int led_tick;

void beep_increase(int t)
{
  tick -= t;
  if (tick > 0) {
#ifndef AGV_DRV_DEVOLOP
    htim14.Instance->CCR1 = htim14.Instance->ARR >> 1;
#else
    htim14.Instance->CCR1 = 50;
#endif // !AGV_DRV_DEVOLOP
  } else {
    tick = 0;
    htim14.Instance->CCR1 = 0;
  }
}

void beep_on(int ms)
{
  tick = tick > ms ? tick : ms;
}

void led_increase(int ms)
{
  led_tick -= ms;
  if (led_tick > 0) {
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  } else {
    led_tick = 0;
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  }
}

void led_on(int ms)
{
  led_tick = led_tick > ms ? led_tick : ms;
}
