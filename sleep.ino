/*
Sleep.
*/

#include "driver/rtc_io.h"
#include "sleep.h"

unsigned long lastLinTime;

void goToSleep() {
  digitalWrite(XIAO_BL, LOW);
  rtc_gpio_hold_en((gpio_num_t)XIAO_BL);
  rtc_gpio_pullup_dis((gpio_num_t)WAKEUP_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)WAKEUP_PIN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 1);
  esp_deep_sleep_start();
}

void wakeupCleanup() {
  rtc_gpio_hold_dis((gpio_num_t)XIAO_BL);
  rtc_gpio_deinit((gpio_num_t)XIAO_BL);
  rtc_gpio_deinit((gpio_num_t)WAKEUP_PIN);
}

void updateLinTime() {
  lastLinTime = millis();
}

void checkSleep() {
  return; // TODO
  if (millis() - lastLinTime > LIN_OFF_TIME) { goToSleep(); }
}
