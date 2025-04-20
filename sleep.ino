/*
Sleep.
*/

#include "driver/rtc_io.h"
#include "sleep.h"

unsigned long lastLinTime;

void goToSleep() {
  if (attitudeInitialized) {
    bno085.modeSleep();
  }
  delay(10);
  tft.writecommand(0x10);
  delay(10);

  digitalWrite(XIAO_BL, LOW);
  gpio_hold_en((gpio_num_t)XIAO_BL);

  detachInterrupt(VEHICLE_LIN);
  gpio_pulldown_dis((gpio_num_t)VEHICLE_LIN);
  gpio_pullup_en((gpio_num_t)VEHICLE_LIN);
  gpio_hold_en((gpio_num_t)VEHICLE_LIN);

  rtc_gpio_pullup_dis((gpio_num_t)WAKEUP_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)WAKEUP_PIN);

  gpio_deep_sleep_hold_en();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN, 1);
  esp_deep_sleep_start();
}

void wakeupCleanup() {
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)XIAO_BL);

  gpio_hold_dis((gpio_num_t)VEHICLE_LIN);
  gpio_pullup_dis((gpio_num_t)VEHICLE_LIN);

  rtc_gpio_pulldown_dis((gpio_num_t)WAKEUP_PIN);
  rtc_gpio_deinit((gpio_num_t)WAKEUP_PIN);
}

void updateLinTime() {
  lastLinTime = millis();
}

void checkSleep() {
  if (millis() - lastLinTime > LIN_OFF_TIME) { goToSleep(); }
}

void attachSleepInterrupt() {
  pinMode(VEHICLE_LIN, INPUT);
  pinMode(WAKEUP_PIN, INPUT);
  attachInterrupt(VEHICLE_LIN, updateLinTime, RISING);
}
