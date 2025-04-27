/**
 * Methods for handling sleep mode.
 */
#include <driver/rtc_io.h>

#include "sleep.h"
#include "../pins.h"
#include "../imu/imu.h"
#include "../display/lv_xiao_round_screen.h"

unsigned long lastLinTime;

/**
 * Put the microcontroller to deep sleep.
 */
void goToSleep()
{
  sleepIMU();
  screenSleep();

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

/**
 * Cleanup sleep pin assignments after waking up from sleep.
 */
void wakeupCleanup()
{
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)XIAO_BL);

  gpio_hold_dis((gpio_num_t)VEHICLE_LIN);
  gpio_pullup_dis((gpio_num_t)VEHICLE_LIN);

  rtc_gpio_pulldown_dis((gpio_num_t)WAKEUP_PIN);
  rtc_gpio_deinit((gpio_num_t)WAKEUP_PIN);
}

/**
 * ISR to update the last activity time on vehicle's LIN bus.
 */
void updateLinTime()
{
  lastLinTime = millis();
}

/**
 * Check whether the elapsed time since last LIN activity is greather thatn `LIN_OFF_TIME` and go
 * to sleep if it is.
 */
void checkSleep()
{
  if (millis() - lastLinTime > LIN_OFF_TIME)
  {
    goToSleep();
  }
}

/**
 * Attach ISR for checking activity on vehicle's LIN bus.
 */
void attachSleepInterrupt()
{
  pinMode(VEHICLE_LIN, INPUT);
  pinMode(WAKEUP_PIN, INPUT);
  attachInterrupt(VEHICLE_LIN, updateLinTime, RISING);
}
