/* Sleep. */

// TODO: Enter sleep after some amount of inactivity on LIN bus
// TODO: Wake up on LIN activating.

#include "driver/rtc_io.h"

void goToSleep() {
  rtc_gpio_pullup_dis(VEHICLE_LIN);
  rtc_gpio_pulldown_en(VEHICLE_LIN);
  esp_sleep_enable_ext0_wakeup(VEHICLE_LIN, 1);
  esp_deep_sleep_start();
}
