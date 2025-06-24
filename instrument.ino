/**
 * Touch: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#touch-function
 */
#include <Arduino.h>
#include <Wire.h>

#include "src/common.h"
#include "src/pins.h"
#include "src/screen.h"
#include "src/display/lv_xiao_round_screen.h"
#include "src/sleep/sleep.h"
#include "src/imu/background.h"
#include "src/imu/imu.h"
#include "src/clock/lr_clock.h"

TFT_eSPI *_tft = getTft();
TFT_eSprite instrument = TFT_eSprite(_tft);
TFT_eSprite helper = TFT_eSprite(_tft);
TFT_eSprite small_helper = TFT_eSprite(_tft);

uint8_t instr = 0;
uint8_t instr_prev = -1;

// TODO: Generally add error handling for instruments and external calls

/**
 * Task to run on second core.
 */
void backgroundTask(void *)
{
  Serial.println("starting background core");
  imuTask();
}

/**
 * Arduino setup function, called on startup and wakeup from sleep.
 */
void setup()
{
  Serial.begin(115200);
  unsigned long start = millis();
  while ((!Serial) && ((millis() - start) < 1000))
  {
    delay(10);
  }
  Serial.println("Booting...");

  wakeupCleanup();
  attachSleepInterrupt();
  xiao_disp_init();
  instrument.createSprite(CARD_SIZE, CARD_SIZE);
  helper.createSprite(CARD_SIZE, CARD_SIZE);
  small_helper.createSprite(HELPER_W, HELPER_H);

  pinMode(TOUCH_INT, INPUT_PULLUP);
  pinMode(VEHICLE_BACKLIGHT, INPUT);
  handleBacklight(100);

  Wire.begin();
#ifdef QTPY_ESP32S3
  Wire1.begin();
#endif
  WIRE_PORT.setClock(400000);
  WIRE_PORT.setTimeout(4);

  xTaskCreatePinnedToCore(
      backgroundTask,   // Function to implement the task
      "backgroundTask", // Name of the task
      6000,            // Stack size in words
      NULL,             // Task input parameter
      1,                // Priority of the task
      NULL,             // Task handle.
      1                 // Core where the task should run
  );
}

/**
 * Arduino loop.
 */
void loop()
{
  if (instr == 0)
  {
    clockInstrument(&instrument, &helper, false);
  }
  else if (instr == 1)
  {
    imuInstrument(&instrument, &helper, &small_helper, ATTITUDE);
  }
  else if (instr == 2)
  {
    imuInstrument(&instrument, &helper, &small_helper, COMPASS);
  }
  delay(200);
  instr++;
  if (instr > 2)
  {
    instr = 0;
  }
}
