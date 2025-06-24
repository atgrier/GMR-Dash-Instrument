/**
 * IMU instruments.
 */
#include "imu.h"
#include "imu_attitude.h"
#include "imu_compass.h"
#include "background.h"
#include "../common.h"
#include "../screen.h"
#include "../fonts/EurostileLTProUnicodeDemi24.h"
#include "../sleep/sleep.h"

uint8_t millisPerReading = 100;

/**
 * Activate the IMU instrument for either attitude or compass mode. This is the entry point which
 * should be called from the main instrument.
 */
void imuInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, TFT_eSprite *word_hlpr, imu_instrument_t instr_type)
{
  setupIMU();
  if (!imuReady())
  {
    return;
  }
  switch (instr_type)
  {
  case ATTITUDE:
    setupAttitude();
    break;
  case COMPASS:
    setupCompass(hlpr);
    break;
  }

  unsigned long millisBacklight = millis();

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi24);
  word_hlpr->loadFont(EurostileLTProDemi24);

  // The background colour will be read during the character rendering
  spr->setTextColor(COLOR_FG);
  word_hlpr->setTextColor(COLOR_FG);
  word_hlpr->setTextDatum(MC_DATUM);

  unsigned long millisPrevious = millis();
  euler_t *ypr;

  while (true)
  {
    if (millis() - millisPrevious >= millisPerReading)
    {
      millisPrevious += millisPerReading;
      ypr = getData();
      Serial.print("Pitch: ");
      Serial.print(ypr->pitch);
      Serial.print(" Roll: ");
      Serial.print(ypr->roll);
      Serial.print(" Yaw: ");
      Serial.println(ypr->yaw);
      switch (instr_type)
      {
      case ATTITUDE:
        drawAttitude(spr, ypr->roll, ypr->pitch);
        break;
      case COMPASS:
        drawCompass(spr, hlpr, word_hlpr, ypr->yaw); // TODO: Calibrate compass
        break;
      }
      if ((millis() - millisBacklight) >= 1000)
      {
        handleBacklight(100);
        millisBacklight = millis();
      }
    }
    int8_t click = clickType(3);
    if (click == 1)
    {
      return;
    }
    else if (click == 2)
    {
      resetIMU();
    }
    else if (click == 3)
    {
      goToSleep();
    }
    checkSleep();
  }
}
