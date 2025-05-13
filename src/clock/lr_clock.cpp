/**
 * Land Rover LR4 Clock.
 *
 * I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
 * RTC Calibration: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#off-line-manual-calibration-of-the-rtc
 * Get RTC Time: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#get-rtc-time
 */
#include <I2C_BM8563.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <SoftwareLin.h>

#include "lr_clock.h"
#include "../common.h"
#include "../pins.h"
#include "../screen.h"
#include "../display/lv_xiao_round_screen.h"
#include "../fonts/EurostileLTProUnicodeDemi48.h"
#include "../sleep/sleep.h"

uint16_t color_bg = COLOR_BG;
uint16_t color_fg = COLOR_FG;
uint16_t color_mh = COLOR_MH;
uint16_t color_hh = COLOR_HH;

float time_secs = 0;
bool clockInitialized = false;
I2C_BM8563_TimeTypeDef timeStruct;
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
SoftwareLin swLin(VEHICLE_LIN, -1);
bool timeInitialized = false;

/**
 * Update `time_secs` variable with seconds since midnight from the RTC.
 */
void syncTime(bool get_time = true)
{
  if (get_time)
  {
    rtc.getTime(&timeStruct);
  }
  time_secs = (timeStruct.hours * 3600) + (timeStruct.minutes * 60) + timeStruct.seconds;
  // Midnight roll-over
  if (time_secs >= 86400)
  {
    time_secs -= 86400;
  }
}

/**
 * Draw triangular buttons used to indicated areas of screen to press for manually setting time.
 */
void renderButtons(TFT_eSprite *spr)
{
  uint16_t button_color = 0xAD55;
  spr->fillTriangle(
      CARD_C - SETTING_LATERAL_OFFSET,
      CARD_C - SETTING_BASE_OFFSET - SETTING_H_HEIGHT,
      CARD_C - SETTING_LATERAL_OFFSET - (SETTING_WIDTH / 2),
      CARD_C - SETTING_BASE_OFFSET,
      CARD_C - SETTING_LATERAL_OFFSET + (SETTING_WIDTH / 2),
      CARD_C - SETTING_BASE_OFFSET,
      button_color);
  spr->fillTriangle(
      CARD_C - SETTING_LATERAL_OFFSET,
      CARD_C + SETTING_BASE_OFFSET + SETTING_H_HEIGHT,
      CARD_C - SETTING_LATERAL_OFFSET - (SETTING_WIDTH / 2),
      CARD_C + SETTING_BASE_OFFSET,
      CARD_C - SETTING_LATERAL_OFFSET + (SETTING_WIDTH / 2),
      CARD_C + SETTING_BASE_OFFSET,
      button_color);
  spr->fillTriangle(
      CARD_C + SETTING_LATERAL_OFFSET,
      CARD_C - SETTING_BASE_OFFSET - SETTING_M_HEIGHT,
      CARD_C + SETTING_LATERAL_OFFSET + (SETTING_WIDTH / 2),
      CARD_C - SETTING_BASE_OFFSET,
      CARD_C + SETTING_LATERAL_OFFSET - (SETTING_WIDTH / 2),
      CARD_C - SETTING_BASE_OFFSET,
      button_color);
  spr->fillTriangle(
      CARD_C + SETTING_LATERAL_OFFSET,
      CARD_C + SETTING_BASE_OFFSET + SETTING_M_HEIGHT,
      CARD_C + SETTING_LATERAL_OFFSET + (SETTING_WIDTH / 2),
      CARD_C + SETTING_BASE_OFFSET,
      CARD_C + SETTING_LATERAL_OFFSET - (SETTING_WIDTH / 2),
      CARD_C + SETTING_BASE_OFFSET,
      button_color);
}

/**
 * Draw clock face, i.e. ticks and numbers.
 */
void renderFace(TFT_eSprite *spr)
{
  spr->fillSprite(color_bg);

  // Text offset adjustment
  constexpr uint32_t dialRadius = CARD_R - 24;
  constexpr uint32_t hOffset = 4;

  float xp = 0.0, yp = 0.0; // Use float pixel position for smooth AA motion
  float xt = 0.0, yt = 0.0; // Use float pixel position for smooth AA motion

  uint8_t tickPositions[] = {1, 2, 4, 5, 7, 8, 10, 11};
  for (uint8_t h = 0; h < sizeof tickPositions / sizeof tickPositions[0]; h++)
  {
    getCoord(CARD_C, CARD_C, &xp, &yp, CARD_R - TICK_HEIGHT, 30 * tickPositions[h]);
    getCoord(CARD_C, CARD_C, &xt, &yt, CARD_R + TICK_HEIGHT + 2, 30 * tickPositions[h]);
    spr->drawWideLine(xp, yp, xt, yt, TICK_WIDTH, color_fg);
    spr->fillSmoothCircle(CARD_C, CARD_C, CARD_R - TICK_HEIGHT, color_bg);
  }

  spr->drawNumber(1, CARD_C - 14, CARD_C - dialRadius + 6);
  spr->drawNumber(2, CARD_C + 14, CARD_C - dialRadius + 6);
  spr->drawNumber(3, CARD_C + dialRadius + hOffset, CARD_C + 2);
  spr->drawNumber(6, CARD_C, CARD_C + dialRadius + 2 + 4);
  spr->drawNumber(9, CARD_C - dialRadius - hOffset, CARD_C + 2);
}

/**
 * Draw clock hands at the number of seconds since midnight.
 */
void renderHands(TFT_eSprite *hlpr)
{
  float h_angle = time_secs * HOUR_ANGLE;
  float m_angle = time_secs * MINUTE_ANGLE;

  // The hands are completely redrawn - this can be done quickly
  hlpr->fillSprite(color_bg);

  float xp = 0.0, yp = 0.0; // Use float pixel position for smooth AA motion

  // Draw hour hand
  getCoord(CARD_C, CARD_C, &xp, &yp, H_HAND_LENGTH, h_angle - 90);
  hlpr->drawWedgeLine(CARD_C, CARD_C, xp, yp, 10.0, 6.0, color_hh);
  hlpr->drawArc(CARD_C, CARD_C, H_HAND_LENGTH + 8, H_HAND_LENGTH, 0, 360, color_bg, 0x00FFFFFF);

  // Draw minute hand
  getCoord(CARD_C, CARD_C, &xp, &yp, M_HAND_LENGTH, m_angle - 90);
  hlpr->drawWedgeLine(CARD_C, CARD_C, xp, yp, 8.0, 4.0, color_mh);
  hlpr->drawArc(CARD_C, CARD_C, M_HAND_LENGTH + 8, M_HAND_LENGTH, 0, 360, color_bg, 0x00FFFFFF);

  // Draw the central pivot circle
  hlpr->fillSmoothCircle(CARD_C, CARD_C, 21, COLOR_PIVOT);
}

/**
 * Draw entire clock at number of seconds since midnight.
 */
void drawClock(TFT_eSprite *spr, TFT_eSprite *hlpr, bool setting_mode = false)
{
  spr->setTextColor(color_fg, color_bg); // TODO: evaluate need for background here
  renderFace(spr);
  if (setting_mode)
  {
    renderButtons(spr);
  }
  renderHands(hlpr);
  hlpr->pushToSprite(spr, 0, 0, color_bg);
  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}

/**
 * Get the current time from the vehicle's LIN bus.
 *
 * Sample message:
 * ID  Data . . . . .                      Checksum
 * F0  01  2D  51  00                      80
 *                 ^^ Unused?
 *             ^^ This appears to be hours beginning at 40, i.e. decimal 64. In our case 51 would be hour 17. It is the same in both 12 and 24 hour mode.
 *         ^^ Minutes, i.e. 2D = 45 minutes
 *     ^^ Seconds, i.e. 01 = 1 second
 *
 * Additionally, the LIN bus voltage appears to never exceed 13.2 V, even when the battery is at 14.5 V.
 */
void getTimeFromVehicle(bool force = false, uint32_t timeout = 10000)
{
  if ((!force) && timeInitialized)
  {
    return;
  }

  detachInterrupt(VEHICLE_LIN);
  swLin.begin(LIN_BAUD_MAX);
  unsigned long start = millis();
  while (true)
  {
    if (millis() - start > 10000)
    {
      swLin.end();
      break;
    }
    const int frame_data_bytes = 4;

    uint8_t buf[2 + frame_data_bytes]; // 2 bytes for PID and CHECKSUM. !!! The SYNC is consumed by swLin.setAutoBaud()

    // sw_lin.checkBreak() blocks until UART ISR gives the semaphore.
    // I think I've made adequate changes so checkBreak() is no longer blocking
    if (swLin.checkBreak(timeout))
    {

      const uint32_t commonBaud[] = {9597, 9600, 9615};
      uint32_t autobaud = swLin.setAutoBaud(commonBaud, sizeof(commonBaud) / sizeof(commonBaud[0]), timeout);

      const int read_timeout = 100000; // 100ms timeout
      int start_time = micros();

      int bytes_to_read = sizeof(buf) / sizeof(buf[0]);
      int bytes_read = 0;
      while ((bytes_read < bytes_to_read) && ((micros() - start_time) <= read_timeout))
      {
        bytes_read += swLin.read(buf + bytes_read, bytes_to_read - bytes_read);
        delay(0); // yield for other tasks
      }
      swLin.endFrame();

      if (bytes_read < bytes_to_read)
      {
        // Serial.printf("Timeout: only %d bytes is read\n", bytes_read);
        continue;
      }
      // Serial.printf("ID: %02X\n", buf[0]);
      if (buf[0] != 0xF0)
      {
        continue;
      }
      uint8_t chk = buf[1] + buf[2] + buf[3] + buf[4];
      if (chk + buf[5] != 0xFF)
      {
        // Serial.print("Data is invalid, got Checksum: ");
        // Serial.print(buf[5]);
        // Serial.print(". Expected: ");
        // Serial.println(0xFF - chk);
        continue;
      }
      for (int i = 0; i < bytes_to_read; ++i)
      {
        Serial.printf("0x%02X ", buf[i]);
      }
      Serial.print("\nHour: ");
      Serial.println(buf[HOUR_BYTE] & HOUR_MASK);
      Serial.print("Minute: ");
      Serial.println(buf[MIN_BYTE]);
      Serial.print("Second: ");
      Serial.println(buf[SEC_BYTE]);
      Serial.println();
      timeStruct.hours = buf[HOUR_BYTE] & HOUR_MASK;
      timeStruct.minutes = buf[MIN_BYTE];
      timeStruct.seconds = buf[SEC_BYTE];
      rtc.setTime(&timeStruct);
      timeInitialized = true;
      swLin.end();
      break;
    }
  }
  attachSleepInterrupt();
}

/**
 * Increment time by specified amount, staying within the bounds of [0, max_time].
 */
uint8_t incrementTime(uint8_t old_time, uint8_t max_time, int8_t _inc)
{
  int16_t new_time = old_time + _inc;
  if (new_time < 0)
  {
    new_time += (max_time + 1);
  }
  else if (new_time > max_time)
  {
    new_time -= (max_time + 1);
  }
  return (uint8_t)new_time;
}

/**
 * Enter GUI time setting mode, where buttons are rendered on screen and touches are read to
 * update the seconds and hours.
 */
void getTimeFromFingers(TFT_eSprite *spr, TFT_eSprite *hlpr)
{
  Serial.println("Setting time.");
  uint8_t touch_padding = 10;

  lv_indev_state_t prev_state = LV_INDEV_STATE_REL;
  lv_indev_data_t data;
  unsigned long last_touch = millis();

  while (millis() - last_touch < 10000)
  {
    syncTime(false);
    drawClock(spr, hlpr, true);
    readScreen(&data);
    if (data.state != prev_state)
    {
      prev_state = data.state;
      if (data.state == LV_INDEV_STATE_REL)
      {
        if ((data.point.x < CARD_C - touch_padding) && (data.point.y < CARD_C - touch_padding))
        {
          timeStruct.hours = incrementTime(timeStruct.hours, 23, 1);
        }
        else if ((data.point.x < CARD_C - touch_padding) && (data.point.y > CARD_C + touch_padding))
        {
          timeStruct.hours = incrementTime(timeStruct.hours, 23, -1);
        }
        else if ((data.point.x > CARD_C + touch_padding) && (data.point.y < CARD_C - touch_padding))
        {
          timeStruct.minutes = incrementTime(timeStruct.minutes, 59, 1);
        }
        else if ((data.point.x > CARD_C + touch_padding) && (data.point.y > CARD_C + touch_padding))
        {
          timeStruct.minutes = incrementTime(timeStruct.minutes, 59, -1);
        }
      }
    }
    if (data.state == LV_INDEV_STATE_PR)
    {
      last_touch = millis();
    }
  }

  rtc.setTime(&timeStruct);
}

/**
 * Activate the clock instrument. This is the entry point which should be called from the main
 * instrument.
 */
void clockInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, bool no_lin)
{
  if (!clockInitialized)
  {
    rtc.begin();
    clockInitialized = true;
  }

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi48);
  spr->setTextDatum(MC_DATUM);
  if (handleBacklight(100))
  {
    color_bg = COLOR_BG_NIGHT;
    color_fg = COLOR_FG_NIGHT;
    color_mh = COLOR_MH_NIGHT;
    color_hh = COLOR_HH_NIGHT;
  }
  else
  {
    color_bg = COLOR_BG;
    color_fg = COLOR_FG;
    color_mh = COLOR_MH;
    color_hh = COLOR_HH;
  }

  syncTime();
  drawClock(spr, hlpr);
  if (!no_lin)
  {
    getTimeFromVehicle();
  }
  unsigned long lastCheckTime = millis();
  unsigned long lastBacklightTime = millis();

  // Time for next tick
  unsigned long previousTime = 0;
  unsigned long currentTime = 0;

  while (true)
  {
    currentTime = millis();
    if ((!no_lin) && (!clockInitialized) && (currentTime - lastCheckTime >= 60000))
    {
      getTimeFromVehicle();
      lastCheckTime = millis();
    }
    if (currentTime - lastBacklightTime >= 1000)
    {
      if (handleBacklight(100))
      {
        color_bg = COLOR_BG_NIGHT;
        color_fg = COLOR_FG_NIGHT;
        color_mh = COLOR_MH_NIGHT;
        color_hh = COLOR_HH_NIGHT;
      }
      else
      {
        color_bg = COLOR_BG;
        color_fg = COLOR_FG;
        color_mh = COLOR_MH;
        color_hh = COLOR_HH;
      }
      lastBacklightTime = currentTime;
    }
    if (currentTime - previousTime >= 100)
    {
      // Update next tick time in 100 milliseconds for smooth movement
      previousTime = currentTime;
      syncTime();
      drawClock(spr, hlpr);
    }
    int8_t click = clickType(3);
    if (click == 1)
    {
      break;
    }
    else if (click == -1)
    {
      getTimeFromFingers(spr, hlpr);
    }
    else if ((!no_lin) && (click == 2))
    {
      getTimeFromVehicle(true);
    }
    else if (click == 3)
    {
      goToSleep();
    }
    checkSleep();
  }
}
