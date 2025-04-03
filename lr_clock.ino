/*
I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
RTC Calibration: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#off-line-manual-calibration-of-the-rtc
Get RTC Time: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#get-rtc-time
*/
#include <Wire.h>
#include "src/SoftwareLin/SoftwareLin.h"
#include "fonts/EurostileLTProUnicodeDemi48.h"
#include "common.h"
#include "sleep.h"
#include "lin.h"

#define COLOR_PIVOT 0x18C3
#define COLOR_MH 0xF7BE
#define COLOR_HH 0xDEFB
#define COLOR_MH_NIGHT 0x7F4E
#define COLOR_HH_NIGHT 0x772C

uint16_t color_fg = COLOR_FG;
uint16_t color_mh = COLOR_MH;
uint16_t color_hh = COLOR_HH;

#define M_HAND_LENGTH 110.0f
#define H_HAND_LENGTH 86.0f
#define TICK_HEIGHT 26.0f
#define TICK_WIDTH 5.0f

// Calculate 1 second increment angles. Hours and minute hand angles
// change every second so we see smooth sub-pixel movement
#define SECOND_ANGLE (360.0 / 60.0)
#define MINUTE_ANGLE (SECOND_ANGLE / 60.0)
#define HOUR_ANGLE (MINUTE_ANGLE / 12.0)

float time_secs = 0;
bool clockInitialized = false;
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
SoftwareLin swLin(VEHICLE_LIN, -1);
bool timeInitialized = false;

void clockInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr) {
  if (!clockInitialized) {
    rtc.begin();
    clockInitialized = true;
  }

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi48);
  spr->setTextDatum(MC_DATUM);

  syncTime();
  drawClock(spr, hlpr);
  getTimeFromVehicle();

  // Time for next tick
  unsigned long previousTime = 0;
  unsigned long currentTime = 0;

  while (true) {
    currentTime = millis();
    if (currentTime - previousTime >= 100) {
      // Update next tick time in 100 milliseconds for smooth movement
      previousTime = currentTime;

      // Midnight roll-over
      syncTime();
      if (time_secs >= 86400) {
        time_secs -= 86400;
      }

      drawClock(spr, hlpr);
      if (handleBacklight(100)) {
        color_fg = COLOR_FG_NIGHT;
        color_mh = COLOR_MH_NIGHT;
        color_hh = COLOR_HH_NIGHT;
      } else {
        color_fg = COLOR_FG;
        color_mh = COLOR_MH;
        color_hh = COLOR_HH;
      }
    }
    int8_t click = clickType(2);
    if (click == 1) {
      break;
    } else if (click == 2) {
      getTimeFromVehicle(true);
    }
    checkSleep();
  }
}

void drawClock(TFT_eSprite *spr, TFT_eSprite *hlpr) {
  spr->setTextColor(color_fg, COLOR_BG);  // TODO: evaluate need for background here
  renderFace(spr);
  renderHands(hlpr);
  hlpr->pushToSprite(spr, 0, 0, COLOR_BG);
  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}

void renderFace(TFT_eSprite *spr) {
  spr->fillSprite(COLOR_BG);

  // Text offset adjustment
  constexpr uint32_t dialRadius = CARD_R - 24;
  constexpr uint32_t hOffset = 4;

  float xp = 0.0, yp = 0.0;  // Use float pixel position for smooth AA motion
  float xt = 0.0, yt = 0.0;  // Use float pixel position for smooth AA motion

  uint8_t tickPositions[] = { 1, 2, 4, 5, 7, 8, 10, 11 };
  for (uint8_t h = 0; h < sizeof tickPositions / sizeof tickPositions[0]; h++) {
    getCoord(CARD_C, CARD_C, &xp, &yp, CARD_R - TICK_HEIGHT, 30 * tickPositions[h]);
    getCoord(CARD_C, CARD_C, &xt, &yt, CARD_R + TICK_HEIGHT + 2, 30 * tickPositions[h]);
    spr->drawWideLine(xp, yp, xt, yt, TICK_WIDTH, color_fg);
    spr->fillSmoothCircle(CARD_C, CARD_C, CARD_R - TICK_HEIGHT, COLOR_BG);
  }

  spr->drawNumber(1, CARD_C - 14, CARD_C - dialRadius + 6);
  spr->drawNumber(2, CARD_C + 14, CARD_C - dialRadius + 6);
  spr->drawNumber(3, CARD_C + dialRadius + hOffset, CARD_C + 2);
  spr->drawNumber(6, CARD_C, CARD_C + dialRadius + 2 + 4);
  spr->drawNumber(9, CARD_C - dialRadius - hOffset, CARD_C + 2);
}

void renderHands(TFT_eSprite *hlpr) {
  float h_angle = time_secs * HOUR_ANGLE;
  float m_angle = time_secs * MINUTE_ANGLE;

  // The hands are completely redrawn - this can be done quickly
  hlpr->fillSprite(COLOR_BG);

  float xp = 0.0, yp = 0.0;  // Use float pixel position for smooth AA motion

  // Draw hour hand
  getCoord(CARD_C, CARD_C, &xp, &yp, H_HAND_LENGTH, h_angle - 90);
  hlpr->drawWedgeLine(CARD_C, CARD_C, xp, yp, 10.0, 6.0, color_hh);
  hlpr->drawArc(CARD_C, CARD_C, H_HAND_LENGTH + 8, H_HAND_LENGTH, 0, 360, COLOR_BG, 0x00FFFFFF);

  // Draw minute hand
  getCoord(CARD_C, CARD_C, &xp, &yp, M_HAND_LENGTH, m_angle - 90);
  hlpr->drawWedgeLine(CARD_C, CARD_C, xp, yp, 8.0, 4.0, color_mh);
  hlpr->drawArc(CARD_C, CARD_C, M_HAND_LENGTH + 8, M_HAND_LENGTH, 0, 360, COLOR_BG, 0x00FFFFFF);

  // Draw the central pivot circle
  hlpr->fillSmoothCircle(CARD_C, CARD_C, 21, COLOR_PIVOT);
}

/*
Sample message:
ID  Data . . . . .                      Checksum
F0  01  2D  51  00                      80
                ^^ Unused?
            ^^ This appears to be hours beginning at 40, i.e. decimal 64. In our case 51 would be hour 17. It is the same in both 12 and 24 hour mode.
        ^^ Minutes, i.e. 2D = 45 minutes
    ^^ I supect this is seconds, it seems to have reset every time I changed something

*/
void syncTime(void) {
  rtc.getTime(&timeStruct);
  time_secs = (timeStruct.hours * 3600) + (timeStruct.minutes * 60) + timeStruct.seconds;
}

void getTimeFromVehicle(bool force, uint32_t timeout) {
  if ((!force) && timeInitialized) {
    return;
  }

  detachInterrupt(VEHICLE_LIN);
  swLin.begin(LIN_BAUD_MAX);
  unsigned long start = millis();
  while (true) {
    if (millis() - start > 10000) {
      swLin.end();
      break;
    }
    const int frame_data_bytes = 4;

    uint8_t buf[2 + frame_data_bytes];  // 2 bytes for PID and CHECKSUM. !!! The SYNC is consumed by swLin.setAutoBaud()

    // sw_lin.checkBreak() blocks until UART ISR gives the semaphore.
    // I think I've made adequate changes so checkBreak() is no longer blocking
    if (swLin.checkBreak(timeout)) {

      const uint32_t commonBaud[] = { 9597, 9600, 9615 };
      uint32_t autobaud = swLin.setAutoBaud(commonBaud, sizeof(commonBaud) / sizeof(commonBaud[0]), timeout);

      const int read_timeout = 100000;  // 100ms timeout
      int start_time = micros();

      int bytes_to_read = sizeof(buf) / sizeof(buf[0]);
      int bytes_read = 0;
      while ((bytes_read < bytes_to_read) && ((micros() - start_time) <= read_timeout)) {
        bytes_read += swLin.read(buf + bytes_read, bytes_to_read - bytes_read);
        delay(0);  // yield for other tasks
      }
      swLin.endFrame();

      if (bytes_read < bytes_to_read) {
        // Serial.printf("Timeout: only %d bytes is read\n", bytes_read);
        continue;
      }
      // Serial.println(buf[0]);
      if (buf[0] != 0xF0) {
        continue;
      }
      uint8_t chk = buf[1] + buf[2] + buf[3] + buf[4];
      if (chk + buf[5] != 0xFF) {
        // Serial.print("Data is invalid, got Checksum: ");
        // Serial.print(buf[5]);
        // Serial.print(". Expected: ");
        // Serial.println(0xFF - chk);
        continue;
      }
      for (int i = 0; i < bytes_to_read; ++i) {
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
