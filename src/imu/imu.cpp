#include <SparkFun_BNO080_Arduino_Library.h>

#include "imu.h"
#include "imu_attitude.h"
#include "imu_compass.h"
#include "../common.h"
#include "../fonts/EurostileLTProUnicodeDemi24.h"
#include "../sleep/sleep.h"
#include "../screen/lv_xiao_round_screen.h"

BNO080 bno085;

bool imuInitialized = false;
uint8_t millisPerReading = 50;
uint8_t missedReadings = 0;
bool justReset = true;

euler_t ypr;

void setupIMU() {
  unsigned long start = millis();
#ifdef XIAO_ESP32S3
  while (!bno085.begin(BNO085_ADDR)) {
#endif
#ifdef QTPY_ESP32S3
  while (!bno085.begin(BNO085_ADDR, &Wire1)) {
#endif
    Serial.println("Failed to find BNO08x chip");
    if (millis() - start > 1000) {
      return;
    }
    delay(10);
  }
  Serial.println("BNO08x Found!");
  bno085.enableRotationVector(millisPerReading);
  imuInitialized = true;
}

void resetIMU() {
  bno085.modeSleep();
  delay(10);
  imuInitialized = false;
  missedReadings = 0;
  justReset = true;
  setupIMU();
}

void sleepIMU() {
  if (imuInitialized) {
    bno085.modeSleep();
  }
  delay(10);
}

void imuInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, imu_instrument_t instr_type) {
  if (!imuInitialized) {
    setupIMU();
    if (!imuInitialized) {
      return;
    }
  }
  switch (instr_type) {
    case ATTITUDE:
      setupAttitude();
      break;
    case COMPASS:
      setupCompass();
      break;
  }

  unsigned long millisNow;
  unsigned long millisPrevious = millis();
  unsigned long millisBacklight = millis();

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi24);
  hlpr->loadFont(EurostileLTProDemi24);

  // The background colour will be read during the character rendering
  spr->setTextColor(COLOR_FG);
  hlpr->setTextColor(COLOR_FG);
  hlpr->setTextDatum(MC_DATUM);

  while (true) {
    millisNow = millis();
    if (millisNow - millisPrevious >= millisPerReading) {
      millisPrevious += millisPerReading;

      if (bno085.dataAvailable()) {
        justReset = false;
        quaternionToEuler(bno085.getQuatReal(), bno085.getQuatI(), bno085.getQuatJ(), bno085.getQuatK(), &ypr);
        switch (instr_type) {
          case ATTITUDE:
            drawAttitude(spr, ypr.pitch, ypr.roll);
            break;
          case COMPASS:
            drawCompass(spr, hlpr, -ypr.yaw);
            break;
        }
        if ((millis() - millisBacklight) >= 1000) {
          handleBacklight(100);
          millisBacklight = millis();
        }
      } else {
        missedReadings++;
        if (missedReadings > 50 || ((!justReset) && (missedReadings > 20))) {
          resetIMU();
        }
      }
    }
    int8_t click = clickType(3);
    if (click == 1) {
      break;
    } else if (click == 2) {
      if (imuInitialized) {
        resetIMU();
      }
    } else if (click == 3) {
      goToSleep();
    }
    checkSleep();
  }
}

void mmToPx(float x, float y, float *xp, float *yp, float roll) {
  float _sin = sin((roll)*DEG2RAD);
  float _cos = cos((roll)*DEG2RAD);
  float _x = MM2PX * x;
  float _y = MM2PX * y;
  *xp = (_x * _cos) - (_y * _sin);
  *yp = (_y * _cos) + (_x * _sin);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *_data) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  _data->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) / DEG2RAD;
  _data->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) / DEG2RAD;
  _data->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) / DEG2RAD;
}

float getAngle(float x_c, float y_c, float x, float y) {
  return atan2(y - y_c, x - x_c) / DEG2RAD;
}
