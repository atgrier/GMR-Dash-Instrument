/**
 * IMU instruments.
 */
#include <SparkFun_BNO080_Arduino_Library.h>

#include "imu.h"
#include "imu_attitude.h"
#include "imu_compass.h"
#include "../common.h"
#include "../pins.h"
#include "../screen.h"
#include "../fonts/EurostileLTProUnicodeDemi24.h"
#include "../sleep/sleep.h"

BNO080 bno085;

bool imuInitialized = false;
uint8_t millisPerReading = 50;
uint8_t missedReadings = 0;
bool justReset = true;

euler_t ypr;

/**
 * Initialize the BNO085 IMU.
 *
 * Orientation Modes:
 * - Orientation Vector
 *   - Susceptible to large errors due to vehicle's acceleraion, i.e. it doesn't seem to adequately separate acceleration due to gravity to other sources
 * - AR/VR-Stabilized Orientation Vector
 *   - Total errors are smaller, but it very quickly gets off, especially when turning, and takes a long time once stationary/non-accelerating to reset
 * - Gyro (Integrated) Rotation Vector
 *   - This is sometimes better, but sometimes much worse
 */
void setupIMU()
{
  unsigned long start = millis();
#if defined(XIAO_ESP32S3)
  while (!bno085.begin(BNO085_ADDR))
  {
#elif defined(QTPY_ESP32S3)
  while (!bno085.begin(BNO085_ADDR, &Wire1))
  {
#else
#error "One of XIAO_ESP32S3 or QTPY_ESP32S3 must be defined"
#endif
    Serial.println("Failed to find BNO08x chip");
    if (millis() - start > 1000)
    {
      return;
    }
    delay(10);
  }
  Serial.println("BNO08x Found!");
  bno085.enableRotationVector(millisPerReading);
  imuInitialized = true;
}

/**
 * Re-initialze the BNO085 IMU, e.g. for when it stops providing data.
 */
void resetIMU()
{
  bno085.modeSleep();
  delay(10);
  imuInitialized = false;
  missedReadings = 0;
  justReset = true;
  setupIMU();
}

/**
 * Put BNO085 IMU to sleep.
 */
void sleepIMU()
{
  if (imuInitialized)
  {
    bno085.modeSleep();
  }
  delay(10);
}

/**
 * Activate the IMU instrument for either attitude or compass mode. This is the entry point which
 * should be called from the main instrument.
 */
void imuInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, TFT_eSprite *word_hlpr, imu_instrument_t instr_type)
{
  if (!imuInitialized)
  {
    setupIMU();
    if (!imuInitialized)
    {
      return;
    }
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

  unsigned long millisNow;
  unsigned long millisPrevious = millis();
  unsigned long millisBacklight = millis();

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi24);
  word_hlpr->loadFont(EurostileLTProDemi24);

  // The background colour will be read during the character rendering
  spr->setTextColor(COLOR_FG);
  word_hlpr->setTextColor(COLOR_FG);
  word_hlpr->setTextDatum(MC_DATUM);

  while (true)
  {
    millisNow = millis();
    if (millisNow - millisPrevious >= millisPerReading)
    {
      millisPrevious += millisPerReading;

      if (bno085.dataAvailable())
      {
        justReset = false;
        quaternionToEuler(bno085.getQuatReal(), bno085.getQuatI(), bno085.getQuatJ(), bno085.getQuatK(), &ypr);
        Serial.print("Pitch: ");
        Serial.print(ypr.pitch);
        Serial.print(" Roll: ");
        Serial.print(ypr.roll);
        Serial.print(" Yaw: ");
        Serial.println(ypr.yaw);
        switch (instr_type)
        {
        case ATTITUDE:
          drawAttitude(spr, ypr.pitch, -ypr.roll - 70);  // Pitch and roll are reversed based on the mounting orientation
          break;
        case COMPASS:
          drawCompass(spr, hlpr, word_hlpr, ypr.yaw + 135);  // TODO: Calibrate compass
          break;
        }
        if ((millis() - millisBacklight) >= 1000)
        {
          handleBacklight(100);
          millisBacklight = millis();
        }
      }
      else
      {
        missedReadings++;
        if (missedReadings > 50 || ((!justReset) && (missedReadings > 20)))
        {
          resetIMU();
        }
      }
    }
    int8_t click = clickType(3);
    if (click == 1)
    {
      break;
    }
    else if (click == 2)
    {
      if (imuInitialized)
      {
        resetIMU();
      }
    }
    else if (click == 3)
    {
      goToSleep();
    }
    checkSleep();
  }
}

/**
 * Convert a position in MM to a pixel position, factoring in the bank angle.
 */
void mmToPx(float x, float y, float *xp, float *yp, float roll)
{
  float _sin = sin((roll)*DEG2RAD);
  float _cos = cos((roll)*DEG2RAD);
  float _x = MM2PX * x;
  float _y = MM2PX * y;
  *xp = (_x * _cos) - (_y * _sin);
  *yp = (_y * _cos) + (_x * _sin);
}

/**
 * Get Euler orientation angles from quaternion values.
 */
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *_data)
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  _data->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) / DEG2RAD;
  _data->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) / DEG2RAD;
  _data->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) / DEG2RAD;
}

/**
 * Get arctangent angle of a position (x, y) relative to (x_c, y_c).
 */
float getAngle(float x_c, float y_c, float x, float y)
{
  return atan2(y - y_c, x - x_c) / DEG2RAD;
}
