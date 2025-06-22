/**
 * IMU instruments.
 */
#include <ICM_20948.h>

#include "imu.h"
#include "imu_attitude.h"
#include "imu_compass.h"
#include "../common.h"
#include "../screen.h"
#include "../fonts/EurostileLTProUnicodeDemi24.h"
#include "../sleep/sleep.h"

ICM_20948_I2C icm20948;

bool imuInitialized = false;
// uint8_t millisPerReading = 50;
// uint8_t missedReadings = 0;
// bool justReset = true;

// Quaternion rotation for pitch offset
float qx = sin(PITCH_OFFSET * DEG2RAD / 2.0);
float qy = 0.0;
float qz = 0.0;
float qw = cos(PITCH_OFFSET * DEG2RAD / 2.0);
euler_t ypr;

/**
 * Initialize the ICM-20948 IMU.
 *
 * Orientation Modes:
 */
void setupIMU()
{
  unsigned long start = millis();
  while (!imuInitialized) {
    icm20948.begin(ICM20948_WIRE, ICM20948_ADDR);
    if (icm20948.status == ICM_20948_Stat_Ok) {
      break;
    }
    Serial.println("Failed to find ICM-20948 chip.");
    if (millis() - start > 1000)
    {
      return;
    }
    delay(10);
  }
  Serial.println("ICM-20948 Found.");
  bool success = true;
  while (!imuInitialized) {
    success &= (icm20948.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (icm20948.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Quat9, 4) == ICM_20948_Stat_Ok);
    success &= (icm20948.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (icm20948.enableDMP() == ICM_20948_Stat_Ok);
    success &= (icm20948.resetDMP() == ICM_20948_Stat_Ok);
    success &= (icm20948.resetFIFO() == ICM_20948_Stat_Ok);
    if (success) {
      break;
    }
  }
  Serial.println("ICM-20948 DMP enabled.");
  imuInitialized = true;
}

/**
 * Re-initialze the ICM-20948 IMU, e.g. for when it stops providing data.
 */
void resetIMU()
{
  sleepIMU();
  imuInitialized = false;
  // missedReadings = 0;
  // justReset = true;
  setupIMU();
}

/**
 * Put ICM-20948 IMU to sleep.
 */
void sleepIMU()
{
  if (imuInitialized)
  {
    icm20948.sleep();
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

  // unsigned long millisNow;
  // unsigned long millisPrevious = millis();
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
    // millisNow = millis();
    // if (millisNow - millisPrevious >= millisPerReading)
    // {
      // millisPrevious += millisPerReading;
      icm_20948_DMP_data_t data;
      icm20948.readDMPdataFromFIFO(&data);

      if ((icm20948.status == ICM_20948_Stat_Ok) || (icm20948.status == ICM_20948_Stat_FIFOMoreDataAvail))
      {
        if ((data.header & DMP_header_bitmap_Quat9) <= 0) { continue; }
        // justReset = false;
        quaternionToEuler(
          ((double)data.Quat9.Data.Q1) / 1073741824.0,
          ((double)data.Quat9.Data.Q2) / 1073741824.0,
          ((double)data.Quat9.Data.Q3) / 1073741824.0,
          &ypr
        );
        Serial.print("Pitch: ");
        Serial.print(ypr.pitch);
        Serial.print(" Roll: ");
        Serial.print(ypr.roll);
        Serial.print(" Yaw: ");
        Serial.println(ypr.yaw);
        switch (instr_type)
        {
        case ATTITUDE:
          drawAttitude(spr, ypr.roll, ypr.pitch);
          break;
        case COMPASS:
          drawCompass(spr, hlpr, word_hlpr, ypr.yaw);  // TODO: Calibrate compass
          break;
        }
        if ((millis() - millisBacklight) >= 1000)
        {
          handleBacklight(100);
          millisBacklight = millis();
        }
      // }
      // else
      // {
      //   missedReadings++;
      //   if (missedReadings > 50 || ((!justReset) && (missedReadings > 20)))
      //   {
      //     resetIMU();
      //   }
      // }
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
void quaternionToEuler(double q1, double q2, double q3, euler_t *_data)
{
  float qr = sqrt(1.0 - (sq(q1) + sq(q2) + sq(q3)));

  // Need to re-orient quaternion axes
  float qi = q1;
  float qj = -q2;
  float qk = -q3;

  float qip = qi;
  float qkp = qk;
  float qjp = qj;
  float qrp = qr;

  qr = (qrp * qw) - (qip * qx) - (qjp * qy) - (qkp * qz);
  qi = (qrp * qx) + (qip * qw) + (qjp * qz) - (qkp * qy);
  qj = (qrp * qy) - (qip * qz) + (qjp * qw) + (qkp * qx);
  qk = (qrp * qz) + (qip * qy) - (qjp * qx) + (qkp * qw);

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  _data->yaw = COMPASS_OFFSET - (atan2(2.0 * ((qi * qj) + (qk * qr)), (sqi - sqj - sqk + sqr)) / DEG2RAD);
  _data->roll = asin(-2.0 * ((qi * qk) - (qj * qr)) / (sqi + sqj + sqk + sqr)) / DEG2RAD;
  _data->pitch = atan2(2.0 * ((qj * qk) + (qi * qr)), (-sqi - sqj + sqk + sqr)) / DEG2RAD;
}

/**
 * Get arctangent angle of a position (x, y) relative to (x_c, y_c).
 */
float getAngle(float x_c, float y_c, float x, float y)
{
  return atan2(y - y_c, x - x_c) / DEG2RAD;
}
