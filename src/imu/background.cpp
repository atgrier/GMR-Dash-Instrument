/**
 * IMU background task.
 *
 * This borrows heavily from https://github.com/jremington/ICM_20948-AHRS
 * Refer to calibration scripts included in repository, also taken from SJR
 *
 * Standard sensor orientation X North (yaw=0), Y West, Z up
 * magnetometer Y and Z axes are reflected to reconcile with accelerometer.
 * New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
 */
#include <ICM_20948.h>
#include "background.h"
#include "../common.h"
#include "../pins.h"

#define ICM20948_ADDR 0x69
ICM_20948_I2C imu;

// TODO: Probably need to re-orient code for sensor
// TODO: Determine calibration values

// Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; // 250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {74.3, 153.8, -5.5};

// Accel scale: divide by 16604.0 to normalize
float A_B[3]{79.60, -18.56, 383.31};

float A_Ainv[3][3]{{1.00847, 0.00470, -0.00428},
                   {0.00470, 1.00846, -0.00328},
                   {-0.00428, -0.00328, 0.99559}};

// Mag scale divide by 369.4 to normalize
float M_B[3]{-156.70, -52.79, -141.07};

float M_Ainv[3][3]{{1.12823, -0.01142, 0.00980},
                   {-0.01142, 1.09539, 0.00927},
                   {0.00980, 0.00927, 1.10625}};

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0;
float deltat = 0;

#define UPDATE_SPEED 300
unsigned long lastUpdate = 0;

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};

euler_t ypr;
bool imuInitialized = false;
uint8_t resetCounter = 0;
bool sleepRequested = false;

/**
 * Vector dot product.
 */
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Vector length normalization.
 */
void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

/**
 * Subtract offsets and apply scale/correction matrices to IMU data.
 */
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
  float temp[3];
  byte i;

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  // apply accel offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++)
  {
    temp[i] = (Axyz[i] - A_B[i]);
  }
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  // apply mag offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++)
  {
    temp[i] = (Mxyz[i] - M_B[i]);
  }
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

/**
 * Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
 * Modified from Madgwick version to remove Z component of magnetometer:
 * The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
 * input vectors ax, ay, az and mx, my, mz MUST be normalized!
 * gx, gy, gz must be in units of radians/second
 */
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;             // observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; // calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f)
    return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.
  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex; // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }

  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

  // update quaternion with integrated contribution
  gx = gx * (0.5 * deltat); // pre-multiply common factors
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void imuLoop()
{
  static float Gxyz[3], Axyz[3], Mxyz[3]; // centered and scaled gyro/accel/mag data

  if (imu.dataReady())
  {
    imu.getAGMT();
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
    Mxyz[1] = -Mxyz[1]; // reflect Y and Z
    Mxyz[2] = -Mxyz[2]; // must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6;
    last = now;

    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastUpdate > UPDATE_SPEED)
    {
      // Define Tait-Bryan angles. Strictly valid only for approximately level movement
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      ypr.roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2])) / DEG2RAD;
      ypr.pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3])) / DEG2RAD;
      ypr.yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3])) / DEG2RAD;
    }
  }
}

/**
 * Get yaw, pitch, and roll values.
 */
void getData(euler_t *data) {
  data = &ypr;
}

/**
 * Initialize the ICM-20948 IMU.
 */
void setupIMU()
{
  unsigned long start = millis();
  while (!imuInitialized)
  {
    imu.begin(WIRE_PORT, ICM20948_ADDR);
    if (imu.status == ICM_20948_Stat_Ok)
    {
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
  imuInitialized = true;
}

/**
 * Whether the IMU is initialized.
 */
bool imuReady() {
  return imuInitialized;
}

/**
 * Put ICM-20948 IMU to sleep.
 */
void _sleepIMU()
{
  if (imuInitialized)
  {
    imu.sleep();
  }
  delay(10);
}

/**
 * Re-initialze the ICM-20948 IMU, e.g. for when it stops providing data.
 */
void _resetIMU()
{
  if (imuInitialized)
  {
    _sleepIMU();
    imuInitialized = false;
    setupIMU();
  }
}

/**
 * IMU background task.
 */
void imuTask(void*)
{
  setupIMU();
  uint8_t resetCount = resetCounter;
  while (true)
  {
    imuLoop();
    if (sleepRequested) {
      _sleepIMU();
    }
    if (resetCount != resetCounter) {
      _resetIMU();
      resetCount = resetCounter;
    }
  }
}

/**
 * Re-initialze the ICM-20948 IMU, e.g. for when it stops providing data.
 */
void resetIMU()
{
  resetCounter++;
}

/**
 * Put ICM-20948 IMU to sleep.
 */
void sleepIMU()
{
  sleepRequested = true;
}
