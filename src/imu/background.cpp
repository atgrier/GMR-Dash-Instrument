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
#include "imu.h"

#define ICM20948_ADDR 0x69
ICM_20948_I2C imu;

// TODO: Probably need to re-orient code for sensor

// Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; // 250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {54.5, -8.5, -16.4};

// Accel scale: divide by 16604.0 to normalize
float A_B[3]{143.43, 105.35, 180.47};

float A_Ainv[3][3]{{0.0611, 3e-05, -0.00012},
                   {3e-05, 0.06087, 7e-05},
                   {-0.00012, 7e-05, 0.06052}};

// Mag scale divide by 369.4 to normalize
float M_B[3]{109.6, 43.4, 165.04};

float M_Ainv[3][3]{{5.76911, 0.01231, -0.04636},
                   {0.01231, 5.42147, 0.00912},
                   {-0.04636, 0.00912, 5.79055}};

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 1.0
#define Ki 0.0001

unsigned long now = 0, last = 0;
float deltat = 0;

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};

euler_t ypr;
bool imuInitialized = false;
uint8_t resetCounter = 0;
bool sleepRequested = false;
bool imuAsleep = false;

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
 * Apply offsets and scale/correction factors to IMU data.
 */
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
  float temp[3];
  byte i;

  // Apply gyro offset
  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  // Apply accel offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++)
  {
    temp[i] = (Axyz[i] - A_B[i]);
  }
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  // Apply mag offsets (bias) and scale factors from Magneto
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
 * Apply correction for mounting orientation, i.e. 70 degree rotation about x axis
 */
void mountingCorrection(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
  float gy = Gxyz[1];
  float gz = Gxyz[2];
  Gxyz[1] = (gy * cos(-PITCH_OFFSET)) - (gz * sin(-PITCH_OFFSET));
  Gxyz[2] = (gy * sin(-PITCH_OFFSET)) + (gz * cos(-PITCH_OFFSET));

  float ay = Axyz[1];
  float az = Axyz[2];
  Axyz[1] = (ay * cos(-PITCH_OFFSET)) - (az * sin(-PITCH_OFFSET));
  Axyz[2] = (ay * sin(-PITCH_OFFSET)) + (az * cos(-PITCH_OFFSET));

  float my = Mxyz[1];
  float mz = Mxyz[2];
  Mxyz[1] = (my * cos(-PITCH_OFFSET)) - (mz * sin(-PITCH_OFFSET));
  Mxyz[2] = (my * sin(-PITCH_OFFSET)) + (mz * cos(-PITCH_OFFSET));
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
  static float Gxyz[3], Axyz[3], Mxyz[3]; // Centered and scaled gyro/accel/mag data

  if (imu.dataReady())
  {
    imu.getAGMT();
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // Reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
    Mxyz[1] = -Mxyz[1]; // reflect Y and Z
    Mxyz[2] = -Mxyz[2]; // must be done after offsets & scales applied to raw data
    mountingCorrection(Gxyz, Axyz, Mxyz);

    now = micros();
    deltat = (now - last) * 1.0e-6;
    last = now;

    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    // Define Tait-Bryan angles. Strictly valid only for approximately level movement
    // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll.
    ypr.pitch = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2])) / DEG2RAD;
    ypr.roll = ROLL_OFFSET - (asin(2.0 * (q[0] * q[2] - q[1] * q[3])) / DEG2RAD);
    ypr.yaw = COMPASS_OFFSET + (atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3])) / DEG2RAD);
  }
}

/**
 * Get yaw, pitch, and roll values.
 */
euler_t *getData()
{
  return &ypr;
}

/**
 * Initialize the ICM-20948 IMU.
 */
void setupIMU()
{
  unsigned long start = millis();
  while ((millis() - start) < 1000)
  {
    imu.begin(WIRE_PORT, ICM20948_ADDR);
    if (imu.status == ICM_20948_Stat_Ok)
    {
      Serial.println("ICM-20948 Found.");
      imu.startupMagnetometer();
      Serial.println("Magnetometer initialized.");
      break;
    }
    Serial.println("Failed to find ICM-20948 chip.");
    delay(50);
  }
  imuInitialized = true;
}

/**
 * Whether the IMU is initialized.
 */
bool imuReady()
{
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
  imuAsleep = true;
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
    imuAsleep = false;
  }
}

/**
 * IMU background task.
 */
void imuTask()
{
  // setupIMU();
  uint8_t resetCount = resetCounter;
  while (true)
  {
    if (!imuInitialized)
    {
      delay(50);
      continue;
    }
    imuLoop();
    if (sleepRequested)
    {
      _sleepIMU();
    }
    if (resetCount != resetCounter)
    {
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
  while (imuInitialized && !imuAsleep) {
    delay(5);
  }
}
