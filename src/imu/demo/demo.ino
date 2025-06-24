//
/*
  Sparkfun ICM_20948
  Hardware setup: This library supports communicating with the
  ICM_20948 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V or 3.3V
   GND ------------- GND

*/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

// Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; // 250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {54.5, -8.5, -16.4};
#define DEG2RAD 0.0174532925
#define PITCH_OFFSET 70.0 * DEG2RAD

// Accel scale: divide by 16604.0 to normalize
float A_B[3]{143.43, 105.35, 180.47};

float A_Ainv[3][3]{{0.0611, 3e-05, -0.00012},
                   {3e-05, 0.06087, 7e-05},
                   {-0.00012, 7e-05, 0.06052}};

// Mag scale divide by 369.4 to normalize
float M_B[3]{111.47, 44.59, 162.5};

float M_Ainv[3][3]{{3.84753, -0.06611, 0.02635},
                   {-0.06611, 3.69417, -0.01908},
                   {0.02635, -0.01908, 3.67146}};

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; // micros() timers for AHRS loop
float deltat = 0;                // loop time in seconds

#define PRINT_SPEED 300      // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; // Euler angle output

void imuLoop(void *)
{
  Serial.println("starting IMU task");
  while (true)
  {
    Serial.println("imu loop");
    static float Gxyz[3], Axyz[3], Mxyz[3]; // centered and scaled gyro/accel/mag data

    // Update the sensor values whenever new data is available
    if (imu.dataReady())
    {
      imu.getAGMT();
      get_scaled_IMU(Gxyz, Axyz, Mxyz);

      // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0
      Mxyz[1] = -Mxyz[1]; // reflect Y and Z
      Mxyz[2] = -Mxyz[2]; // must be done after offsets & scales applied to raw data
      mountingCorrection(Gxyz, Axyz, Mxyz);

      now = micros();
      deltat = (now - last) * 1.0e-6; // seconds since last update
      last = now;

      //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
      MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                             Mxyz[0], Mxyz[1], Mxyz[2], deltat);

      if (millis() - lastPrint > PRINT_SPEED)
      {

        // Define Tait-Bryan angles. Strictly valid only for approximately level movement

        // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        // Earth is positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // Tait-Bryan angles as well as Euler angles are
        // non-commutative; that is, the get the correct orientation the rotations
        // must be applied in the correct order.
        //
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.

        // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
        // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

        roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2])) / DEG2RAD;
        pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3])) / DEG2RAD;
        yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3])) / DEG2RAD;

        //     Serial.print("ypr ");
        Serial.print(yaw, 0);
        Serial.print(", ");
        Serial.print(pitch, 0);
        Serial.print(", ");
        Serial.print(roll, 0);
        //          Serial.print(", ");  //prints 49 in 300 ms (~160 Hz) with 8 MHz ATmega328
        Serial.println();
        lastPrint = millis(); // Update lastPrint time
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok)
  {
    Serial.println(F("ICM_90248 not detected"));
    while (1)
      ;
  }
  imu.startupMagnetometer();
  xTaskCreatePinnedToCore(
      imuLoop,   // Function to implement the task
      "imuLoop", // Name of the task
      60000,     // Stack size in words
      NULL,      // Task input parameter
      1,         // Priority of the task
      NULL,      // Task handle.
      1          // Core where the task should run
  );
}

void loop() {}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
  byte i;
  float temp[3];

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

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
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

/**
 * Apply correction for mounting orientation, i.e. 70 degree rotation about x axis
 */
void mountingCorrection(float Gxyz[3], float Axyz[3], float Mxyz[3])
{
  float gy = Gxyz[1];
  float gz = Gxyz[2];
  Gxyz[1] = (gy * cos(PITCH_OFFSET)) - (gz * sin(PITCH_OFFSET));
  Gxyz[2] = (gy * sin(PITCH_OFFSET)) + (gz * cos(PITCH_OFFSET));

  float ay = Axyz[1];
  float az = Axyz[2];
  Axyz[1] = (ay * cos(PITCH_OFFSET)) - (az * sin(PITCH_OFFSET));
  Axyz[2] = (ay * sin(PITCH_OFFSET)) + (az * cos(PITCH_OFFSET));

  float my = Mxyz[1];
  float mz = Mxyz[2];
  Mxyz[1] = (my * cos(PITCH_OFFSET)) - (mz * sin(PITCH_OFFSET));
  Mxyz[2] = (my * sin(PITCH_OFFSET)) + (mz * cos(PITCH_OFFSET));
}
