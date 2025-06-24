/**
 * IMU background task.
 */

#ifndef IMU_BACKGROUND_H
#define IMU_BACKGROUND_H
typedef struct euler_t
{
  float yaw;
  float pitch;
  float roll;
};
#endif

void sleepIMU();
void resetIMU();
bool imuReady();
void imuTask();
euler_t* getData();
