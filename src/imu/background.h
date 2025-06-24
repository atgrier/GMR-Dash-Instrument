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
void setupIMU();
void resetIMU();
bool imuReady();
void imuTask(void*);
euler_t* getData();
