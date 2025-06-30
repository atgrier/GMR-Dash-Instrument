/**
 * IMU instruments.
 */
#include <TFT_eSPI.h>
#include "../pins.h"

#define ICM20948_ADDR 0x69

#define COLOR_SKY 0x4457
#define COLOR_GROUND 0x9B06
#define COLOR_POINTER 0xF7BE
#define COLOR_CAR 0x18C3
#define COLOR_HEADING 0xF81E

#define HORIZON_THICKNESS 3.0f
#define BAR_THICKNESS 2.0f
#define DEG_10_LENGTH 104.0f
#define DEG_5_LENGTH 60.0f
#define DEG_2_5_LENGTH 30.0f

#define ROLL_TICK_SHORT 9.0f
#define ROLL_TICK_LONG 18.0f
#define ROLL_ARC_INSIDE 114.0f

#define ROLL_MULTIPLIER 1.6
#define PITCH_OFFSET -70 * DEG2RAD // Radians
#define ROLL_OFFSET -2             // Degress
#define COMPASS_OFFSET 35          // Degress
#define DIST_PER_DEG (60.0f / 15.0f)
#define DEG_PER_SCREEN (CARD_SIZE / DIST_PER_DEG)

#ifndef IMU_DEFINED
#define IMU_DEFINED
typedef struct euler_t
{
  float yaw;
  float pitch;
  float roll;
};

enum imu_instrument_t
{
  ATTITUDE,
  COMPASS
};
#endif

void quaternionToEuler(double q1, double q2, double q3, euler_t *_data);
void imuInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, TFT_eSprite *word_hlpr, imu_instrument_t instr_type);
void sleepIMU();
void setupIMU();
