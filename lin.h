/* LIN Bus methods. */

#include "I2C_BM8563.h"

#define LIN_BAUD_MAX (10000)

#define SEC_BYTE 1
#define MIN_BYTE 2
#define HOUR_BYTE 3
#define HOUR_MASK 0x1F

void getTimeFromVehicle(bool force = false, uint32_t timeout = 1000);
void drawClock(TFT_eSprite *spr, TFT_eSprite *hlpr, bool setting_mode = false);
void syncTime(bool get_time = true);
I2C_BM8563_TimeTypeDef timeStruct;
