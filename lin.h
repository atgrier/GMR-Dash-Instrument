/* LIN Bus methods. */

#include "I2C_BM8563.h"

#define LIN_BAUD_MAX (10000)

#define SEC_BYTE 1
#define MIN_BYTE 2
#define HOUR_BYTE 3
#define HOUR_MASK 0x1F

void getTimeFromVehicle(bool force = false, uint32_t timeout = 1000);
I2C_BM8563_TimeTypeDef timeStruct;
