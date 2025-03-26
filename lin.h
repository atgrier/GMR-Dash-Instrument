/* LIN Bus methods. */

#define LIN_BAUD_MAX (10000)

#define SEC_BYTE 1
#define MIN_BYTE 2
#define HOUR_BYTE 3
#define HOUR_MASK 0x1F

void getTimeFromVehicle(uint32_t timeout = 1000);
