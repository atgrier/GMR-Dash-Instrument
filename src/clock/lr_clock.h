/**
 * Land Rover LR4 Clock.
 */
#include <TFT_eSPI.h>

#define COLOR_PIVOT 0x18C3
#define COLOR_MH 0xF7BE
#define COLOR_HH 0xDEFB
#define COLOR_MH_NIGHT 0x8F88
#define COLOR_HH_NIGHT 0x8789

#define M_HAND_LENGTH 110.0f
#define H_HAND_LENGTH 86.0f
#define TICK_HEIGHT 26.0f
#define TICK_WIDTH 5.0f

// Calculate 1 second increment angles. Hours and minute hand angles
// change every second so we see smooth sub-pixel movement
#define SECOND_ANGLE (360.0 / 60.0)
#define MINUTE_ANGLE (SECOND_ANGLE / 60.0)
#define HOUR_ANGLE (MINUTE_ANGLE / 12.0)

#define SETTING_WIDTH 60
#define SETTING_H_HEIGHT 50
#define SETTING_M_HEIGHT 70
#define SETTING_BASE_OFFSET 10
#define SETTING_LATERAL_OFFSET 65

#define SEC_BYTE 1
#define MIN_BYTE 2
#define HOUR_BYTE 3
#define HOUR_MASK 0x1F
#define LIN_BAUD_MAX (10000)

void clockInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, bool no_lin);
