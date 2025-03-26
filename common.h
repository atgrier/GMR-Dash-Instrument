/*
Common settings.
*/

#define SCREEN_SIZE 240
#define SCREEN_WIDTH SCREEN_SIZE
#define SCREEN_HEIGHT SCREEN_SIZE
#define LVGL_BUFF_SIZE 10  // Number of rows

#define CHSC6X_I2C_ID 0x2e
#define CHSC6X_MAX_POINTS_NUM 1
#define CHSC6X_READ_POINT_LEN 5

#define TOUCH_INT 16
#define VEHICLE_BACKLIGHT 18
#define VEHICLE_LIN 9
#ifndef XIAO_BL
#define XIAO_BL 5
#endif
#define XIAO_DC 8
#define XIAO_CS 17

// #define TOUCH_INT D7
// #define VEHICLE_BACKLIGHT D0
// #define VEHICLE_LIN D9
// #ifndef XIAO_BL
// #define XIAO_BL D6
// #endif
// #define XIAO_DC D3
// #define XIAO_CS D1

#define COLOR_BG 0x2104
#define COLOR_FG 0xEF7D
#define COLOR_FG_NIGHT 0x774D

#define DEG2RAD 0.0174532925
#define CARD_SIZE 240.0f
#define CARD_R (CARD_SIZE / 2.0f)
#define CENTER_OFFSET ((CARD_SIZE - SCREEN_SIZE) / 2.0f)  // To the left and up (negative in both X and Y)
#define CARD_C ((CARD_SIZE / 2.0f) - CENTER_OFFSET)
