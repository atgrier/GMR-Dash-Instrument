#define XIAO_ESP32S3
// #define QTPY_ESP32S3

#ifdef QTPY_ESP32S3
#define TOUCH_INT 16
#define VEHICLE_BACKLIGHT 18
#define VEHICLE_LIN 37
#define WAKEUP_PIN 9
#ifndef XIAO_BL
#define XIAO_BL 5
#endif
#define XIAO_DC 8
#define XIAO_CS 17
#endif

#ifdef XIAO_ESP32S3
#define TOUCH_INT D9 // Mismatched from original D7
#define VEHICLE_BACKLIGHT D0
#define VEHICLE_LIN D7
#define WAKEUP_PIN 3 // D2
#ifndef XIAO_BL
#define XIAO_BL 43 // D6
#endif
#define XIAO_DC D3
#define XIAO_CS D1
#endif
