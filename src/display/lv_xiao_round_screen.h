#include <TFT_eSPI.h>
#include <lvgl.h>

#include "../screen.h"

#define USE_TFT_ESPI_LIBRARY

#define SCREEN_WIDTH SCREEN_SIZE
#define SCREEN_HEIGHT SCREEN_SIZE
#define LVGL_BUFF_SIZE 10 // Number of rows

#define CHSC6X_I2C_ID 0x2e
#define CHSC6X_MAX_POINTS_NUM 1
#define CHSC6X_READ_POINT_LEN 5

void readScreen(lv_indev_data_t *data);
bool chsc6x_is_pressed(void);
void xiao_disp_init(void);
void screenSleep();
TFT_eSPI *getTft();
