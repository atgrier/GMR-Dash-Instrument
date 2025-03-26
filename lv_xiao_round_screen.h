/* display and touch driver for XIAO round screen
I had to modify C:\Users\Alan\OneDrive\Documents\Arduino\libraries\TFT_eSPI\User_Setups\Setup66_Seeed_XIAO_Round.h for the QTPY
*/

#include <Arduino.h>
#include <lvgl.h>
#include <SPI.h>
#include <Wire.h>

#include "common.h"

uint8_t screen_rotation;

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);

#if LVGL_VERSION_MAJOR == 9
void xiao_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
#elif LVGL_VERSION_MAJOR == 8
void xiao_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
#else
#error "Not support LVGL version"
#endif
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if LVGL_VERSION_MAJOR == 9
  uint16_t *px_buf = (uint16_t *)px_map;
#elif LVGL_VERSION_MAJOR == 8
  uint16_t *px_buf = (uint16_t *)&color_p->full;
#endif

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(px_buf, w * h, true);
  tft.endWrite();


#if LVGL_VERSION_MAJOR == 9
  lv_display_flush_ready(disp);
#elif LVGL_VERSION_MAJOR == 8
  lv_disp_flush_ready(disp);
#endif
}

void xiao_disp_init(void) {
#if XIAO_BL > 0
  pinMode(XIAO_BL, OUTPUT);  //Turn on screen backlight
  digitalWrite(XIAO_BL, HIGH);
#endif

  tft.init();
  tft.setRotation(screen_rotation);
  tft.fillScreen(COLOR_BG);
}

void lv_xiao_disp_init(void) {
  xiao_disp_init();

#if LVGL_VERSION_MAJOR == 9
  // static uint8_t draw_buf[ SCREEN_WIDTH * LVGL_BUFF_SIZE * LV_COLOR_DEPTH / 8 ];
  static uint32_t draw_buf[SCREEN_WIDTH * LVGL_BUFF_SIZE * LV_COLOR_DEPTH / 8 / 4];
  lv_display_t *disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(disp, xiao_disp_flush);
  lv_display_set_buffers(disp, (void *)draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
#elif LVGL_VERSION_MAJOR == 8
  /*Initialize the display buffer*/
  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf[SCREEN_WIDTH * LVGL_BUFF_SIZE];
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * LVGL_BUFF_SIZE);
  /*Initialize the display driver for lvgl*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = xiao_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);
#endif
}



/* touch driver : chsc6x */

bool chsc6x_is_pressed(void) {
  if (digitalRead(TOUCH_INT) != LOW) {
    delay(5);
    if (digitalRead(TOUCH_INT) != LOW)
      return false;
  }
  return true;
}

void chsc6x_convert_xy(uint8_t *x, uint8_t *y) {
  uint8_t x_tmp = *x, y_tmp = *y, _end = 0;
  for (int i = 1; i <= screen_rotation; i++) {
    x_tmp = *x;
    y_tmp = *y;
    _end = (i % 2) ? SCREEN_WIDTH : SCREEN_HEIGHT;
    *x = y_tmp;
    *y = _end - x_tmp;
  }
}

void chsc6x_get_xy(lv_coord_t *x, lv_coord_t *y) {
  uint8_t temp[CHSC6X_READ_POINT_LEN] = { 0 };
  uint8_t read_len = Wire.requestFrom(CHSC6X_I2C_ID, CHSC6X_READ_POINT_LEN);
  if (read_len == CHSC6X_READ_POINT_LEN) {
    Wire.readBytes(temp, read_len);
    if (temp[0] == 0x01) {
      chsc6x_convert_xy(&temp[2], &temp[4]);
      *x = temp[2];
      *y = temp[4];
    }
  }
}

#if LVGL_VERSION_MAJOR == 9
void chsc6x_read(lv_indev_t *indev, lv_indev_data_t *data)
#elif LVGL_VERSION_MAJOR == 8
void chsc6x_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
#endif
{
  lv_coord_t touchX, touchY;
  if (!chsc6x_is_pressed()) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    chsc6x_get_xy(&touchX, &touchY);
    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

void lv_xiao_touch_init(void) {
  pinMode(TOUCH_INT, INPUT_PULLUP);
  Wire.begin();  // Turn on the IIC bus for touch driver
                 /*Initialize the touch driver for lvgl*/
#if LVGL_VERSION_MAJOR == 9
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, chsc6x_read);
#elif LVGL_VERSION_MAJOR == 8
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = chsc6x_read;
  lv_indev_drv_register(&indev_drv);
#endif
}
