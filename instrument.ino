/*
I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
Touch: https://wiki.seeedstudio.com/seeedstudio_round_display_usage/#touch-function
*/

#include <Arduino.h>
#include <Wire.h>
#include "sleep.h"

#define USE_TFT_ESPI_LIBRARY
#include "lv_xiao_round_screen.h"

TFT_eSprite instrument = TFT_eSprite(&tft);
TFT_eSprite helper = TFT_eSprite(&tft);

uint8_t instr = 0;
uint8_t instr_prev = -1;
uint8_t vehicle_brightness = 0;

// TODO: Generally add error handling for instruments and external calls

// =========================================================================
// Setup
// =========================================================================
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while ((!Serial) && ((millis() - start) < 1000)) { delay(10); }
  Serial.println("Booting...");

  wakeupCleanup();
  attachSleepInterrupt();
  xiao_disp_init();
  instrument.createSprite(CARD_SIZE, CARD_SIZE);
  helper.createSprite(CARD_SIZE, CARD_SIZE);

  pinMode(TOUCH_INT, INPUT_PULLUP);
  pinMode(VEHICLE_BACKLIGHT, INPUT);
  handleBacklight(100);
  Wire.begin();
#ifdef QTPY_ESP32S3
  Wire1.begin();
#endif
}

// =========================================================================
// Loop
// =========================================================================
void loop() {
  if (instr == 0) {
    clockInstrument(&instrument, &helper);
  } else if (instr == 1) {
    attitudeInstrument(&instrument);
  }
  delay(200);
  instr++;
  if (instr > 1) { instr = 0; }
}

/* clickType
Returns: Number of clicks, negative if last click was long
*/
int8_t clickType(uint8_t max_clicks = 127) {
  if (!chsc6x_is_pressed()) {
    return 0;
  }
  int8_t clicks = 1;
  unsigned long start = 0;
  while (true) {
    start = millis();
    while (chsc6x_is_pressed()) {}
    if ((millis() - start) >= 1000) {
      return -clicks;
    }
    if (clicks == max_clicks) {
      return clicks;
    }
    start = millis();
    while (!chsc6x_is_pressed()) {
      if ((millis() - start) >= 400) {
        return clicks;
      }
    }
    clicks++;
  }
}

/* handleBacklight
Returns: Boolean for whether vehicle backlight is on, i.e. it is night time.

It appears to be 100Hz PWM (driven by a MOSFET) that can range from 0% to 100% duty cycle, based on dial in light switch cluster.
I will need to pass it through a voltage divider down to logic level range.
Assume 14.5V -> 13V, a resistor ratio of 16:4.7 works out nicely, say 1600 Ohm and 470 Ohm.
Dimmest pulse is 1054 ms high (Basically 10% duty cycle)
Highest pulse is 100% duty cycle
*/
bool handleBacklight(uint32_t freq) {
  // TODO: The brightness flickers a lot when the vehicle is on, maybe the PWM varies a lot?
  uint8_t brightness = (uint8_t)max(min((255 * pulseIn(VEHICLE_BACKLIGHT, HIGH, 3000000 / freq) * freq / 1000000), (long unsigned int)255), (long unsigned int)0);
  if (brightness == 0 && digitalRead(VEHICLE_BACKLIGHT)) {
    brightness = 255;
  }
  if ((brightness == vehicle_brightness) || ((brightness != 0) && (brightness != 255) && (fabsf(brightness - vehicle_brightness) <= 8))) {
    return (brightness != 0);
  }
  vehicle_brightness = brightness;
  if (vehicle_brightness == 0) {
    analogWrite(XIAO_BL, 192);  // TODO: Tune daytime brightness
    return false;
  }
  analogWrite(XIAO_BL, map(vehicle_brightness, 24, 255, 32, 128));  // TODO: Tune nighttime brightness
  return true;
}

// =========================================================================
// Get coordinates of end of a line, pivot at x,y, length r, angle a
// =========================================================================
// Coordinates are returned to caller via the xp and yp pointers
void getCoord(float x, float y, float *xp, float *yp, float r, float a) {
  float sx1 = cos(a * DEG2RAD);
  float sy1 = sin(a * DEG2RAD);
  *xp = sx1 * r + x;
  *yp = sy1 * r + y;
}
