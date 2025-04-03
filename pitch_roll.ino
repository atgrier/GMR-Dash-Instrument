/*
I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
*/
#include "SparkFun_BNO080_Arduino_Library.h"

#include "fonts/EurostileLTProUnicodeDemi24.h"
#include "common.h"

BNO080 bno085;
#define BNO085_ADDR 0x4A

#define COLOR_SKY 0x4457
#define COLOR_GROUND 0x9B06
#define COLOR_POINTER 0xF7BE
#define COLOR_CAR 0x18C3

#define HORIZON_THICKNESS 3.0f
#define BAR_THICKNESS 2.0f
#define DEG_10_LENGTH 104.0f
#define DEG_5_LENGTH 60.0f
#define DEG_2_5_LENGTH 30.0f

#define ROLL_TICK_SHORT 9.0f
#define ROLL_TICK_LONG 18.0f
#define ROLL_ARC_INSIDE 114.0f

#define ROLL_MULTIPLIER 1.4
#define DIST_PER_DEG (60.0f / 15.0f)
#define DEG_PER_SCREEN (CARD_SIZE / DIST_PER_DEG)

// float roll_p, pitch_p;
bool attitudeInitialized = false;
uint8_t millisPerReading = 4;

// TODO: Determine which angles I need, and any necessary offsets
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *_data);

void setupAttitude() {
  unsigned long start = millis();
#ifdef XIAO_ESP32S3
  while (!bno085.begin(BNO085_ADDR)) {
#endif
#ifdef QTPY_ESP32S3
  while (!bno085.begin(BNO085_ADDR, &Wire1)) {
#endif
    Serial.println("Failed to find BNO08x chip");
    if (millis() - start > 1000) {
      return;
    }
    delay(10);
  }
  Serial.println("BNO08x Found!");
  bno085.enableRotationVector(millisPerReading);
  attitudeInitialized = true;
}

void attitudeInstrument(TFT_eSprite *spr) {
  if (!attitudeInitialized) {
    setupAttitude();
    if (!attitudeInitialized) {
      return;
    }
  }

  unsigned long millisNow;
  unsigned long millisPrevious = millis();
  unsigned long millisBacklight = millis();

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi24);

  // The background colour will be read during the character rendering
  spr->setTextColor(COLOR_FG);

  while (true) {
    // check if it's time to read data and update the filter
    millisNow = millis();
    if (millisNow - millisPrevious >= millisPerReading) {
      millisPrevious = millisPrevious + millisPerReading;

      if (bno085.dataAvailable()) {
        quaternionToEuler(bno085.getQuatReal(), bno085.getQuatI(), bno085.getQuatJ(), bno085.getQuatK(), &ypr);
      }

      drawAttitude(spr, ypr.roll, ypr.pitch);
      if ((millis() - millisBacklight) >= 1000) {
        handleBacklight(100);
        millisBacklight = millis();
      }
    }
    if (clickType(1) == 1) {
      break;
    }
    checkSleep();
  }
}

void drawAttitude(TFT_eSprite *spr, float roll, float pitch) {
  drawBackground(spr, pitch);
  drawForeground(spr, roll * ROLL_MULTIPLIER);
  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}

void drawBackground(TFT_eSprite *spr, float pitch) {
  spr->fillSprite(COLOR_SKY);

  if ((pitch - (DEG_PER_SCREEN / 2.0f)) <= 0.0f) {
    float zeroPitch = fminf(CARD_C + (pitch * DIST_PER_DEG), CARD_SIZE);
    spr->fillRect(CARD_C - CARD_R, zeroPitch, CARD_SIZE, CARD_SIZE - zeroPitch, COLOR_GROUND);
  }

  for (int16_t twiceAngle = floorf(floorf(pitch) - (DEG_PER_SCREEN / 2.0f)) * 2; twiceAngle <= ceilf(ceilf(pitch) + (DEG_PER_SCREEN / 2.0f)) * 2; twiceAngle++) {
    float ypos = CARD_C - (DIST_PER_DEG * ((twiceAngle / 2.0) - pitch));
    if (twiceAngle == 0) {
      spr->drawWideLine(CARD_C - CARD_R, ypos, CARD_SIZE, ypos, HORIZON_THICKNESS, COLOR_FG);
    } else if (!(twiceAngle % 20)) {
      spr->drawWideLine(CARD_C - (DEG_10_LENGTH / 2.0f), ypos, CARD_C + (DEG_10_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
      spr->setTextDatum(MR_DATUM);
      spr->drawNumber(fabsf(twiceAngle) / 2, CARD_C - (DEG_10_LENGTH / 2.0f) - 4, ypos + 2);
      spr->setTextDatum(ML_DATUM);
      spr->drawNumber(fabsf(twiceAngle) / 2, CARD_C + (DEG_10_LENGTH / 2.0f) + 4, ypos + 2);
    } else if (!(twiceAngle % 10)) {
      spr->drawWideLine(CARD_C - (DEG_5_LENGTH / 2.0f), ypos, CARD_C + (DEG_5_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
    } else if (!(twiceAngle % 5)) {
      spr->drawWideLine(CARD_C - (DEG_2_5_LENGTH / 2.0f), ypos, CARD_C + (DEG_2_5_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
    }
  }

  float xp = 0.0, yp = 0.0;  // Use float pixel position for smooth AA motion
  float xt = 0.0, yt = 0.0;  // Use float pixel position for smooth AA motion

  // Draw roll arc
  for (uint16_t side = 90; side <= 270; side += 180) {
    spr->drawSmoothArc(CARD_C, CARD_C, ROLL_ARC_INSIDE + 3, ROLL_ARC_INSIDE, side - (50 * ROLL_MULTIPLIER), side + (50 * ROLL_MULTIPLIER), COLOR_CAR, 0x00FFFFFF, true);
    for (int8_t angle = -45; angle <= 45; angle += 15) {  // 15 degree ticks
      getCoord(CARD_C, CARD_C, &xp, &yp, ROLL_ARC_INSIDE - ROLL_TICK_LONG, 90 + side + (angle * ROLL_MULTIPLIER));
      getCoord(CARD_C, CARD_C, &xt, &yt, ROLL_ARC_INSIDE, 90 + side + (angle * ROLL_MULTIPLIER));
      spr->drawWideLine(xp, yp, xt, yt, angle == 0 ? 5.0f : 3.0f, angle == 0 ? COLOR_FG : COLOR_CAR);
    }
    for (int8_t angle = -50; angle <= 50; angle += 5) {  // 5 degree ticks
      if (!(angle % 15)) { continue; }
      getCoord(CARD_C, CARD_C, &xp, &yp, ROLL_ARC_INSIDE - ROLL_TICK_SHORT, 90 + side + (angle * ROLL_MULTIPLIER));
      getCoord(CARD_C, CARD_C, &xt, &yt, ROLL_ARC_INSIDE, 90 + side + (angle * ROLL_MULTIPLIER));
      spr->drawWideLine(xp, yp, xt, yt, 2.5f, COLOR_CAR);
    }
  }
}

void drawForeground(TFT_eSprite *spr, float roll) {
  // Draw center marker
  spr->fillSmoothCircle(CARD_C, CARD_C, 4, COLOR_CAR);

  // Draw center car, all dimensions in milimeters from center
  float car[][2] = {
    // Bottom Center, moving rightwards
    { 0.0, -1.541 },
    { 1.998, -1.541 },
    { 1.998, -2.896 },
    { 2.777, -2.896 },
    { 2.777, -1.270 },
    { 2.946, -0.085 },
    { 2.811, 0.152 },
    { 2.726, 0.729 },
    { 2.151, 2.591 },
    { 1.829, 2.896 },
    { 0.0, 2.896 }
  };
  float mirror[][2] = {
    // Right Mirror, starting at bottom left, moving rightwards
    { 2.701, 0.809 },
    { 3.269, 0.809 },
    { 3.387, 0.935 },
    { 3.387, 1.282 },
    { 3.242, 1.386 },
    { 2.688, 1.386 },
    { 2.584, 1.190 }
  };
  float boot[][2] = {
    // Asymmetric Boot, starting at left, moving rightwards
    { -2.679, 0.882 },
    { -0.690, 0.882 },
    { -0.169, 0.172 },
    { 2.808, 0.172 }
  };

  float xp = 0.0, yp = 0.0;  // Use float pixel position for smooth AA motion
  float xt = 0.0, yt = 0.0;  // Use float pixel position for smooth AA motion

  for (uint8_t i = 0; i < 10; i++) {
    mmToPx(car[i][0], car[i][1], &xp, &yp, roll);
    mmToPx(car[i + 1][0], car[i + 1][1], &xt, &yt, roll);
    spr->drawWideLine(CARD_C + xp, CARD_C - yp, CARD_C + xt, CARD_C - yt, 2.0f, COLOR_CAR);
    mmToPx(car[i][0], car[i][1], &xp, &yp, -roll);
    mmToPx(car[i + 1][0], car[i + 1][1], &xt, &yt, -roll);
    spr->drawWideLine(CARD_C - xp, CARD_C - yp, CARD_C - xt, CARD_C - yt, 2.0f, COLOR_CAR);
  }
  for (uint8_t i = 0; i < 6; i++) {
    mmToPx(mirror[i][0], mirror[i][1], &xp, &yp, roll);
    mmToPx(mirror[i + 1][0], mirror[i + 1][1], &xt, &yt, roll);
    spr->drawWideLine(CARD_C + xp, CARD_C - yp, CARD_C + xt, CARD_C - yt, 2.0f, COLOR_CAR);
    mmToPx(mirror[i][0], mirror[i][1], &xp, &yp, -roll);
    mmToPx(mirror[i + 1][0], mirror[i + 1][1], &xt, &yt, -roll);
    spr->drawWideLine(CARD_C - xp, CARD_C - yp, CARD_C - xt, CARD_C - yt, 2.0f, COLOR_CAR);
  }
  for (uint8_t i = 0; i < 3; i++) {
    mmToPx(boot[i][0], boot[i][1], &xp, &yp, roll);
    mmToPx(boot[i + 1][0], boot[i + 1][1], &xt, &yt, roll);
    spr->drawWideLine(CARD_C + xp, CARD_C - yp, CARD_C + xt, CARD_C - yt, 2.0f, COLOR_CAR);
  }

  getCoord(0, 0, &xp, &yp, 44.0, -roll);
  getCoord(0, 0, &xt, &yt, 100.0, -roll);
  spr->drawWideLine(CARD_C + xp, CARD_C + yp, CARD_C + xt, CARD_C + yt, 4.0f, COLOR_CAR);
  spr->drawWideLine(CARD_C - xp, CARD_C - yp, CARD_C - xt, CARD_C - yt, 4.0f, COLOR_CAR);
}

void mmToPx(float x, float y, float *xp, float *yp, float roll) {
  float _sin = sin((roll)*DEG2RAD);
  float _cos = cos((roll)*DEG2RAD);
  float _x = (240.0 / 32.5) * x;
  float _y = (240.0 / 32.5) * y;
  *xp = (_x * _cos) - (_y * _sin);
  *yp = (_y * _cos) + (_x * _sin);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *_data) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  _data->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) / DEG2RAD;
  _data->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) / DEG2RAD;
  _data->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) / DEG2RAD;
}
