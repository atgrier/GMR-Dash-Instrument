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

bool imuInitialized = false;
uint8_t millisPerReading = 50;
uint8_t missedReadings = 0;
bool justReset = true;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *_data);

void setupIMU() {
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
  imuInitialized = true;
}

void resetIMU() {
  bno085.modeSleep();
  delay(10);
  imuInitialized = false;
  missedReadings = 0;
  justReset = true;
  setupIMU();
}

void imuInstrument(TFT_eSprite *spr, TFT_eSprite *hlpr, imu_instrument_t instr_type) {
  if (!imuInitialized) {
    setupIMU();
    if (!imuInitialized) {
      return;
    }
  }

  unsigned long millisNow;
  unsigned long millisPrevious = millis();
  unsigned long millisBacklight = millis();

  // Only 1 font used in the sprite, so can remain loaded
  spr->loadFont(EurostileLTProDemi24);
  hlpr->loadFont(EurostileLTProDemi24);

  // The background colour will be read during the character rendering
  spr->setTextColor(COLOR_FG);
  hlpr->setTextColor(COLOR_FG);
  hlpr->setTextDatum(MC_DATUM);

  while (true) {
    millisNow = millis();
    if (millisNow - millisPrevious >= millisPerReading) {
      millisPrevious += millisPerReading;

      if (bno085.dataAvailable()) {
        justReset = false;
        quaternionToEuler(bno085.getQuatReal(), bno085.getQuatI(), bno085.getQuatJ(), bno085.getQuatK(), &ypr);
        switch (instr_type) {
          case ATTITUDE:
            drawAttitude(spr, ypr.pitch, ypr.roll);
            break;
          case COMPASS:
            drawCompass(spr, hlpr, -ypr.yaw);
            break;
        }
        if ((millis() - millisBacklight) >= 1000) {
          handleBacklight(100);
          millisBacklight = millis();
        }
      } else {
        missedReadings++;
        if (missedReadings > 50 || ((!justReset) && (missedReadings > 20))) {
          resetIMU();
        }
      }
    }
    int8_t click = clickType(3);
    if (click == 1) {
      break;
    } else if (click == 2) {
      if (imuInitialized) {
        resetIMU();
      }
    } else if (click == 3) {
      goToSleep();
    }
    checkSleep();
  }
}

void drawAttitude(TFT_eSprite *spr, float roll, float pitch) {
  drawBackground(spr, pitch + 90 - 25);  // TODO: Calibrate zero pitch offset
  drawForeground(spr, roll * ROLL_MULTIPLIER);  // TODO: Calibrate zero roll offset
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

  float symbols[3][11][2] = {
    {
      // Car (mm), starting at bottom center, moving rightwards
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
    }, {
      // Right Mirror, starting at bottom left, moving rightwards
      { 2.701, 0.809 },
      { 3.269, 0.809 },
      { 3.387, 0.935 },
      { 3.387, 1.282 },
      { 3.242, 1.386 },
      { 2.688, 1.386 },
      { 2.584, 1.190 }
    }, {
      // Asymmetric Boot, starting at left, moving rightwards
      { -2.679, 0.882 },
      { -0.690, 0.882 },
      { -0.169, 0.172 },
      { 2.808, 0.172 }
    },
  };
  uint8_t arr_bounds[] = {11, 7, 4};

  float xp = 0.0, yp = 0.0;
  float xt = 0.0, yt = 0.0;

  for (uint8_t j = 0; j < 3; j++){
    for (uint8_t i = 0; i < arr_bounds[j] - 1; i++) {
      for (int8_t k = 1; k > (j == 2 ? 0 : -2); k -= 2){
        mmToPx(symbols[j][i][0], symbols[j][i][1], &xp, &yp, roll * k);
        mmToPx(symbols[j][i + 1][0], symbols[j][i + 1][1], &xt, &yt, roll * k);
        spr->drawWideLine(CARD_C + (xp * k), CARD_C - yp, CARD_C + (xt * k), CARD_C - yt, 2.0f, COLOR_CAR);
      }
    }
  }

  getCoord(0, 0, &xp, &yp, 44.0, -roll);
  getCoord(0, 0, &xt, &yt, 100.0, -roll);
  spr->drawWideLine(CARD_C + xp, CARD_C + yp, CARD_C + xt, CARD_C + yt, 4.0f, COLOR_CAR);
  spr->drawWideLine(CARD_C - xp, CARD_C - yp, CARD_C - xt, CARD_C - yt, 4.0f, COLOR_CAR);
}

void mmToPx(float x, float y, float *xp, float *yp, float roll) {
  float _sin = sin((roll)*DEG2RAD);
  float _cos = cos((roll)*DEG2RAD);
  float _x = MM2PX * x;
  float _y = MM2PX * y;
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

void drawCompass(TFT_eSprite *spr, TFT_eSprite *hlpr, float heading) {
  spr->fillSprite(COLOR_BG);
  float xp = 0.0, yp = 0.0;
  float xt = 0.0, yt = 0.0;
  char words[][10] = {"N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"};

  float c180 = cos(180 * DEG2RAD);
  float s180 = sin(180 * DEG2RAD);
  float tick_specs[][2] = {{15, 0}, {6, 5}};
  for (uint8_t j = 0; j < 2; j++) {
    for (uint8_t i = 0; i < 18; i++) {
      getCoord(0, 0, &xp, &yp, CARD_R, tick_specs[j][1] + (10 * i) - heading);
      getCoord(0, 0, &xt, &yt, CARD_R - tick_specs[j][0], tick_specs[j][1] + (10 * i) - heading);
      spr->drawWideLine(CARD_C + xp, CARD_C - yp, CARD_C + xt, CARD_C - yt, 2.0f, COLOR_FG);
      spr->drawWideLine(CARD_C + (xp * c180) + (yp * s180), CARD_C - ((yp * c180) - (xp * s180)), CARD_C + (xt * c180) + (yt * s180), CARD_C - ((yt * c180) - (xt * s180)), 2.0f, COLOR_FG);
    }
  }

  hlpr->setPivot(32, CARD_R - 12 - 10);
  for (uint8_t i = 0; i < 12; i++) {
    hlpr->fillSprite(COLOR_BG);
    hlpr->drawString(words[i], 32, 12);
    hlpr->pushRotated(spr, (30 * i) + heading, COLOR_BG);
  }

  spr->fillTriangle(CARD_C, 10, CARD_C - 20, 25, CARD_C + 20, 25, 0xF81E);
  spr->drawWideLine(CARD_C, CARD_C + 105, CARD_C, CARD_C - 105, 6.0f, 0XF81E);

  float arcs[][4][2] = {  // Center, Radius, Start, End
    // {{0, -1.567}, {8.851, 0}, {0, 7.285}, {1.495, 7.157}},
    // {{1.256, 5.762}, {1.415, 0}, {1.495, 7.157}, {2.451, 6.520}},
    // {{2.108, 5.945}, {0.670, 0}, {2.451, 6.520}, {2.777, 5.983}},
    // {{0, -0.074}, {6.790, 0}, {0, 6.716}, {1.563, 6.534}},
    // {{1.270, 5.295}, {1.273, 0}, {1.563, 6.534}, {2.299, 6.045}},
    {{0, -0.547}, {4.503, 0}, {0, 3.956}, {2.466, 3.221}},
    // {{1.129, 4.050}, {1.756, 0}, {2.882, 4.150}, {2.750, 3.375}},
    // {{-86.940, -0.332}, {89.766, 0}, {2.750, 3.375}, {2.802, -2.426}},
    // {{1.514, -3.054}, {1.433, 0}, {2.802, -2.426}, {2.946, -3.109}},
    // {{1.601, -4.694}, {1.282, 0}, {2.882, -4.744}, {2.615, -5.479}},
    {{0, -3.312}, {5.026, 0}, {0, 1.714}, {2.076, 1.265}},
    {{0, -3.417}, {4.363, 0}, {0, 0.945}, {1.432, 0.704}},
    {{0, -6.620}, {6.224, 0}, {0, -0.396}, {1.432, -0.563}},
    {{0, -9.117}, {7.506, 0}, {0, -1.612}, {2.005, -1.884}},
    // {{0, 5.422}, {11.890, 0}, {1.885, -6.318}, {0, -6.468}},
    // {{1.849, -6.146}, {0.718, 0}, {2.566, -6.195}, {1.977, -6.853}},
    // {{1.303, -6.108}, {1.266, 0}, {2.566, -6.195}, {1.944, -7.200}},
    // {{0, 15.053}, {22.337, 0}, {1.944, -7.200}, {0, -7.285}}
  };
  float lines[][2][2] = {  // Start, End
    {{2.777, 5.983}, {2.822, 4.150}},
    {{2.299, 6.045}, {2.466, 3.221}},
    {{2.466, 3.221}, {2.076, 1.265}},
    {{2.076, 1.265}, {2.005, -1.884}},
    {{1.432, 0.704}, {1.432, -0.563}},
    {{2.005, -1.884}, {1.885, -6.318}},
    {{2.946, -3.109}, {2.882, -4.744}},
    {{2.615, -5.479}, {2.566, -6.195}},
    {{1.977, -6.853}, {0, -6.908}},
    // Mirror
    {{2.763, 1.944}, {3.387, 1.768}},
    {{3.387, 1.768}, {3.341, 2.110}},
    {{3.341, 2.110}, {2.759, 2.333}},
    // Arcs that I have given up on
    {{1.495, 7.157}, {2.451, 6.520}},
    {{2.451, 6.520}, {2.777, 5.983}},
    {{1.563, 6.534}, {2.299, 6.045}},
    {{2.882, 4.150}, {2.750, 3.375}},
    {{2.750, 3.375}, {2.802, -2.426}},
    {{2.802, -2.426}, {2.946, -3.109}},
    {{2.882, -4.744}, {2.615, -5.479}},
    {{2.566, -6.195}, {1.977, -6.853}},
    {{2.566, -6.195}, {1.944, -7.200}},
    // Other arcs
    {{0, 7.285}, {1.495, 7.157}},
    {{0, 6.716}, {1.563, 6.534}},
    {{1.885, -6.318}, {0, -6.468}},
    {{1.944, -7.200}, {0, -7.285}}
  };

  float rad, _rad = 0;
  float thick = 1.0f;
  for (uint8_t i = 0; i < 5; i++) {
    xp = arcs[i][0][0] * MM2PX;
    yp = arcs[i][0][1] * MM2PX;
    rad = arcs[i][1][0] * MM2PX;
    float start = (-getAngle(arcs[i][0][0], arcs[i][0][1], arcs[i][2][0], arcs[i][2][1])) + 270;
    float end = (-getAngle(arcs[i][0][0], arcs[i][0][1], arcs[i][3][0], arcs[i][3][1])) + 270;
    Serial.println(start);
    Serial.println(end);
    for (int8_t k = 1; k > -2; k -= 2){
      if (k < 0) {
        start = (start * -1) + 360;
        end = (end * -1) + 360;
      }
      spr->drawSmoothArc(CARD_C + (xp * k), CARD_C - yp, rad + (thick / 2), rad - (thick / 2), min(start, end), max(start, end), COLOR_FG, 0x00FFFFFF, true);
    }
  }
  for (uint8_t i = 0; i < 25; i++) {
    xp = lines[i][0][0] * MM2PX;
    yp = lines[i][0][1] * MM2PX;
    xt = lines[i][1][0] * MM2PX;
    yt = lines[i][1][1] * MM2PX;
    for (int8_t k = 1; k > -2; k -= 2){
      spr->drawWideLine(CARD_C + (xp * k), CARD_C - yp, CARD_C + (xt * k), CARD_C - yt, thick * 2, COLOR_FG, 0x00FFFFFF);
    }
  }

  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}
