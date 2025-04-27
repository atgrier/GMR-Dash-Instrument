/**
 * Attitude instrument.
 *
 * I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
 */
#include "imu.h"
#include "imu_attitude.h"
#include "../common.h"
#include "../pins.h"
#include "../screen.h"

float attitudeTicks[2][28][3][2];

/**
 * Setup the attitude instrument, i.e. pre-compute static trigonometric values.
 */
void setupAttitude()
{
  float xp = 0.0, yp = 0.0;
  float xt = 0.0, yt = 0.0;
  // Draw roll arc
  uint8_t index1, index2 = 0;
  for (uint16_t side = 90; side <= 270; side += 180)
  {
    for (int8_t angle = -45; angle <= 45; angle += 15)
    { // 15 degree ticks
      getCoord(CARD_C, CARD_C, &xp, &yp, ROLL_ARC_INSIDE - ROLL_TICK_LONG, 90 + side + (angle * ROLL_MULTIPLIER));
      getCoord(CARD_C, CARD_C, &xt, &yt, ROLL_ARC_INSIDE, 90 + side + (angle * ROLL_MULTIPLIER));
      attitudeTicks[0][index1][0][0] = xp;
      attitudeTicks[0][index1][0][1] = yp;
      attitudeTicks[0][index1][1][0] = xt;
      attitudeTicks[0][index1][1][1] = yt;
      attitudeTicks[0][index1][2][0] = angle;
      index1++;
    }
    for (int8_t angle = -50; angle <= 50; angle += 5)
    { // 5 degree ticks
      if (!(angle % 15))
      {
        continue;
      }
      getCoord(CARD_C, CARD_C, &xp, &yp, ROLL_ARC_INSIDE - ROLL_TICK_SHORT, 90 + side + (angle * ROLL_MULTIPLIER));
      getCoord(CARD_C, CARD_C, &xt, &yt, ROLL_ARC_INSIDE, 90 + side + (angle * ROLL_MULTIPLIER));
      attitudeTicks[1][index2][0][0] = xp;
      attitudeTicks[1][index2][0][1] = yp;
      attitudeTicks[1][index2][1][0] = xt;
      attitudeTicks[1][index2][1][1] = yt;
      index2++;
    }
  }
}

/**
 * Draw attitude indicator background card at current pitch angle, i.e. sky, ground, horizon bar,
 * pitch indication, and bank scale (but not the bank indicator).
 */
void drawBackground(TFT_eSprite *spr, float pitch)
{
  spr->fillSprite(COLOR_SKY);

  if ((pitch - (DEG_PER_SCREEN / 2.0f)) <= 0.0f)
  {
    float zeroPitch = fminf(CARD_C + (pitch * DIST_PER_DEG), CARD_SIZE);
    spr->fillRect(CARD_C - CARD_R, zeroPitch, CARD_SIZE, CARD_SIZE - zeroPitch, COLOR_GROUND);
  }

  for (int16_t twiceAngle = floorf(floorf(pitch) - (DEG_PER_SCREEN / 2.0f)) * 2; twiceAngle <= ceilf(ceilf(pitch) + (DEG_PER_SCREEN / 2.0f)) * 2; twiceAngle++)
  {
    float ypos = CARD_C - (DIST_PER_DEG * ((twiceAngle / 2.0) - pitch));
    if (twiceAngle == 0)
    {
      spr->drawWideLine(CARD_C - CARD_R, ypos, CARD_SIZE, ypos, HORIZON_THICKNESS, COLOR_FG);
    }
    else if (!(twiceAngle % 20))
    {
      spr->drawWideLine(CARD_C - (DEG_10_LENGTH / 2.0f), ypos, CARD_C + (DEG_10_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
      spr->setTextDatum(MR_DATUM);
      spr->drawNumber(fabsf(twiceAngle) / 2, CARD_C - (DEG_10_LENGTH / 2.0f) - 4, ypos + 2);
      spr->setTextDatum(ML_DATUM);
      spr->drawNumber(fabsf(twiceAngle) / 2, CARD_C + (DEG_10_LENGTH / 2.0f) + 4, ypos + 2);
    }
    else if (!(twiceAngle % 10))
    {
      spr->drawWideLine(CARD_C - (DEG_5_LENGTH / 2.0f), ypos, CARD_C + (DEG_5_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
    }
    else if (!(twiceAngle % 5))
    {
      spr->drawWideLine(CARD_C - (DEG_2_5_LENGTH / 2.0f), ypos, CARD_C + (DEG_2_5_LENGTH / 2.0f), ypos, BAR_THICKNESS, COLOR_FG);
    }
  }

  // Draw roll arc
  for (uint8_t i = 0; i < 14; i++)
  {
    spr->drawWideLine(
        attitudeTicks[0][i][0][0], attitudeTicks[0][i][0][1],
        attitudeTicks[0][i][1][0], attitudeTicks[0][i][1][1],
        attitudeTicks[0][i][2][0] == 0 ? 5.0f : 3.0f,
        attitudeTicks[0][i][2][0] == 0 ? COLOR_FG : COLOR_CAR);
  }
  for (uint8_t i = 0; i < 28; i++)
  {
    spr->drawWideLine(
        attitudeTicks[1][i][0][0], attitudeTicks[1][i][0][1],
        attitudeTicks[1][i][1][0], attitudeTicks[1][i][1][1],
        2.5f, COLOR_CAR);
  }
}

/**
 * Draw reference car for bank indication at current bank angle.
 */
void drawForeground(TFT_eSprite *spr, float roll)
{
  // Draw center marker
  spr->fillSmoothCircle(CARD_C, CARD_C, 4, COLOR_CAR);

  float symbols[3][11][2] = {
      {// Car (mm), starting at bottom center, moving rightwards
       {0.0, -1.541},
       {1.998, -1.541},
       {1.998, -2.896},
       {2.777, -2.896},
       {2.777, -1.270},
       {2.946, -0.085},
       {2.811, 0.152},
       {2.726, 0.729},
       {2.151, 2.591},
       {1.829, 2.896},
       {0.0, 2.896}},
      {// Right Mirror, starting at bottom left, moving rightwards
       {2.701, 0.809},
       {3.269, 0.809},
       {3.387, 0.935},
       {3.387, 1.282},
       {3.242, 1.386},
       {2.688, 1.386},
       {2.584, 1.190}},
      {// Asymmetric Boot, starting at left, moving rightwards
       {-2.679, 0.882},
       {-0.690, 0.882},
       {-0.169, 0.172},
       {2.808, 0.172}},
  };
  uint8_t arr_bounds[] = {11, 7, 4};

  float xp = 0.0, yp = 0.0;
  float xt = 0.0, yt = 0.0;

  for (uint8_t j = 0; j < 3; j++)
  {
    for (uint8_t i = 0; i < arr_bounds[j] - 1; i++)
    {
      for (int8_t k = 1; k > (j == 2 ? 0 : -2); k -= 2)
      {
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

/**
 * Draw the attitude instrument at pitch and bank angles.
 */
void drawAttitude(TFT_eSprite *spr, float roll, float pitch)
{
  drawBackground(spr, pitch + 90 - 25);        // TODO: Calibrate zero pitch offset
  drawForeground(spr, roll * ROLL_MULTIPLIER); // TODO: Calibrate zero roll offset
  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}
