/*
I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
*/
#include "imu_compass.h"
#include "imu.h"
#include "../common.h"
#include "../pins.h"
#include "../screen.h"

float compassArcs[10][3][2];
float compassLines[50][2][2];

/**
 * Draw car and course reference line.
 */
void drawCar(TFT_eSprite *hlpr)
{
  uint8_t tip_w = 20;
  uint8_t tip_h1 = 10;
  uint8_t tip_h2 = 25;
  uint8_t length = 105;
  hlpr->fillSprite(COLOR_BG);
  hlpr->fillTriangle(CARD_C, tip_h1, CARD_C - tip_w, tip_h2, CARD_C + tip_w, tip_h2, COLOR_HEADING);
  hlpr->drawWideLine(CARD_C, CARD_C + length, CARD_C, CARD_C - length, 6.0f, COLOR_HEADING);

  float thick = 1.0f;
  float thick2 = thick / 2.;
  for (uint8_t i = 0; i < 10; i++)
  {
    hlpr->drawSmoothArc(
        compassArcs[i][0][0],
        compassArcs[i][0][1],
        compassArcs[i][1][0] + thick2,
        compassArcs[i][1][0] - thick2,
        compassArcs[i][2][0],
        compassArcs[i][2][1],
        COLOR_FG, 0x00FFFFFF, true);
  }
  for (uint8_t i = 0; i < 50; i++)
  {
    hlpr->drawWideLine(
        compassLines[i][0][0],
        compassLines[i][0][1],
        compassLines[i][1][0],
        compassLines[i][1][1],
        thick * 2,
        COLOR_FG, 0x00FFFFFF);
  }
}

/**
 * Setup the compass instrument, i.e. pre-compute static trigonometric values and draw car to the
 * helper sprite.
 */
void setupCompass(TFT_eSprite *hlpr)
{
  // Center, Radius, Start, End
  float arcs[5][4][2] = {
      // {{0, -1.567}, {8.851, 0}, {0, 7.285}, {1.495, 7.157}},
      // {{1.256, 5.762}, {1.415, 0}, {1.495, 7.157}, {2.451, 6.520}},
      // {{2.108, 5.945}, {0.670, 0}, {2.451, 6.520}, {2.777, 5.983}},
      // {{0, -0.074}, {6.790, 0}, {0, 6.716}, {1.563, 6.534}},
      // {{1.270, 5.295}, {1.273, 0}, {1.563, 6.534}, {2.299, 6.045}},
      {{0, -0.547 * MM2PX}, {4.503 * MM2PX, 0}, {0, 3.956 * MM2PX}, {2.466 * MM2PX, 3.221 * MM2PX}},
      // {{1.129, 4.050}, {1.756, 0}, {2.882, 4.150}, {2.750, 3.375}},
      // {{-86.940, -0.332}, {89.766, 0}, {2.750, 3.375}, {2.802, -2.426}},
      // {{1.514, -3.054}, {1.433, 0}, {2.802, -2.426}, {2.946, -3.109}},
      // {{1.601, -4.694}, {1.282, 0}, {2.882, -4.744}, {2.615, -5.479}},
      {{0, -3.312 * MM2PX}, {5.026 * MM2PX, 0}, {0, 1.714 * MM2PX}, {2.076 * MM2PX, 1.265 * MM2PX}},
      {{0, -3.417 * MM2PX}, {4.363 * MM2PX, 0}, {0, 0.945 * MM2PX}, {1.432 * MM2PX, 0.704 * MM2PX}},
      {{0, -6.620 * MM2PX}, {6.224 * MM2PX, 0}, {0, -0.396 * MM2PX}, {1.432 * MM2PX, -0.563 * MM2PX}},
      {{0, -9.117 * MM2PX}, {7.506 * MM2PX, 0}, {0, -1.612 * MM2PX}, {2.005 * MM2PX, -1.884 * MM2PX}},
      // {{0, 5.422}, {11.890, 0}, {1.885, -6.318}, {0, -6.468}},
      // {{1.849, -6.146}, {0.718, 0}, {2.566, -6.195}, {1.977, -6.853}},
      // {{1.303, -6.108}, {1.266, 0}, {2.566, -6.195}, {1.944, -7.200}},
      // {{0, 15.053}, {22.337, 0}, {1.944, -7.200}, {0, -7.285}}
  };
  // Start, End
  float lines[25][2][2] = {
      {{2.777 * MM2PX, 5.983 * MM2PX}, {2.822 * MM2PX, 4.150 * MM2PX}},
      {{2.299 * MM2PX, 6.045 * MM2PX}, {2.466 * MM2PX, 3.221 * MM2PX}},
      {{2.466 * MM2PX, 3.221 * MM2PX}, {2.076 * MM2PX, 1.265 * MM2PX}},
      {{2.076 * MM2PX, 1.265 * MM2PX}, {2.005 * MM2PX, -1.884 * MM2PX}},
      {{1.432 * MM2PX, 0.704 * MM2PX}, {1.432 * MM2PX, -0.563 * MM2PX}},
      {{2.005 * MM2PX, -1.884 * MM2PX}, {1.885 * MM2PX, -6.318 * MM2PX}},
      {{2.946 * MM2PX, -3.109 * MM2PX}, {2.882 * MM2PX, -4.744 * MM2PX}},
      {{2.615 * MM2PX, -5.479 * MM2PX}, {2.566 * MM2PX, -6.195 * MM2PX}},
      {{1.977 * MM2PX, -6.853 * MM2PX}, {0 * MM2PX, -6.908 * MM2PX}},
      // Mirror
      {{2.763 * MM2PX, 1.944 * MM2PX}, {3.387 * MM2PX, 1.768 * MM2PX}},
      {{3.387 * MM2PX, 1.768 * MM2PX}, {3.341 * MM2PX, 2.110 * MM2PX}},
      {{3.341 * MM2PX, 2.110 * MM2PX}, {2.759 * MM2PX, 2.333 * MM2PX}},
      // Arcs that I have given up on
      {{1.495 * MM2PX, 7.157 * MM2PX}, {2.451 * MM2PX, 6.520 * MM2PX}},
      {{2.451 * MM2PX, 6.520 * MM2PX}, {2.777 * MM2PX, 5.983 * MM2PX}},
      {{1.563 * MM2PX, 6.534 * MM2PX}, {2.299 * MM2PX, 6.045 * MM2PX}},
      {{2.882 * MM2PX, 4.150 * MM2PX}, {2.750 * MM2PX, 3.375 * MM2PX}},
      {{2.750 * MM2PX, 3.375 * MM2PX}, {2.802 * MM2PX, -2.426 * MM2PX}},
      {{2.802 * MM2PX, -2.426 * MM2PX}, {2.946 * MM2PX, -3.109 * MM2PX}},
      {{2.882 * MM2PX, -4.744 * MM2PX}, {2.615 * MM2PX, -5.479 * MM2PX}},
      {{2.566 * MM2PX, -6.195 * MM2PX}, {1.977 * MM2PX, -6.853 * MM2PX}},
      {{2.566 * MM2PX, -6.195 * MM2PX}, {1.944 * MM2PX, -7.200 * MM2PX}},
      // Other arcs
      {{0, 7.285 * MM2PX}, {1.495 * MM2PX, 7.157 * MM2PX}},
      {{0, 6.716 * MM2PX}, {1.563 * MM2PX, 6.534 * MM2PX}},
      {{1.885 * MM2PX, -6.318 * MM2PX}, {0, -6.468 * MM2PX}},
      {{1.944 * MM2PX, -7.200 * MM2PX}, {0, -7.285 * MM2PX}}};

  uint8_t index = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    float start = (-getAngle(arcs[i][0][0], arcs[i][0][1], arcs[i][2][0], arcs[i][2][1])) + 270;
    float end = (-getAngle(arcs[i][0][0], arcs[i][0][1], arcs[i][3][0], arcs[i][3][1])) + 270;
    for (int8_t k = 1; k > -2; k -= 2)
    {
      if (k < 0)
      {
        start = (start * -1) + 360;
        end = (end * -1) + 360;
      }
      compassArcs[index][0][0] = CARD_C + (arcs[i][0][0] * k);
      compassArcs[index][0][1] = CARD_C - arcs[i][0][1];
      compassArcs[index][1][0] = arcs[i][1][0];
      compassArcs[index][2][0] = min(start, end);
      compassArcs[index][2][1] = max(start, end);
      index++;
    }
  }

  index = 0;
  for (uint8_t i = 0; i < 25; i++)
  {
    for (int8_t k = 1; k > -2; k -= 2)
    {
      compassLines[index][0][0] = CARD_C + (lines[i][0][0] * k);
      compassLines[index][0][1] = CARD_C - lines[i][0][1];
      compassLines[index][1][0] = CARD_C + (lines[i][1][0] * k);
      compassLines[index][1][1] = CARD_C - lines[i][1][1];
      index++;
    }
  }

  drawCar(hlpr);
}

/**
 * Draw compass scale at current heading.
 */
void drawDial(TFT_eSprite *spr, TFT_eSprite *word_hlpr, float heading)
{
  float xp = 0.0, yp = 0.0;
  float xt = 0.0, yt = 0.0;
  char words[][10] = {"N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"};

  float c180 = cos(180 * DEG2RAD);
  float s180 = sin(180 * DEG2RAD);
  float tick_specs[][2] = {{15, 0}, {6, 5}};
  for (uint8_t j = 0; j < 2; j++)
  {
    for (uint8_t i = 0; i < 18; i++)
    {
      getCoord(0, 0, &xp, &yp, CARD_R, tick_specs[j][1] + (10 * i) - heading);
      getCoord(0, 0, &xt, &yt, CARD_R - tick_specs[j][0], tick_specs[j][1] + (10 * i) - heading);
      spr->drawWideLine(CARD_C + xp, CARD_C - yp, CARD_C + xt, CARD_C - yt, 2.0f, COLOR_FG);
      spr->drawWideLine(CARD_C + (xp * c180) + (yp * s180), CARD_C - ((yp * c180) - (xp * s180)), CARD_C + (xt * c180) + (yt * s180), CARD_C - ((yt * c180) - (xt * s180)), 2.0f, COLOR_FG);
    }
  }

  word_hlpr->setPivot(HELPER_W / 2, CARD_R - (HELPER_H / 2) - 10);
  for (uint8_t i = 0; i < 12; i++)
  {
    word_hlpr->fillSprite(COLOR_BG);
    word_hlpr->drawString(words[i], HELPER_W / 2, HELPER_H / 2);
    word_hlpr->pushRotated(spr, (30 * i) + heading, COLOR_BG);
  }
}

/**
 * Draw compass instrument at current heading.
 */
void drawCompass(TFT_eSprite *spr, TFT_eSprite *hlpr, TFT_eSprite *word_hlpr, float heading)
{
  spr->fillSprite(COLOR_BG);
  drawDial(spr, word_hlpr, heading);
  hlpr->pushToSprite(spr, 0, 0, COLOR_BG);
  spr->pushSprite(-CENTER_OFFSET, -CENTER_OFFSET);
}
