/**
 * Attitude instrument.
 *
 * I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
 */
#include <TFT_eSPI.h>

void setupAttitude();
void drawAttitude(TFT_eSprite *spr, float roll, float pitch);
