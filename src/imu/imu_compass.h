/**
 * Compass instrument.
 *
 * I had to modify TFT_eSPI::drawArc so that it would anti-alias properly against any background color, i.e. when passing in 0x00FFFFFF
 */
#include <TFT_eSPI.h>

void setupCompass(TFT_eSprite *hlpr);
void drawCompass(TFT_eSprite *spr, TFT_eSprite *hlpr, TFT_eSprite *word_hlpr, float heading);
