/**
 * Common utilities.
 */

#define DEG2RAD 0.0174532925
#define MM2PX 7.3846153846

void getCoord(float x, float y, float *xp, float *yp, float r, float a);
bool handleBacklight(uint32_t freq);
int8_t clickType(uint8_t max_clicks);
