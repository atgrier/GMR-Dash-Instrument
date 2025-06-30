/**
 * Screen assignments.
 */
#define SCREEN_SIZE TFT_WIDTH
#define CARD_SIZE 240.0f
#define CARD_R (CARD_SIZE / 2.0f)
#define CENTER_OFFSET ((CARD_SIZE - SCREEN_SIZE) / 2.0f) // To the left and up (negative in both X and Y)
#define CARD_C ((CARD_SIZE / 2.0f) - CENTER_OFFSET)
#define HELPER_W 32
#define HELPER_H 24

#define COLOR_BG 0x2104
#define COLOR_BG_NIGHT 0x0000
#define COLOR_FG 0xEF7D
#define COLOR_FG_NIGHT 0x8fa9
#define COLOR_GREY 0x39E7
