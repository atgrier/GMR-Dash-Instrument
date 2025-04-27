/**
 * Methods for handling sleep mode.
 */
#define LIN_OFF_TIME 10000

void updateLinTime();
void checkSleep();
void attachSleepInterrupt();
void wakeupCleanup();
void goToSleep();
