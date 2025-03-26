/* LIN Bus methods. */

#include "src/SoftwareLin/SoftwareLin.h"
#include "lin.h"
#include "common.h"

SoftwareLin swLin(VEHICLE_LIN, -1);
bool timeInitialized = false;

void getTimeFromVehicle(uint32_t timeout) {
  swLin.begin(LIN_BAUD_MAX);
  uint32_t start = millis();
  while (true) {
    if (millis() - start > 10000) {
      swLin.end();
      return;
    }
    const int frame_data_bytes = 4;

    uint8_t buf[2 + frame_data_bytes];  // 2 bytes for PID and CHECKSUM. !!! The SYNC is consumed by swLin.setAutoBaud()

    // sw_lin.checkBreak() blocks until UART ISR gives the semaphore.
    // TODO: I NEED this to not be blocking, so that I don't get stuck if the bus is inactive or the wire comes undone
    if (swLin.checkBreak(timeout)) {

      const uint32_t commonBaud[] = { 9597, 9600, 9615 };
      uint32_t autobaud = swLin.setAutoBaud(commonBaud, sizeof(commonBaud) / sizeof(commonBaud[0]), timeout);

      const int read_timeout = 100000;  // 100ms timeout
      int start_time = micros();

      int bytes_to_read = sizeof(buf) / sizeof(buf[0]);
      int bytes_read = 0;
      while (bytes_read < bytes_to_read && micros() - start_time <= read_timeout) {
        bytes_read += swLin.read(buf + bytes_read, bytes_to_read - bytes_read);
        delay(0);  // yield for other tasks
      }
      swLin.endFrame();

      if (bytes_read < bytes_to_read) {
        // Serial.printf("Timeout: only %d bytes is read\n", bytes_read);
        continue;
      }
      // Serial.println(buf[0]);
      if (buf[0] != 0xF0) {
        continue;
      }
      uint8_t chk = buf[1] + buf[2] + buf[3] + buf[4];
      if (chk + buf[5] != 0xFF) {
        // Serial.print("Data is invalid, got Checksum: ");
        // Serial.print(buf[5]);
        // Serial.print(". Expected: ");
        // Serial.println(0xFF - chk);
        continue;
      }
      for (int i = 0; i < bytes_to_read; ++i) {
        Serial.printf("0x%02X ", buf[i]);
      }
      Serial.print("\nHour: ");
      Serial.println(buf[HOUR_BYTE] & HOUR_MASK);
      Serial.print("Minute: ");
      Serial.println(buf[MIN_BYTE]);
      Serial.print("Second: ");
      Serial.println(buf[SEC_BYTE]);
      Serial.println();
      // TODO: Need to set RTC
      // timeStruct.hours = buf[HOUR_BYTE] & HOUR_MASK;
      // timeStruct.minutes = buf[MIN_BYTE];
      // timeStruct.seconds = buf[SEC_BYTE];
      timeInitialized = true;
      swLin.end();
      return;
    }
  }
}
