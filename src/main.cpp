#include <Arduino.h>
#include <USBSerial.h>

#include "buffer_logic.h"

void setup() {
  SerialUSB.begin(115200);
  Serial2.begin(115200);
  // SerialUSB.dtr(false);
  buffer_init();
}

void loop() {
  buffer_loop();
  // Serial.println("loop() is running");
  // delay(1000);
}
