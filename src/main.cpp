#include <Arduino.h>
#include <USBSerial.h>

#include "buffer_logic.h"

#ifdef ARDUINO
#include "hardware_impl.h"
#endif

template class Buffer<BoardHardware>;
static Buffer<BoardHardware> buffer;

void setup() {
  SerialUSB.begin(115200);
  Serial2.begin(115200);
  // SerialUSB.dtr(false);
  buffer.init();
}

void loop() {
  buffer.loop();
}
