#include <Arduino.h>
#include <USBSerial.h>

#include "buffer_hardware.h"
#include "buffer_logic.h"

template class Buffer<BufferHardware>;
static Buffer<BufferHardware> buffer;

void setup() {
  SerialUSB.begin(115200);
  Serial2.begin(115200);
  // SerialUSB.dtr(false);
  buffer.init();
}

void loop() {
  buffer.loop();
}
