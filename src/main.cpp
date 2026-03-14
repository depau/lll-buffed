#include <Arduino.h>

#include "buffer_hardware.hpp"
#include "buffer_logic.hpp"

template class Buffer<BufferHardware>;
static Buffer<BufferHardware> buffer;

void setup() {
  buffer.init();
}

void loop() {
  buffer.loop();
}
