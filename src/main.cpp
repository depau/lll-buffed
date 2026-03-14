#include <Arduino.h>

#include "buffer_hardware.h"
#include "buffer_logic.h"

template class Buffer<BufferHardware>;
static Buffer<BufferHardware> buffer;

void setup() {
  buffer.init();
}

void loop() {
  buffer.loop();
}
