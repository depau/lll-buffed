#include <Arduino.h>

#include "buffer_logic.hpp"
#include "simulator_hardware.hpp"

template class Buffer<SimulatorHardware>;
static Buffer<SimulatorHardware> buffer;

void setup() {
  buffer.init();
}

void loop() {
  buffer.loop();
}
