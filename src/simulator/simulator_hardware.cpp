#include "simulator_hardware.hpp"

#ifdef ENABLE_I2C_PROTOCOL
volatile uint8_t simulator_i2c_address __attribute__((used, unused)); // BSS → starts at 0
#endif
