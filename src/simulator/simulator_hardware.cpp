#include "simulator_hardware.hpp"

volatile uint8_t simulator_buffer_id __attribute__((used, unused));
#ifdef ENABLE_I2C_PROTOCOL
volatile uint8_t simulator_i2c_address __attribute__((used, unused));
#endif