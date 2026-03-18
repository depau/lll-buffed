#include <Arduino.h>

#include "dfu.hpp"
#include "stm32f0xx.h"

// Huge credits to:
// https://community.st.com/t5/stm32-mcus-embedded-software/tutorial-restart-stm32f-in-dfu-mode/td-p/640149

// Relevant application note:
// AN2606: Introduction to system memory boot mode on STM32 MCUs
// https://www.st.com/content/ccc/resource/technical/document/application_note/b9/9b/16/3a/12/1e/40/0c/CD00167594.pdf/files/CD00167594.pdf/jcr:content/translations/en.CD00167594.pdf

static constexpr uint32_t BOOT_MAGIC = 0xB007DF01u;
static constexpr uint32_t RAM_BASE = 0x20000000u;

auto ram_base_ptr = reinterpret_cast<uint32_t *>(RAM_BASE);
uint32_t &ram_base = *ram_base_ptr;

[[noreturn]] void reboot_to_dfu() {
  ram_base = BOOT_MAGIC;
  __DSB();
  __ISB();
  NVIC_SystemReset();
}
