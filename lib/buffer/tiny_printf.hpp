#pragma once

#include <cstdarg>
#include <cstddef>

namespace tiny {
int vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);

int snprintf(char *buf, size_t size, const char *fmt, ...);

unsigned int strtoul(const char *str);

float strtof(const char *str);

template<typename T>
constexpr T abs(const T value) {
  return value < 0 ? -value : value;
}
} // namespace tiny