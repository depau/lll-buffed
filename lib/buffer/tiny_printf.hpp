#pragma once

#include <cstdarg>
#include <cstddef>

namespace tiny {
int vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);

int snprintf(char *buf, size_t size, const char *fmt, ...);

unsigned int strtoul(const char *str);

float strtof(const char *str);
} // namespace tiny