#pragma once

#include <cstdarg>
#include <cstddef>

int tiny_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);

int tiny_snprintf(char *buf, size_t size, const char *fmt, ...);

unsigned int tiny_strtoul(const char *str);

float tiny_strtof(const char *str);
