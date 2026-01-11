#pragma once

#include <cstdarg>
#include <cstddef>

int tiny_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap);
