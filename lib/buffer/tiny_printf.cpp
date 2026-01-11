#include <cmath>
#include <cstdint>

#include "tiny_printf.hpp"

static void out_char(char *&buf, size_t &rem, const char c, int &count) {
  if (rem > 1) {
    *buf++ = c;
    --rem;
  }
  ++count;
}

static void out_str(char *&buf, size_t &rem, const char *s, int &count) {
  while (*s) {
    out_char(buf, rem, *s++, count);
  }
}

static void out_uint(char *&buf, size_t &rem, uint32_t v, unsigned base, int &count) {
  char tmp[16];
  unsigned i = 0;
  do {
    tmp[i++] = "0123456789abcdef"[v % base];
    v /= base;
  } while (v);
  while (i--)
    out_char(buf, rem, tmp[i], count);
}

static void out_int(char *&buf, size_t &rem, int32_t v, int &count) {
  if (v < 0) {
    out_char(buf, rem, '-', count);
    v = -v;
  }
  out_uint(buf, rem, static_cast<uint32_t>(v), 10, count);
}

static void out_float(char *&buf, size_t &rem, double f, int &count) {
  if (std::isnan(f)) {
    out_str(buf, rem, "nan", count);
    return;
  }
  if (std::isinf(f)) {
    out_str(buf, rem, "inf", count);
    return;
  }

  if (f < 0) {
    out_char(buf, rem, '-', count);
    f = -f;
  }

  const auto ip = static_cast<uint32_t>(f);
  double frac = f - static_cast<double>(ip);

  out_uint(buf, rem, ip, 10, count);
  out_char(buf, rem, '.', count);

  for (int i = 0; i < 2; ++i) { // 2 decimal places
    frac *= 10.0;
    const int d = static_cast<int>(frac);
    out_char(buf, rem, '0' + d, count);
    frac -= d;
  }
}

int tiny_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap) {
  char *p = buf;
  size_t rem = size;
  int count = 0;

  for (; *fmt; ++fmt) {
    if (*fmt != '%') {
      out_char(p, rem, *fmt, count);
      continue;
    }

    ++fmt;
    switch (*fmt) {
    case 'd':
      out_int(p, rem, va_arg(ap, int), count);
      break;
    case 'u':
      out_uint(p, rem, va_arg(ap, unsigned), 10, count);
      break;
    case 'x':
      out_uint(p, rem, va_arg(ap, unsigned), 16, count);
      break;
    case 's':
      out_str(p, rem, va_arg(ap, const char *), count);
      break;
    case 'c':
      out_char(p, rem, static_cast<char>(va_arg(ap, int)), count);
      break;
    case 'f':
      out_float(p, rem, va_arg(ap, double), count);
      break;
    case '%':
      out_char(p, rem, '%', count);
      break;
    default:
      out_char(p, rem, '?', count);
      break;
    }
  }

  if (rem > 0)
    *p = '\0';
  else if (size)
    buf[size - 1] = '\0';

  return count;
}

int tiny_snprintf(char *buf, const size_t size, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const int r = tiny_vsnprintf(buf, size, fmt, ap);
  va_end(ap);
  return r;
}
