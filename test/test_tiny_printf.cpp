#include <cmath>
#include <cstring>
#include <gtest/gtest.h>

#include "tiny_printf.hpp"

TEST(TinyPrintf, StrToUl) {
  EXPECT_EQ(tiny::strtoul("123"), 123u);
  EXPECT_EQ(tiny::strtoul("0"), 0u);
  EXPECT_EQ(tiny::strtoul("  456"), 456u);
  EXPECT_EQ(tiny::strtoul("\t\n789"), 789u);
  EXPECT_EQ(tiny::strtoul("12.34"), 12u); // Should stop at '.'
  EXPECT_EQ(tiny::strtoul("-123"), static_cast<unsigned int>(-123));
  EXPECT_EQ(tiny::strtoul("+123"), 123u);
}

TEST(TinyPrintf, StrToF) {
  EXPECT_FLOAT_EQ(tiny::strtof("123.45"), 123.45f);
  EXPECT_FLOAT_EQ(tiny::strtof("0.0"), 0.0f);
  EXPECT_FLOAT_EQ(tiny::strtof("-123.45"), -123.45f);
  EXPECT_FLOAT_EQ(tiny::strtof("  0.123"), 0.123f);
  EXPECT_FLOAT_EQ(tiny::strtof("123"), 123.0f);
  EXPECT_FLOAT_EQ(tiny::strtof(".45"), 0.45f);
  EXPECT_FLOAT_EQ(tiny::strtof("-.45"), -0.45f);
}

TEST(TinyPrintf, Snprintf) {
  char buf[64];
  int r;

  r = tiny::snprintf(buf, sizeof(buf), "test %d %u %X %x %s %c %%", -123, 456u, 0xDEAD, 0xBEEF, "hello", 'X');
  EXPECT_STREQ(buf, "test -123 456 DEAD beef hello X %");
  EXPECT_EQ(r, static_cast<int>(strlen("test -123 456 DEAD beef hello X %")));

  r = tiny::snprintf(buf, sizeof(buf), "float: %f", 123.456);
  EXPECT_STREQ(buf, "float: 123.45"); // Implementation uses 2 decimal places
  EXPECT_EQ(r, static_cast<int>(strlen("float: 123.45")));

  r = tiny::snprintf(buf, sizeof(buf), "nan: %f inf: %f", NAN, INFINITY);
  EXPECT_STREQ(buf, "nan: nan inf: inf");

  r = tiny::snprintf(buf, sizeof(buf), "neg: %f", -3.14159);
  EXPECT_STREQ(buf, "neg: -3.14");
}

TEST(TinyPrintf, SnprintfOverflow) {
  char buf[10];
  int r;

  // Buffer is exactly enough for "123456789" + null
  r = tiny::snprintf(buf, sizeof(buf), "123456789");
  EXPECT_STREQ(buf, "123456789");
  EXPECT_EQ(r, 9);

  // Buffer is too small
  r = tiny::snprintf(buf, sizeof(buf), "1234567890abcdef");
  EXPECT_STREQ(buf, "123456789"); // Should be truncated at 9 chars + null
  EXPECT_EQ(r, 16); // vsnprintf should return total expected length
}
