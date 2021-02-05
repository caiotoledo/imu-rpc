#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#define FAIL_GTEST    \
do {                  \
  EXPECT_FALSE(true); \
} while (false)

#define LOGDEBUG(...)
#define LOGWARN(...)  FAIL_GTEST
#define LOGERROR(...) FAIL_GTEST
#define LOG(...)
