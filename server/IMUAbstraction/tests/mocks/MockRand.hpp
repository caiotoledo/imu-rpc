#pragma once

#include <gmock/gmock.h>

class MockRand {
 public:
  MOCK_METHOD0(rand, int(void));
};
