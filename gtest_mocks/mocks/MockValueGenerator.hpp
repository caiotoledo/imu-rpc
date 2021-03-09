#pragma once

#include <gmock/gmock.h>

#include <IValueGenerator.hpp>

namespace IMUAbstraction
{

  class MockValueGenerator : public IValueGenerator {
  public:
    MOCK_METHOD3(GetRawAccel, eIMUAbstractionError(DBusTypes::eAxis, double &, double));
    MOCK_METHOD3(GetRawGyro, eIMUAbstractionError(DBusTypes::eAxis, double &, double));
  };

} // namespace IMUAbstraction
