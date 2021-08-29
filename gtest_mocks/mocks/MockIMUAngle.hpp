#pragma once

#include <gmock/gmock.h>

#include <IIMUAngle.hpp>

namespace IMUAngle
{

  class MockIMUAngle : public IIMUAngle {
  public:
    MOCK_METHOD3(GetEulerAngle, eIMUAngleError(double &, DBusTypes::eAxis, const DBusTypes::eAngleUnit &));
  };

} // namespace IMUAngle
