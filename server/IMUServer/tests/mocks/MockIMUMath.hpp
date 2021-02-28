#pragma once

#include <gmock/gmock.h>

#include <IIMUMath.hpp>

namespace IMUMath
{

  class MockIMUAbstraction : public IIMUMath {
  public:
    MOCK_METHOD0(Init, eIMUMathError(void));
    MOCK_METHOD3(GetEulerAngle, eIMUMathError(double &, DBusTypes::eAxis, const DBusTypes::eAngleUnit &));
    MOCK_METHOD3(GetComplFilterAngle, eIMUMathError(double &, DBusTypes::eAxis, const DBusTypes::eAngleUnit &));
    MOCK_METHOD0(DeInit, void(void));
  };

} // namespace IMUMath
