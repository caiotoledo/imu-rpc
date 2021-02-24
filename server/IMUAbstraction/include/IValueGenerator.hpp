#pragma once

#include "IIMUAbstraction.hpp"

namespace IMUAbstraction
{

  class IValueGenerator
  {
  public:
    IValueGenerator() = default;

    virtual eIMUAbstractionError GetRawAccel(DBusTypes::eAxis axis, double &val, double scale) = 0;
    virtual eIMUAbstractionError GetRawGyro (DBusTypes::eAxis axis, double &val, double scale) = 0;

    ~IValueGenerator() = default;
  };

} // namespace IMUAbstraction
