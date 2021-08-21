#pragma once

#include <DBusTypes.hpp>

#include <IIMUAbstraction.hpp>

namespace IMUAngle
{

  enum class eIMUAngleError
  {
    eRET_OK,
    eRET_ERROR,
    eRET_INVALID_PARAMETER,
  };

  class IIMUAngle
  {
  public:
    IIMUAngle() = default;

    /**
     * @brief Get Euler Angle of an axis
     *
     * @param value Get Angle Value
     * @param axis Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param unit Desired angle unit
     * @return eIMUAngleError Returns #eRET_OK when successful, ref #eIMUAngleError
     */
    virtual eIMUAngleError GetEulerAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit) = 0;

    virtual ~IIMUAngle() = default;
  };

} // namespace IMUAngle
