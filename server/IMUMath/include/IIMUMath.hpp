#pragma once

#include <DBusTypes.hpp>

#include <IIMUAbstraction.hpp>

namespace IMUMath
{

  enum class eIMUMathError
  {
    eRET_OK,
    eRET_ERROR,
  };

  class IIMUMath
  {
  public:
    IIMUMath() = default;

    /**
     * @brief Initialize IMU Abstraction
     */
    virtual eIMUMathError Init(void) = 0;

    /**
     * @brief Get Euler Angle of an axis
     *
     * @param value Get Angle Value
     * @param axis Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param unit Desired angle unit
     * @return eIMUMathError Returns #eRET_OK when successful, ref #eIMUMathError
     */
    virtual eIMUMathError GetEulerAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit) = 0;

    /**
     * @brief DeInitialize IMU Abstraction
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUMath() = default;
  };

} // namespace IMUMath
