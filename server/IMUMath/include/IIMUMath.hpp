#pragma once

#include <IIMUAbstraction.hpp>

namespace IMUMath
{

  enum class eIMUMathError
  {
    eRET_OK,
    eRET_ERROR,
  };

  enum class eAngleUnit
  {
    eRadians,
    eDegrees,
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
     * @brief Get Euler Angles of an axis
     *
     * @param value Get Angle Value
     * @param axis Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param unit Desired angle unit
     * @return eIMUMathError Returns #eRET_OK when successful, ref #eIMUMathError
     */
    virtual eIMUMathError GetEulerAngles(double &value, IMUAbstraction::eAxis axis, const eAngleUnit &unit) = 0;

    /**
     * @brief DeInitialize IMU Abstraction
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUMath() = default;
  };

} // namespace IMUMath
