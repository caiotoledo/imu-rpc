#pragma once

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
     * @brief Get Euler Angles
     *
     * @param axis_x Get Roll axis angle
     * @param axis_y Get Pitch axis angle
     * @param axis_z Get Yaw axis angle
     * @param unit Desired angle unit
     * @return eIMUMathError Returns #eRET_OK when successful, ref #eIMUMathError
     */
    virtual eIMUMathError GetEulerAngles(double &axis_x, double &axis_y, double &axis_z, const eAngleUnit &unit) = 0;

    /**
     * @brief DeInitialize IMU Abstraction
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUMath() = default;
  };

} // namespace IMUMath
