#pragma once

#include <functional>

namespace IMUAbstraction
{

  enum class eIMUAbstractionError {
    eRET_OK,
    eRET_ERROR,
  };

  enum class eAxis {
    X,
    Y,
    Z
  };

  class IIMUAbstraction
  {
  public:
    IIMUAbstraction() = default;

    /**
     * @brief Set callback notification which is trigger whenever the IMU data was updated
     *
     * @param cb Function callback
     */
    virtual void AddUpdateDataCallback(std::function<void()> &&cb) = 0;

    /**
     * @brief Get raw accelerometer data
     *
     * @param axis IMU Axis
     * @param val Value reference to receive the data
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eIMUAbstractionError GetRawAccel(eAxis axis, double &val) = 0;

    /**
     * @brief Get raw gyroscope data
     *
     * @param axis IMU axis
     * @param val Value reference to receive the data
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eRPCError
     */
    virtual eIMUAbstractionError GetRawGyro(eAxis axis, double &val) = 0;

    virtual ~IIMUAbstraction() = default;
  };

} // namespace IMUServer
