#pragma once

#include <DBusTypes.hpp>

namespace IMUClient
{

  enum class eIMUError {
    eRET_OK,
    eRET_ERROR
  };

  class IIMUClient
  {
  public:
    IIMUClient() = default;

    /**
     * @brief Initialize RPC Client
     *
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError Init(void) = 0;

    /**
     * @brief Set callback notification which is trigger whenever the IMU data was updated
     *
     * @param cb Function callback
     */
    virtual void AddUpdateDataCallback(std::function<void()> &&cb) = 0;

    /**
     * @brief Get Raw Accelerometer data
     *
     * @param axis IMU axis
     * @param val Value reference to receive the data
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError GetRawAccel(DBusTypes::eAxis axis, double &val) = 0;

    /**
     * @brief Get Raw Accelerometer data
     *
     * @param axis IMU axis
     * @param val Value reference to receive the data
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError GetRawGyro(DBusTypes::eAxis axis, double &val) = 0;

    /**
     * @brief Get Euler Angle of an axis
     *
     * @param axis Get Angle Value
     * @param unit Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param val Desired angle unit
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError GetEulerAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val) = 0;

    /**
     * @brief Get Euler Angle with Complementary Filter of an axis
     *
     * @param axis Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param unit Desired angle unit
     * @param value Angle Value
     * @return eIMUMathError Returns #eRET_OK when successful, ref #eIMUMathError
     */
    virtual eIMUError GetComplFilterAngle(DBusTypes::eAxis axis, DBusTypes::eAngleUnit unit, double &val) = 0;

    /**
     * @brief DeInitialize RPC Client
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUClient() = default;
  };

} // namespace RPCClient
