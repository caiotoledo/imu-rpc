#pragma once

namespace IMUClient
{

  enum class eIMUError {
    eRET_OK,
    eRET_ERROR
  };

  enum class eAxis {
    X,
    Y,
    Z
  };

  enum class eAngleUnit
  {
    eRadians,
    eDegrees,
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
    virtual eIMUError GetRawAccel(eAxis axis, double &val) = 0;

    /**
     * @brief Get Raw Accelerometer data
     *
     * @param axis IMU axis
     * @param val Value reference to receive the data
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError GetRawGyro(eAxis axis, double &val) = 0;

    /**
     * @brief Get Euler Angle of an axis
     *
     * @param axis Get Angle Value
     * @param unit Desired axis (X = Roll, Y = Pitch, Z = Yaw)
     * @param val Desired angle unit
     * @return eIMUError Returns #eRET_OK when successful, ref #eIMUError
     */
    virtual eIMUError GetEulerAngle(eAxis axis, eAngleUnit unit, double &val) = 0;

    /**
     * @brief DeInitialize RPC Client
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUClient() = default;
  };

} // namespace RPCClient
