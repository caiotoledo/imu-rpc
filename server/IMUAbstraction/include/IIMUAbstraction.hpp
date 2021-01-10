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

  enum class eAccelScale {
    Accel_2g,
    Accel_4g,
    Accel_8g,
    Accel_16g,
  };

  enum class eGyroScale {
    Gyro_250,
    Gyro_500,
    Gyro_1000,
    Gyro_2000,
  };

  enum class eSampleFreq {
    Freq_10ms,
    Freq_20ms,
    Freq_50ms,
    Freq_100ms,
    Freq_200ms,
    Freq_500ms,
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
     * @brief Set Sample Frequency in ms
     *
     * @param freq Sample period in ms, ref #eSampleFreq
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError SetSampleFrequency(eSampleFreq freq) = 0;

    /**
     * @brief Get raw accelerometer data
     *
     * @param axis IMU Axis
     * @param val Accelerometer value in milli g-force
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError GetRawAccel(eAxis axis, double &val) = 0;

    /**
     * @brief Set Accel Scale
     *
     * @param scale Accelerometer Scale, ref #eAccelScale
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError SetAccelScale(eAccelScale scale) = 0;

    /**
     * @brief Get raw gyroscope data
     *
     * @param axis IMU axis
     * @param val Gyroscope value in Degrees per second
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError GetRawGyro(eAxis axis, double &val) = 0;

    /**
     * @brief Set Gyro Scale
     *
     * @param scale Gyroscope Scale, ref #eGyroScale
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError SetGyroScale(eGyroScale scale) = 0;

    virtual ~IIMUAbstraction() = default;
  };

} // namespace IMUServer
