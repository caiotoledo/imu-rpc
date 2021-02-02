#pragma once

#include <functional>

#include <DBusTypes.hpp>

namespace IMUAbstraction
{
  constexpr int NUM_AXIS = 3;

  enum class eIMUAbstractionError {
    eRET_OK,
    eRET_ERROR,
  };

  enum class eAccelScale {
    Accel_2g,
    Accel_4g,
    Accel_8g,
    Accel_16g,
  };

  enum class eGyroScale {
    Gyro_250dps,
    Gyro_500dps,
    Gyro_1000dps,
    Gyro_2000dps,
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
     * @brief Initialize IMU Abstraction
     */
    virtual eIMUAbstractionError Init(void) = 0;

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
    virtual eIMUAbstractionError GetRawAccel(DBusTypes::eAxis axis, double &val) = 0;

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
    virtual eIMUAbstractionError GetRawGyro(DBusTypes::eAxis axis, double &val) = 0;

    /**
     * @brief Set Gyro Scale
     *
     * @param scale Gyroscope Scale, ref #eGyroScale
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    virtual eIMUAbstractionError SetGyroScale(eGyroScale scale) = 0;

    /**
     * @brief DeInitialize IMU Abstraction
     */
    virtual void DeInit(void) = 0;

    virtual ~IIMUAbstraction() = default;
  };

} // namespace IMUAbstraction
