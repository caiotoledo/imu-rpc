#pragma once

#include <vector>

#include "IIMUAbstraction.hpp"

namespace IMUAbstraction
{

  constexpr int NUM_AXIS = 3;

  typedef struct axisdata_s
  {
    std::string DeviceAccelPath;
    double accel;
    std::string DeviceGyroPath;
    double gyro;
  } axisdata_t;

  typedef struct imudata_s
  {
    std::string DeviceAccelScalePath;
    std::string DeviceGyroScalePath;
    eAccelScale accelScale;
    eGyroScale gyroScale;
    axisdata_t axisdata[NUM_AXIS];
  } imudata_t;

  class IMUIndustrialIO : public IIMUAbstraction
  {
  private:
    std::vector<std::function<void()>> vecCallback;

    std::string DevicePath;
    imudata_t imu_data;

    bool bThreadSampleValues;
    std::thread thSampleValues;

    /**
     * @brief Get value from first line of a File
     *
     * @tparam T Output data type
     * @param path Path of the file
     * @return T Value read from the file
     */
    template <typename T>
    T GetValueInFile(const char *path);

    /**
     * @brief Write a value in a File
     *
     * @tparam T Input data type
     * @param path Path to the file
     * @param val Value to write in a file
     * @return eIMUAbstractionError Returns #eRET_OK when successful, ref #eIMUAbstractionError
     */
    template <typename T>
    eIMUAbstractionError SetValueInFile(const char *path, T val);

    /**
     * @brief Initialize the attributes in the class based on the Industrial IO device path
     *
     * @param sDevicePath Industrial IO device path
     */
    void InitializePaths(std::string const &sDevicePath);

    /**
     * @brief Notify callbacks registered about the new data available
     */
    void NotifyUpdateData(void);

  public:
    IMUIndustrialIO(
      const char *path,
      int device_index,
      eAccelScale accelScale=eAccelScale::Accel_2g,
      eGyroScale gyroScale=eGyroScale::Gyro_250
    );

    void AddUpdateDataCallback(std::function<void()> &&cb) override;
    eIMUAbstractionError SetSampleFrequency(eSampleFreq freq) override;
    eIMUAbstractionError GetRawAccel(eAxis axis, double &val) override;
    eIMUAbstractionError SetAccelScale(eAccelScale scale) override;
    eIMUAbstractionError GetRawGyro(eAxis axis, double &val) override;
    eIMUAbstractionError SetGyroScale(eGyroScale scale) override;

    virtual ~IMUIndustrialIO();
  };

} // namespace IMUServer
