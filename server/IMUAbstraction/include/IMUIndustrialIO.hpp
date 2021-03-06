#pragma once

#include <atomic>
#include <vector>
#include <thread>

#include "IIMUAbstraction.hpp"

namespace IMUAbstraction
{

  typedef struct imupathdata_s
  {
    std::string DeviceBufferPath;
    std::string DeviceAccelScalePath;
    std::string DeviceGyroScalePath;
    std::string DeviceSampleFreqPath;
    std::string DeviceBufferEnablePath;
  } imupathdata_t;

  typedef struct axisdata_s
  {
    std::string DeviceAccelPath;
    std::string DeviceAccelBufferEnPath;
    double accel;
    std::string DeviceGyroPath;
    std::string DeviceGyroBufferEnPath;
    double gyro;
  } axisdata_t;

  typedef struct imudata_s
  {
    imupathdata_s paths;
    eAccelScale accelScale;
    eGyroScale gyroScale;
    eSampleFreq sampleFreq;
    axisdata_t axisdata[NUM_AXIS];
  } imudata_t;

  class IMUIndustrialIO : public IIMUAbstraction
  {
  protected:
    std::vector<std::function<void()>> vecCallback;

    std::string DevicePath;
    imudata_t imu_data;

    std::atomic<bool> bThreadSampleValues;
    std::thread thSampleValues;

    eSampleFreq sampleFreq_ms;

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
      eGyroScale gyroScale=eGyroScale::Gyro_250dps,
      eSampleFreq sampleFreq=eSampleFreq::Freq_500ms
    );

    eIMUAbstractionError Init(void) override;
    void AddUpdateDataCallback(std::function<void()> &&cb) override;
    eIMUAbstractionError SetSampleFrequency(eSampleFreq freq) override;
    eIMUAbstractionError GetRawAccel(DBusTypes::eAxis axis, double &val) override;
    eIMUAbstractionError SetAccelScale(eAccelScale scale) override;
    eIMUAbstractionError GetRawGyro(DBusTypes::eAxis axis, double &val) override;
    eIMUAbstractionError SetGyroScale(eGyroScale scale) override;
    int GetSampleFrequency(void) override;
    void DeInit(void) override;

    virtual ~IMUIndustrialIO();
  };

} // namespace IMUAbstraction
