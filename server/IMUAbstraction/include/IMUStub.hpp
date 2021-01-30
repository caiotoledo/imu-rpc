#pragma once

#include <atomic>
#include <vector>
#include <thread>

#include "IIMUAbstraction.hpp"

#include "IValueGenerator.hpp"

namespace IMUAbstraction
{

  class IMUStub : public IIMUAbstraction
  {
  private:
    std::shared_ptr<IMUAbstraction::IValueGenerator> instanceGen;

    std::atomic<bool> bThreadNotification;
    std::thread thNotification;
    std::vector<std::function<void()>> vecCallback;

    eAccelScale accelScale;
    eGyroScale gyroScale;
    eSampleFreq sampleFreq;

    template <typename T>
    T GetAccelScale();

    template <typename T>
    T GetGyroScale();

    template <typename T>
    T GetSampleFrequency();

  public:
    IMUStub(
      std::shared_ptr<IMUAbstraction::IValueGenerator> generator,
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
    void DeInit(void) override;

    virtual ~IMUStub();
  };

} // namespace IMUAbstraction
