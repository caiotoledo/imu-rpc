#pragma once

#include <vector>
#include <thread>

#include "IIMUAbstraction.hpp"

namespace IMUAbstraction
{

  class IMUStub : public IIMUAbstraction
  {
  private:
    bool bThreadNotification;
    std::thread thNotification;
    std::vector<std::function<void()>> vecCallback;

    eAccelScale accelScale;
    eGyroScale gyroScale;

    template <typename T>
    T GetAccelScale();

    template <typename T>
    T GetGyroScale();

    double GetRandomAccel(void);
    double GetRandomGyro(void);

  public:
    IMUStub(eAccelScale accelScale=eAccelScale::Accel_2g, eGyroScale gyroScale=eGyroScale::Gyro_250);

    void AddUpdateDataCallback(std::function<void()> &&cb) override;
    eIMUAbstractionError SetSampleFrequency(eSampleFreq freq) override;
    eIMUAbstractionError GetRawAccel(eAxis axis, double &val) override;
    eIMUAbstractionError SetAccelScale(eAccelScale scale) override;
    eIMUAbstractionError GetRawGyro(eAxis axis, double &val) override;
    eIMUAbstractionError SetGyroScale(eGyroScale scale) override;

    virtual ~IMUStub();
  };

} // namespace IMUServer
