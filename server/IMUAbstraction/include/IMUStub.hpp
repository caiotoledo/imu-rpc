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

    double GetRandomAccel(void);
    double GetRandomGyro(void);

  public:
    IMUStub();

    void AddUpdateDataCallback(std::function<void()> &&cb) override;
    eIMUAbstractionError GetRawAccel(eAxis axis, double &val) override;
    eIMUAbstractionError GetRawGyro(eAxis axis, double &val) override;

    virtual ~IMUStub();
  };

} // namespace IMUServer
