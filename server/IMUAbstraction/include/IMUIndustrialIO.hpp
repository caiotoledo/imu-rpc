#pragma once

#include <vector>

#include "IIMUAbstraction.hpp"

namespace IMUAbstraction
{

  class IMUIndustrialIO : public IIMUAbstraction
  {
  private:
    std::vector<std::function<void()>> vecCallback;

  public:
    IMUIndustrialIO();

    void AddUpdateDataCallback(std::function<void()> &&cb) override;
    eIMUAbstractionError GetRawAccel(eAxis axis, double &val) override;
    eIMUAbstractionError GetRawGyro(eAxis axis, double &val) override;

    virtual ~IMUIndustrialIO();
  };

} // namespace IMUServer
