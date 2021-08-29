#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <IIMUAbstraction.hpp>

#include "IIMUAngle.hpp"

namespace IMUAngle
{

  class IMUEulerAngle : public IIMUAngle
  {
  private:
    std::shared_ptr<IMUAbstraction::IIMUAbstraction> instanceImu;

  public:
    IMUEulerAngle(
      std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu
    );

    eIMUAngleError GetEulerAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit) override;

    virtual ~IMUEulerAngle() = default;
  };

} // namespace IMUAngle
