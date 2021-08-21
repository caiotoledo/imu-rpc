#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <IIMUAbstraction.hpp>
#include <IIMUAngle.hpp>

#include "IIMUMath.hpp"

namespace IMUMath
{

  class IMUMathImpl : public IIMUMath
  {
  private:
    std::shared_ptr<IMUAbstraction::IIMUAbstraction> instanceImu;
    std::shared_ptr<IMUAngle::IIMUAngle> instanceAngle;

    double const_alpha;
    double angle_compl_filter[IMUAbstraction::NUM_AXIS];

    void UpdateComplFilterAngle(double samplerate_ms);

  public:
    IMUMathImpl(
      std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu,
      std::shared_ptr<IMUAngle::IIMUAngle> angle,
      double alpha
    );

    eIMUMathError Init(void) override;
    eIMUMathError GetComplFilterAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit) override;
    void DeInit(void) override;

    virtual ~IMUMathImpl();
  };

} // namespace IMUMath
