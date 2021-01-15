#pragma once

#include <memory>

#include <IIMUAbstraction.hpp>

#include "IIMUMath.hpp"

namespace IMUMath
{

  class IMUMathImpl : public IIMUMath
  {
  private:
    std::shared_ptr<IMUAbstraction::IIMUAbstraction> instanceImu;

  public:
    IMUMathImpl(std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu);

    eIMUMathError Init(void);
    eIMUMathError GetEulerAngles(double &value, IMUAbstraction::eAxis axis, const eAngleUnit &unit);
    void DeInit(void);

    virtual ~IMUMathImpl();
  };

} // namespace IMUMath
