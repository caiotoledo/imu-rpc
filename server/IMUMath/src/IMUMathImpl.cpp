#include <map>
#include <cmath>

#include <LogInstance.h>

#include <IMUMathImpl.hpp>

using namespace IMUMath;

#define RAD_TO_DEG(x)   (((double)x)*((double)180.0/M_PI))

IMUMathImpl::IMUMathImpl(std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu) :
  instanceImu(imu)
{
}

eIMUMathError IMUMathImpl::Init(void)
{
  /* TODO: Create table to convert from eIMUAbstractionError to eIMUMathError */
  return static_cast<eIMUMathError>(this->instanceImu->Init());
}

eIMUMathError IMUMathImpl::GetEulerAngle(double &value, IMUAbstraction::eAxis axis, const eAngleUnit &unit)
{
  auto ret = eIMUMathError::eRET_OK;
  double valueAngle = 0;

  /* Define function to convert Accelerometer Value to Angle rad */
  auto calcAngleRad = [](double catOp, double catAdj)
  {
    return atan2(catOp, catAdj) + M_PIl;
  };

  /* Get Raw Accelerometer Data */
  double valAccel[3];
  for (size_t i = 0; i < (sizeof(valAccel)/sizeof(double)); i++)
  {
    auto index = i;
    auto eAxis = static_cast<IMUAbstraction::eAxis>(index);

    auto ret = this->instanceImu->GetRawAccel(eAxis, valAccel[index]);
    if (ret != IMUAbstraction::eIMUAbstractionError::eRET_OK)
    {
      LOGWARN("Fail to GetRawAccel Axis[%d] ret[%d]", eAxis, ret);
      /* TODO: Create table to convert from eIMUAbstractionError to eIMUMathError */
      return static_cast<eIMUMathError>(ret);
    }
  }

  /* Convert Raw Accelerometer Data to Angle rad */
  switch (axis)
  {
  case IMUAbstraction::eAxis::X:
    valueAngle = calcAngleRad(-valAccel[1], -valAccel[2]); /* atan2(-Y,-Z) */
    break;
  case IMUAbstraction::eAxis::Y:
    valueAngle = calcAngleRad(-valAccel[0], -valAccel[2]); /* atan2(-X,-Z) */
    break;
  case IMUAbstraction::eAxis::Z:
    valueAngle = calcAngleRad(-valAccel[1], -valAccel[0]); /* atan2(-Y,-X) */
    break;
  default:
    break;
  }

  /* Convert Angle unit */
  switch (unit)
  {
  case eAngleUnit::eDegrees:
    value = RAD_TO_DEG(valueAngle);
    break;
  case eAngleUnit::eRadians:
  default:
    break;
  }

  return ret;
}

void IMUMathImpl::DeInit(void)
{
  this->instanceImu->DeInit();
}

IMUMathImpl::~IMUMathImpl()
{
  this->DeInit();
}
