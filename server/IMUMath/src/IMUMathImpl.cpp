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

eIMUMathError IMUMathImpl::GetEulerAngles(double &axis_x, double &axis_y, double &axis_z, const eAngleUnit &unit)
{
  auto ret = eIMUMathError::eRET_OK;

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
  axis_x = calcAngleRad(-valAccel[1], -valAccel[2]); /* atan2(-Y,-Z) */
  axis_y = calcAngleRad(-valAccel[0], -valAccel[2]); /* atan2(-X,-Z) */
  axis_z = calcAngleRad(-valAccel[1], -valAccel[0]); /* atan2(-Y,-X) */

  /* Convert Angle unit */
  switch (unit)
  {
  case eAngleUnit::eDegrees:
    axis_x = RAD_TO_DEG(axis_x);
    axis_y = RAD_TO_DEG(axis_y);
    axis_z = RAD_TO_DEG(axis_z);
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
