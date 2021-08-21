#include <map>
#include <cmath>

#include <LogInstance.hpp>

#include <IMUEulerAngle.hpp>

using namespace IMUAngle;

#define RAD_TO_DEG(x)   (((double)x)*((double)180.0/M_PIl))
#define DEG_TO_RAD(x)   (((double)x)*(M_PIl/(double)180.0))

IMUEulerAngle::IMUEulerAngle(std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu) :
  instanceImu(imu)
{
}

eIMUAngleError IMUEulerAngle::GetEulerAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit)
{
  auto ret = eIMUAngleError::eRET_OK;
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
    auto axis = static_cast<DBusTypes::eAxis>(index);

    auto ret = this->instanceImu->GetRawAccel(axis, valAccel[index]);
    if (ret != IMUAbstraction::eIMUAbstractionError::eRET_OK)
    {
      LOGWARN("Fail to GetRawAccel Axis[%d] ret[%d]", axis, ret);
      /* TODO: Create table to convert from eIMUAbstractionError to eIMUAngleError */
      return static_cast<eIMUAngleError>(ret);
    }
  }

  /* Convert Raw Accelerometer Data to Angle rad */
  switch (axis)
  {
  case DBusTypes::eAxis::X:
    valueAngle = calcAngleRad(-valAccel[1], -valAccel[2]); /* atan2(-Y,-Z) */
    break;
  case DBusTypes::eAxis::Y:
    valueAngle = calcAngleRad(-valAccel[0], -valAccel[2]); /* atan2(-X,-Z) */
    break;
  case DBusTypes::eAxis::Z:
    valueAngle = calcAngleRad(-valAccel[1], -valAccel[0]); /* atan2(-Y,-X) */
    break;
  default:
    ret = eIMUAngleError::eRET_INVALID_PARAMETER;
    break;
  }

  /* Convert Angle unit */
  switch (unit)
  {
  case DBusTypes::eAngleUnit::eDegrees:
    value = RAD_TO_DEG(valueAngle);
    break;
  case DBusTypes::eAngleUnit::eRadians:
    value = valueAngle;
    break;
  default:
    ret = eIMUAngleError::eRET_INVALID_PARAMETER;
    break;
  }

  return ret;
}
