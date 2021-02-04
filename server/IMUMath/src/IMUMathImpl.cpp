#include <map>
#include <cmath>

#include <LogInstance.h>

#include <IMUMathImpl.hpp>

using namespace IMUMath;

#define RAD_TO_DEG(x)   (((double)x)*((double)180.0/M_PIl))
#define DEG_TO_RAD(x)   (((double)x)*(M_PIl/(double)180.0))

/* Constant used in Complementary Filter */
constexpr double ALPHA = 0.7143;

IMUMathImpl::IMUMathImpl(std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu) :
  instanceImu(imu)
{
}

eIMUMathError IMUMathImpl::Init(void)
{
  /* TODO: Create table to convert from eIMUAbstractionError to eIMUMathError */
  auto ret = static_cast<eIMUMathError>(this->instanceImu->Init());

  if (ret == eIMUMathError::eRET_OK)
  {
    /* Initialize Complementary Filter Angle with Raw Euler Angle */
    for (size_t axis_index = 0; axis_index < IMUAbstraction::NUM_AXIS; axis_index++)
    {
      double angle_value;
      auto axis = static_cast<DBusTypes::eAxis>(axis_index);
      this->GetEulerAngle(angle_value, axis, DBusTypes::eAngleUnit::eDegrees);
      angle_compl_filter[axis_index] = angle_value;
    }

    /* Define lambda function for Complementary Filter Calculation */
    auto funcComplFilter = [this]()
    {
      /* Calculate sample rate based on callback call */
      static auto last_exec = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now - last_exec);
      double samplerate_ms = (diff.count()/1000.0);

      if (samplerate_ms > 0)
      {
        /* Calculate Complementary Filter */
        this->UpdateComplFilterAngle(samplerate_ms);
      }

      /* Update last executiion time */
      last_exec = std::chrono::high_resolution_clock::now();
    };
    this->instanceImu->AddUpdateDataCallback(funcComplFilter);
  }

  return ret;
}

eIMUMathError IMUMathImpl::GetEulerAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit)
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
    auto axis = static_cast<DBusTypes::eAxis>(index);

    auto ret = this->instanceImu->GetRawAccel(axis, valAccel[index]);
    if (ret != IMUAbstraction::eIMUAbstractionError::eRET_OK)
    {
      LOGWARN("Fail to GetRawAccel Axis[%d] ret[%d]", axis, ret);
      /* TODO: Create table to convert from eIMUAbstractionError to eIMUMathError */
      return static_cast<eIMUMathError>(ret);
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
    ret = eIMUMathError::eRET_ERROR;
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
    ret = eIMUMathError::eRET_ERROR;
    break;
  }

  return ret;
}

void IMUMathImpl::UpdateComplFilterAngle(double samplerate_ms)
{
  for (size_t axis_index = 0; axis_index < IMUAbstraction::NUM_AXIS; axis_index++)
  {
    double gyro;
    double angle_measure;
    auto axis = static_cast<DBusTypes::eAxis>(axis_index);

    /* Get Raw Euler Angle */
    auto retMath = this->GetEulerAngle(angle_measure, axis, DBusTypes::eAngleUnit::eDegrees);
    if (retMath != eIMUMathError::eRET_OK)
    {
      LOGWARN("GetEulerAngle failed [%d]", retMath);
      continue;
    }
    /* Get Gyro Value */
    auto retImu = this->instanceImu->GetRawGyro(axis, gyro);
    if (retImu != IMUAbstraction::eIMUAbstractionError::eRET_OK)
    {
      LOGWARN("GetRawGyro failed [%d]", retImu);
      continue;
    }

    /* Calculate Complamentary Filter Angle */
    double samplerate_sec = (samplerate_ms/1000L);
    angle_compl_filter[axis_index] =  (angle_compl_filter[axis_index] + (gyro*samplerate_sec))*ALPHA;
    angle_compl_filter[axis_index] += (1-ALPHA)*(angle_measure);
  }
}

eIMUMathError IMUMathImpl::GetComplFilterAngle(double &value, DBusTypes::eAxis axis, const DBusTypes::eAngleUnit &unit)
{
  auto ret = eIMUMathError::eRET_OK;
  double valueAngle = 0;

  switch (axis)
  {
  case DBusTypes::eAxis::X:
  case DBusTypes::eAxis::Y:
  case DBusTypes::eAxis::Z:
  {
    auto axis_index = static_cast<int>(axis);
    valueAngle = angle_compl_filter[axis_index];
  }
    break;
  default:
    ret = eIMUMathError::eRET_ERROR;
    break;
  }

  /* Convert Angle unit */
  switch (unit)
  {
  case DBusTypes::eAngleUnit::eDegrees:
    value = valueAngle;
    break;
  case DBusTypes::eAngleUnit::eRadians:
    value = DEG_TO_RAD(valueAngle);
    break;
  default:
    ret = eIMUMathError::eRET_ERROR;
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
