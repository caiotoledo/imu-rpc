#include <map>
#include <cmath>

#include <LogInstance.hpp>

#include <IMUMathImpl.hpp>

using namespace IMUMath;

#define DEG_TO_RAD(x)   (((double)x)*(M_PIl/(double)180.0))

IMUMathImpl::IMUMathImpl(
  std::shared_ptr<IMUAbstraction::IIMUAbstraction> imu,
  std::shared_ptr<IMUAngle::IIMUAngle> angle,
  double alpha) :
  instanceImu(imu),
  instanceAngle(angle),
  const_alpha(alpha)
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
      auto ret = this->instanceAngle->GetEulerAngle(angle_value, axis, DBusTypes::eAngleUnit::eDegrees);
      angle_compl_filter[axis_index] = (ret == IMUAngle::eIMUAngleError::eRET_OK) ? angle_value : 0;
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

void IMUMathImpl::UpdateComplFilterAngle(double samplerate_ms)
{
  for (size_t axis_index = 0; axis_index < IMUAbstraction::NUM_AXIS; axis_index++)
  {
    double gyro;
    double angle_measure;
    auto axis = static_cast<DBusTypes::eAxis>(axis_index);

    /* Get Raw Euler Angle */
    auto retMath = this->instanceAngle->GetEulerAngle(angle_measure, axis, DBusTypes::eAngleUnit::eDegrees);
    if (retMath != IMUAngle::eIMUAngleError::eRET_OK)
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
    angle_compl_filter[axis_index] =  (angle_compl_filter[axis_index] + (gyro*samplerate_sec))*const_alpha;
    angle_compl_filter[axis_index] += (1-const_alpha)*(angle_measure);
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
    ret = eIMUMathError::eRET_INVALID_PARAMETER;
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
    ret = eIMUMathError::eRET_INVALID_PARAMETER;
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
