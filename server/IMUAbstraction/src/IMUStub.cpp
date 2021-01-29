#include <map>
#include <thread>
#include <chrono>
#include <cmath>

#include <cstdlib>
#include <ctime>

#include <IMUStub.hpp>

#include <LogInstance.h>

#define DEG_TO_RAD(x)   (((double)x)*(M_PIl/(double)180.0))
#define GET_CURR_MILLI  (((double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count())/1000.0)

using namespace IMUAbstraction;

constexpr int MAX_AXIS = 3;
constexpr auto FREQ_SIN = 0.5;
constexpr auto AXIS_PHASE_SHIFT = 90.0;

double accel[MAX_AXIS] = {0};
double gyro[MAX_AXIS] = {0};

const std::map<eAccelScale, int> mapAccelScale =
{
  {eAccelScale::Accel_2g,  2000*100},
  {eAccelScale::Accel_4g,  4000*100},
  {eAccelScale::Accel_8g,  8000*100},
  {eAccelScale::Accel_16g, 16000*100},
};

const std::map<eGyroScale, int> mapGyroScale =
{
  {eGyroScale::Gyro_250dps,  250*100},
  {eGyroScale::Gyro_500dps,  500*100},
  {eGyroScale::Gyro_1000dps, 1000*100},
  {eGyroScale::Gyro_2000dps, 2000*100},
};

const std::map<eSampleFreq, int> mapSampleFreq =
{
  {eSampleFreq::Freq_10ms,  10},
  {eSampleFreq::Freq_20ms,  20},
  {eSampleFreq::Freq_50ms,  50},
  {eSampleFreq::Freq_100ms, 100},
  {eSampleFreq::Freq_200ms, 200},
  {eSampleFreq::Freq_500ms, 500},
};

IMUStub::IMUStub(eAccelScale accelScale, eGyroScale gyroScale, eSampleFreq sampleFreq) :
  accelScale(accelScale), gyroScale(gyroScale), sampleFreq(sampleFreq)
{
}

double IMUStub::GetRandomAccel(void)
{
  bool signal = ((bool)(rand() % 2));

  auto scale = GetAccelScale<int>();
  double r = (rand() % scale);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}
double IMUStub::GetRandomGyro(void)
{
  bool signal = ((bool)(rand() % 2));

  auto scale = GetGyroScale<int>();
  double r = (rand() % scale);
  double ret = (r/100);

  ret = signal ? ret : (-ret);

  return ret;
}

double IMUStub::SinValue(double amplitude, double freq, double timepoint, double phaseshift)
{
  auto phaseshift_rad = DEG_TO_RAD(phaseshift);
  auto ret = amplitude * sin(2L * M_PIl * freq * timepoint + phaseshift_rad);
  return ret;
}

double IMUStub::GetSinAccel(double phaseshift)
{
  double amp = GetAccelScale<double>();
  double freq = FREQ_SIN;
  double now = GET_CURR_MILLI;

  double ret = this->SinValue(amp, freq, now, phaseshift);
  ret /= 100.0;

  return ret;
}

double IMUStub::GetSinGyro(double phaseshift)
{
  double amp = GetGyroScale<double>();
  double freq = FREQ_SIN;
  double duration = GET_CURR_MILLI;

  double ret = this->SinValue(amp, freq, duration, phaseshift);
  ret /= 100.0;

  return ret;
}

eIMUAbstractionError IMUStub::Init(void)
{
  auto ret = eIMUAbstractionError::eRET_OK;

  /* Random Seed Initialization */
  srand((unsigned) time(0));

  /* IMU Configuration */
  auto retAccel = static_cast<int>(this->SetAccelScale(this->accelScale));
  auto retGyro = static_cast<int>(this->SetGyroScale(this->gyroScale));
  auto retSample = static_cast<int>(this->SetSampleFrequency(this->sampleFreq));
  ret = static_cast<eIMUAbstractionError>(retAccel + retGyro + retSample);

  /* Initialize Sampling IMU Thread if needed */
  if ((!thNotification.joinable()) && (ret == eIMUAbstractionError::eRET_OK))
  {
    bThreadNotification = true;
    auto func = [this]()
    {
      while (bThreadNotification)
      {
        for (size_t i = 0; i < (sizeof(accel)/sizeof(accel[0])); i++)
        {
          double shift = i * AXIS_PHASE_SHIFT;
          accel[i] = this->GetSinAccel(shift);
          gyro[i] = this->GetSinGyro(shift+AXIS_PHASE_SHIFT);
        }

        for(auto &cb : vecCallback)
        {
          cb();
        }

        auto sample_delay = this->GetSampleFrequency<int>();
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_delay));
      }
    };
    thNotification = std::thread(func);
    if (!thNotification.joinable())
    {
      /* Thread not initialized */
      ret = eIMUAbstractionError::eRET_ERROR;
    }
  }

  return ret;
}

void IMUStub::AddUpdateDataCallback(std::function<void()> &&cb)
{
  vecCallback.push_back(cb);
}

eIMUAbstractionError IMUStub::SetSampleFrequency(eSampleFreq freq)
{
  this->sampleFreq = freq;
  LOGDEBUG("Set Frequency Scale [%d]->[%d ms]", freq, static_cast<int>(mapSampleFreq.at(this->sampleFreq)));

  return eIMUAbstractionError::eRET_OK;
}

eIMUAbstractionError IMUStub::SetAccelScale(eAccelScale scale)
{
  this->accelScale = scale;
  LOGDEBUG("Set Accel Scale [%d]->[%0.2f]", scale, static_cast<double>(mapAccelScale.at(this->accelScale)));

  return eIMUAbstractionError::eRET_OK;
}
eIMUAbstractionError IMUStub::SetGyroScale(eGyroScale scale)
{
  this->gyroScale = scale;
  LOGDEBUG("Set Gyro Scale [%d]->[%0.2f]", scale, static_cast<double>(mapGyroScale.at(this->gyroScale)));

  return eIMUAbstractionError::eRET_OK;
}
template <typename T>
T IMUStub::GetAccelScale()
{
  return static_cast<T>(mapAccelScale.at(this->accelScale));
}
template <typename T>
T IMUStub::GetGyroScale()
{
  return static_cast<T>(mapGyroScale.at(this->gyroScale));
}
template <typename T>
T IMUStub::GetSampleFrequency()
{
  return static_cast<T>(mapSampleFreq.at(this->sampleFreq));
}

eIMUAbstractionError IMUStub::GetRawAccel(DBusTypes::eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  int axisVal = (int)axis;

  auto sizeAccelArray = (int)(sizeof(accel)/sizeof(accel[0]));
  if ( axisVal >= 0 && axisVal <= sizeAccelArray)
  {
    val = accel[axisVal];
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}
eIMUAbstractionError IMUStub::GetRawGyro(DBusTypes::eAxis axis, double &val)
{
  auto ret = eIMUAbstractionError::eRET_OK;
  int axisVal = (int)axis;

  auto sizeGyroArray = (int)(sizeof(gyro)/sizeof(gyro[0]));
  if ( axisVal >= 0 && axisVal <= sizeGyroArray)
  {
    val = gyro[axisVal];
  }
  else
  {
    val = 0;
    ret = eIMUAbstractionError::eRET_ERROR;
  }

  return ret;
}

void IMUStub::DeInit(void)
{
  if (thNotification.joinable())
  {
    bThreadNotification = false;
    thNotification.join();
  }
}

IMUStub::~IMUStub()
{
  this->DeInit();
}
