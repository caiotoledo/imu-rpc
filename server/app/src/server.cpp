#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <csignal>
#include <iostream>
#include <memory>
#include <atomic>
#include <fstream>

#include <LogInstance.hpp>

#include <DBusCxxServer.hpp>

#include <IMURPCServer.hpp>

#include <IMUStub.hpp>
#include <IMUIndustrialIO.hpp>
#include <IMUBufferIIO.hpp>

#include <ValueGenImpl.hpp>

#include <IMUEulerAngle.hpp>

#include <IMUMathImpl.hpp>

#include "argvalidator.hpp"

/* Path to iio devices */
#define DEVICE_PATH   "/sys/bus/iio/devices/"

/* Using IMU Industrial IO Buffering */
#define IMU_BUFFER_IIO

/* Uses Sin Wave Generator class in IMU Stub */
#define SINWAVE_GENERATOR

constexpr auto DELAY_CHILD_CREATION = std::chrono::milliseconds(1);

#ifdef SINWAVE_GENERATOR
constexpr auto FREQ_SIN = 0.5;
constexpr auto AXIS_PHASE_SHIFT = 90.0;
#endif

std::atomic<sig_atomic_t> g_run = 1;
static void signal_handler(int signum)
{
  LOGDEBUG("Received signal [%d]", signum);
  g_run = 0;
}

static void init_signal_handler(void)
{
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = signal_handler;
  sigaction(SIGABRT, &action, NULL);
  sigaction(SIGINT, &action, NULL);
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGQUIT, &action, NULL);
}

static pid_t daemonize(void)
{
  pid_t pid = fork();
  if (pid < 0)
  {
    LOG("Daemon creation failed [%s]", strerror(errno));
    exit(pid);
  }
  else if (pid > 0)
  {
    LOG("Daemon created with PID %d", pid);
    std::this_thread::sleep_for(DELAY_CHILD_CREATION);
    exit(0);
  }
  else
  {
    /* Detach child from parent */
    pid = setsid();
    signal(SIGHUP, SIG_IGN);
    LOGDEBUG("Running Child process [PID %d]", pid);
  }

  return pid;
}

static bool path_isvalid(const char *path) {
  struct stat st;
  return (stat(path, &st) >= 0);
}

template <typename T>
T GetValueInFile(const char *path)
{
  T value = {0};

  std::ifstream ifFile(path);
  if (ifFile.is_open())
  {
    std::string strValue;
    if (std::getline(ifFile, strValue))
    {
      std::stringstream sstrValue;
      sstrValue << strValue;
      sstrValue >> value;
    }
  }
  else
  {
    LOGERROR("Invalid Path!");
  }

  return value;
}

int main(int argc, char const *argv[])
{
  /**
   * PARSE ARGUMENTS
   */
  ArgParser::arguments args;
  auto retParse = ArgParser::iProcessArgs(argc, argv, args);
  if (retParse != 0)
  {
    LOGERROR("iProcessArgs failed [%d]", retParse);
    return retParse;
  }

  LOGDEBUG("ReturnParser:\t[%d]", retParse);
  LOGDEBUG("Arg daemon\t\t[%s]", args.daemon ? "Enable" : "Disable");
  LOGDEBUG("Arg accel scale\t[%d] G", args.accel_scale);
  LOGDEBUG("Arg gyro scale\t[%d] °/s", args.gyro_scale);
  LOGDEBUG("Arg alpha constant\t[%0.5f]", args.const_alpha);
  LOGDEBUG("Arg sample rate\t[%d] ms", args.sample_rate);

  /**
   * CONFIGURE LOGGER
   */
  auto logType = args.daemon ? logger::LogType::SYSLOG : logger::LogType::STD_OUT_STREAM;
  LOG_INSTANCE.Init(logType);

  /**
   * PARSE IMU ABSTRACTION ARGS
   */
  IMUAbstraction::eAccelScale accel_scale;
  IMUAbstraction::eGyroScale gyro_scale;
  IMUAbstraction::eSampleFreq sample_rate;
  auto retArgs = ArgValidator::ConvertArgs(args, accel_scale, gyro_scale, sample_rate);
  if (retArgs != 0)
  {
    LOGERROR("validate_args failed [%d]", retArgs);
    return retArgs;
  }

  /**
   * RUN AS DAEMON?
   */
  if (args.daemon)
  {
    pid_t pid = daemonize();
    if (pid > 0)
    {
      init_signal_handler();
    }
  }
  else
  {
    g_run = 0;
  }

  /**
   * IS IMU DEVICE AVAILABLE?
   */
  auto imu_avail = false;
  auto DeviceIndex = 0;
  do
  {
    std::stringstream sPath;
    sPath << DEVICE_PATH;
    sPath << "iio:device" << DeviceIndex << "/name";
    if (!path_isvalid(sPath.str().c_str()))
    {
      break;
    }

    auto sNameDev = GetValueInFile<std::string>(sPath.str().c_str());
    LOGDEBUG("iio device [%s] - name [%s]", sPath.str().c_str(), sNameDev.c_str());
    imu_avail = (sNameDev.compare("mpu6050") == 0);
    if (!imu_avail)
    {
      DeviceIndex++;
    }
  } while (!imu_avail);

  /**
   * START SERVER
   */
  std::shared_ptr<RPCServer::IRPCServer> serverRPC;
  std::shared_ptr<IMUServer::IIMUServer> serverIMU;
  std::shared_ptr<IMUAbstraction::IIMUAbstraction> imuAbstraction;
  std::shared_ptr<IMUAngle::IIMUAngle> imuAngle;
  std::shared_ptr<IMUMath::IIMUMath> imuMath;
  std::shared_ptr<IMUAbstraction::IValueGenerator> valueGen;

  if (imu_avail)
  {
#ifdef IMU_BUFFER_IIO
    imuAbstraction =
      std::make_shared<IMUAbstraction::IMUBufferIIO>(
        DEVICE_PATH,
        DeviceIndex,
        accel_scale,
        gyro_scale,
        sample_rate
      );
    LOGDEBUG("Using IMU Industrial IO Buffering");
#else
    imuAbstraction =
      std::make_shared<IMUAbstraction::IMUIndustrialIO>(
        DEVICE_PATH,
        DeviceIndex,
        accel_scale,
        gyro_scale,
        sample_rate
      );
    LOGDEBUG("Using IMUIndustrialIO");
#endif
  }
  else
  {
#ifdef SINWAVE_GENERATOR
    valueGen = std::make_shared<IMUAbstraction::SinWaveValueGenerator>(FREQ_SIN, AXIS_PHASE_SHIFT);
    LOGDEBUG("Using SinWaveValueGenerator SinFrequency[%0.2f Hz] - PhaseShiftAxis[%0.1f°]", FREQ_SIN, AXIS_PHASE_SHIFT);
#else
    valueGen = std::make_shared<IMUAbstraction::RandomValueGenerator>();
    LOGDEBUG("Using RandomValueGenerator");
#endif
    imuAbstraction = std::make_shared<IMUAbstraction::IMUStub>(
      valueGen,
      accel_scale,
      gyro_scale,
      sample_rate
    );
    LOGDEBUG("Using IMUStub");
  }
  imuAngle = std::make_shared<IMUAngle::IMUEulerAngle>(imuAbstraction);
  imuMath = std::make_shared<IMUMath::IMUMathImpl>(imuAbstraction, imuAngle, args.const_alpha);
  serverRPC = std::make_shared<RPCServer::DBusCxxServer>();
  serverIMU = std::make_shared<IMUServer::IMURPCServer>(serverRPC, imuAbstraction, imuAngle, imuMath);

  auto ret = serverIMU->StartServer();
  LOGDEBUG("Server Started [%d]", (int)ret);

  /**
   * NORMAL OPERATION
   */
  if (ret == IMUServer::eIMUServerError::eRET_OK)
  {
    if (args.daemon)
    {
      while (g_run != 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    else
    {
      std::cin.get();
    }
  }
  else
  {
    LOGERROR("Server Start Failed [%d]", (int)ret);
  }

  LOGDEBUG("Closing server...");
  return (int)ret;
}
