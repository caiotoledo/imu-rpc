#include <iostream>
#include <thread>
#include <memory>
#include <limits>
#include <chrono>

#include <LogInstance.h>

#include <DBusIMUClient.hpp>

#include "argparser.hpp"

constexpr int NUM_AXIS = 3;

static IMUClient::eIMUError PrintRawValues(std::shared_ptr<IMUClient::IIMUClient> &client, bool bAccel, bool bGyro)
{
  auto ret = IMUClient::eIMUError::eRET_OK;
  for (size_t i = 0; i < NUM_AXIS; i++)
  {
    double valAccel;
    double valGyro;
    if (bAccel)
    {
      ret = client->GetRawAccel((IMUClient::eAxis)i, valAccel);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }
    if (bGyro)
    {
      ret = client->GetRawGyro((IMUClient::eAxis)i, valGyro);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }

    LOGDEBUG("[%d]\tAccel[%0.2f]\tGyro[%0.2f]", i, valAccel, valGyro);
  }

  return ret;
}

int main(int argc, char const *argv[])
{
  ArgParser::arguments args;
  auto retParse = ArgParser::iProcessArgs(argc, argv, args);

  LOGDEBUG("ReturnParser: %d", retParse);
  LOGDEBUG("Arg accel\t[%s]", args.accel ? "Enable" : "Disable");
  LOGDEBUG("Arg gyro\t[%s]", args.gyro ? "Enable" : "Disable");
  LOGDEBUG("Arg timeout\t[%d] seconds", args.timeout);

  if (retParse != 0) {
    LOGERROR("Error ProcessArgs [%d]", retParse);
    return retParse;
  }

  std::shared_ptr<IMUClient::IIMUClient> client;

  client = std::make_shared<IMUClient::DBusIMUClient>();
  auto ret = client->Init();

  auto func = [&client, &args]()
  {
    auto ret = PrintRawValues(client, args.accel, args.gyro);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      LOGERROR("IMU Error [%d]", ((int)ret));
    }
  };
  client->AddUpdateDataCallback(func);

  if (ret == IMUClient::eIMUError::eRET_OK)
  {
    if (args.timeout != std::numeric_limits<int>::max())
    {
      std::this_thread::sleep_for(std::chrono::seconds(args.timeout));
    }
    else
    {
      std::cin.get();
    }
  }
  else
  {
    LOGERROR("IMU Client Init Error [%d]", ((int)ret));
  }

  return ((int)ret);
}
