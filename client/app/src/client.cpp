#include <iostream>
#include <thread>
#include <memory>
#include <limits>
#include <chrono>

#include <LogInstance.h>

#include <DBusIMUClient.hpp>

#include "argparser.hpp"

static IMUClient::eIMUError PrintRawValues(std::shared_ptr<IMUClient::IIMUClient> &client, bool bAccel, bool bGyro)
{
  auto ret = IMUClient::eIMUError::eRET_OK;
  double val[3] = {0};
  for (size_t i = 0; i < (sizeof(val)/sizeof(val[0])); i++)
  {
    if (bAccel)
    {
      ret = client->GetRawAccel((IMUClient::eAxis)i, val[i]);
      if (ret == IMUClient::eIMUError::eRET_OK)
      {
        LOGDEBUG("Accel[%d] -> [%0.2f]", i, val[i]);
      }
    }

    if (bGyro)
    {
      ret = client->GetRawGyro((IMUClient::eAxis)i, val[i]);
      if (ret == IMUClient::eIMUError::eRET_OK)
      {
        LOGDEBUG("Gyro[%d] -> [%0.2f]", i, val[i]);
      }
    }

    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      break;
    }
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
