#include <iostream>
#include <map>
#include <iomanip>
#include <thread>
#include <memory>
#include <chrono>

#include <LogInstance.hpp>

#include <DBusIMUClient.hpp>

#include "argparser.hpp"

constexpr auto COUNT_ALIGN = 18;
constexpr int NUM_AXIS = 3;

static IMUClient::eIMUError PrintRawValues(std::shared_ptr<IMUClient::IIMUClient> &client, bool bAccel, bool bGyro, bool bAngle, bool bComplAngle)
{
  auto ret = IMUClient::eIMUError::eRET_OK;

  for (size_t i = 0; i < NUM_AXIS; i++)
  {
    const std::map<int, std::string> mapAxis = { {0, "X"}, {1, "Y"}, {2, "Z"}, };
    double valAccel = 0;
    double valGyro = 0;
    double valAngle = 0;
    double valComplAngle = 0;

    /* Sample data from client */
    if (bAccel)
    {
      ret = client->GetRawAccel((DBusTypes::eAxis)i, valAccel);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }
    if (bGyro)
    {
      ret = client->GetRawGyro((DBusTypes::eAxis)i, valGyro);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }
    if (bAngle)
    {
      ret = client->GetEulerAngle((DBusTypes::eAxis)i, DBusTypes::eAngleUnit::eDegrees, valAngle);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }
    if (bComplAngle)
    {
      ret = client->GetComplFilterAngle((DBusTypes::eAxis)i, DBusTypes::eAngleUnit::eDegrees, valComplAngle);
      if (ret != IMUClient::eIMUError::eRET_OK)
      {
        break;
      }
    }

    /* Show Axis data */
    std::stringstream sAccel;
    sAccel << "Accel[" << std::fixed << std::setprecision(3) << valAccel << "]";
    std::stringstream sGyro;
    sGyro << "Gyro[" << std::fixed << std::setprecision(3) << valGyro << "]";
    std::stringstream sAngle;
    sAngle << "Angle[" << std::fixed << std::setprecision(3) << valAngle << "]";
    std::stringstream sComplAngle;
    sComplAngle << "ComplAngle[" << std::fixed << std::setprecision(3) << valComplAngle << "]";

    std::stringstream sOut;
    if (bAccel)
    {
      sOut << std::left << std::setw(COUNT_ALIGN) << sAccel.str();
    }
    if (bGyro)
    {
      sOut << std::left << std::setw(COUNT_ALIGN) << sGyro.str();
    }
    if (bAngle)
    {
      sOut << std::left << std::setw(COUNT_ALIGN) << sAngle.str();
    }
    if (bComplAngle)
    {
      sOut << std::left << std::setw(COUNT_ALIGN) << sComplAngle.str();
    }

    std::cout << "[" << mapAxis.at(i).c_str() << "] "
              << sOut.str()
              << std::endl;
  }
  printf("\n");

  return ret;
}

int main(int argc, char const *argv[])
{
  /**
   * PARSE ARGUMENTS
   */
  ArgParser::arguments args;
  auto retParse = ArgParser::iProcessArgs(argc, argv, args);

  LOGDEBUG("ReturnParser:\t[%d]", retParse);
  LOGDEBUG("Arg accel\t\t[%s]", args.accel ? "Enable" : "Disable");
  LOGDEBUG("Arg gyro\t\t[%s]", args.gyro ? "Enable" : "Disable");
  LOGDEBUG("Arg euler\t\t[%s]", args.euler ? "Enable" : "Disable");
  LOGDEBUG("Arg compl angle\t[%s]", args.compl_filter_angle ? "Enable" : "Disable");
  LOGDEBUG("Arg timeout\t[%d] seconds", args.timeout);

  if (retParse != 0) {
    LOGERROR("Error ProcessArgs [%d]", retParse);
    return retParse;
  }

  /**
   * START CLIENT
   */
  std::shared_ptr<IMUClient::IIMUClient> client;

  client = std::make_shared<IMUClient::DBusIMUClient>();
  auto ret = client->Init();

  auto func = [&client, &args]()
  {
    auto ret = PrintRawValues(client, args.accel, args.gyro, args.euler, args.compl_filter_angle);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      LOGERROR("IMU Error [%d]", ((int)ret));
    }
  };
  client->AddUpdateDataCallback(func);

  if (ret == IMUClient::eIMUError::eRET_OK)
  {
    if (args.timeout > 0)
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
