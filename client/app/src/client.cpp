#include <iostream>
#include <memory>

#include <LogInstance.h>

#include <DBusIMUClient.hpp>

static IMUClient::eIMUError PrintAccel(std::shared_ptr<IMUClient::IIMUClient> &client)
{
  IMUClient::eIMUError ret;
  double val[3] = {0};
  for (size_t i = 0; i < (sizeof(val)/sizeof(val[0])); i++)
  {
    ret = client->GetRawAccel((IMUClient::eAxis)i, val[i]);
    if (ret == IMUClient::eIMUError::eRET_OK)
    {
      LOGDEBUG("Accel[%d] -> [%0.2f]", i, val[i]);
    }
    else
    {
      LOGERROR("IMU Client GetRawAccel Error [%d]", (int)ret);
      break;
    }

    ret = client->GetRawGyro((IMUClient::eAxis)i, val[i]);
    if (ret == IMUClient::eIMUError::eRET_OK)
    {
      LOGDEBUG("Gyro[%d] -> [%0.2f]", i, val[i]);
    }
    else
    {
      LOGERROR("IMU Client GetRawGyro Error [%d]", (int)ret);
      break;
    }
  }
  return ret;
}

int main(int argc, char const *argv[])
{
  std::shared_ptr<IMUClient::IIMUClient> client;

  client = std::make_shared<IMUClient::DBusIMUClient>();
  auto ret = client->Init();

  auto func = [&client]()
  {
    auto ret = PrintAccel(client);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      LOGERROR("IMU Error [%d]", ((int)ret));
    }
  };
  client->AddUpdateDataCallback(func);

  if (ret == IMUClient::eIMUError::eRET_OK)
  {
    std::cin.get();
  }
  else
  {
    LOGERROR("IMU Client Init Error [%d]", ((int)ret));
  }

  return ((int)ret);
}
