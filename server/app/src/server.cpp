#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <memory>

#include <LogInstance.h>

#include <DBusCxxServer.hpp>
#include <IMURPCServer.hpp>
#include <IMUIndustrialIO.hpp>

#include <IMUStub.hpp>
#include <IMUIndustrialIO.hpp>

#define DEVICE_PATH   "/sys/bus/iio/devices/"

static int file_isvalid(const char *path) {
  struct stat st;

  if (stat(path, &st) < 0)
    return -1;

  return S_ISREG(st.st_mode);
}

int main(int argc, char const *argv[])
{
  std::shared_ptr<RPCServer::IRPCServer> serverRPC;
  std::shared_ptr<IMUServer::IIMUServer> serverIMU;
  std::shared_ptr<IMUAbstraction::IIMUAbstraction> imuAbstraction;

  if (file_isvalid(DEVICE_PATH) == 1)
  {
    imuAbstraction = std::make_shared<IMUAbstraction::IMUIndustrialIO>();
    LOGDEBUG("Using IMUIndustrialIO");
  }
  else
  {
    imuAbstraction = std::make_shared<IMUAbstraction::IMUStub>();
    LOGDEBUG("Using IMUStub");
  }
  serverRPC = std::make_shared<RPCServer::DBusCxxServer>();
  serverIMU = std::make_shared<IMUServer::IMURPCServer>(serverRPC, imuAbstraction);

  auto ret = serverIMU->StartServer();
  LOGDEBUG("Server Started [%d]", (int)ret);

  if (ret == IMUServer::eIMUServerError::eRET_OK)
  {
    std::cin.get();
  }
  else
  {
    LOGERROR("Server Start Failed [%d]", (int)ret);
  }

  return (int)ret;
}
