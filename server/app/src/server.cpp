#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <csignal>
#include <iostream>
#include <memory>
#include <atomic>

#include <LogInstance.h>

#include <DBusCxxServer.hpp>
#include <IMURPCServer.hpp>
#include <IMUIndustrialIO.hpp>

#include <IMUStub.hpp>
#include <IMUIndustrialIO.hpp>

#include "argparser.hpp"

#define DEVICE_PATH   "/sys/bus/iio/devices/"

constexpr auto DELAY_CHILD_CREATION = std::chrono::milliseconds(1);

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
    std::cout << "Daemon creation failed [" << strerror(errno) << "]" << std::endl;
    exit(pid);
  }
  else if (pid > 0)
  {
    std::cout << "Daemon created with PID " << pid << std::endl;
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

static int file_isvalid(const char *path) {
  struct stat st;

  if (stat(path, &st) < 0)
    return -1;

  return S_ISREG(st.st_mode);
}

int main(int argc, char const *argv[])
{
  /**
   * PARSE ARGUMENTS
   */
  ArgParser::arguments args;
  auto retParse = ArgParser::iProcessArgs(argc, argv, args);

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
    g_run = false;
  }

  /**
   * START SERVER
   */
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

  /**
   * NORMAL OPERATION
   */
  if (ret == IMUServer::eIMUServerError::eRET_OK)
  {
    if (args.daemon)
    {
      while (g_run)
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
