#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

#include <iomanip>
#include <memory>

#include <LogInstance.h>
#include <DBusIMUClient.hpp>
#include <SocketServerImpl.hpp>

#include "argparser.hpp"

constexpr auto DELAY_CHILD_CREATION = std::chrono::milliseconds(1);
constexpr int NUM_AXIS = 3;
constexpr int IMU_PRECISION = 5;

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

static int ParseIMUData(std::shared_ptr<IMUClient::IIMUClient> &client, std::string &data)
{
  auto retParseIMUData = 0;
  std::stringstream sOut;

  for (size_t i = 0; i < NUM_AXIS; i++)
  {
    const std::map<int, std::string> mapAxis = { {0, "X"}, {1, "Y"}, {2, "Z"}, };
    double valAccel = 0;
    double valGyro = 0;
    double valAngle = 0;

    /* Sample data from client */
    auto ret = client->GetRawAccel((DBusTypes::eAxis)i, valAccel);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      retParseIMUData = -1;
      break;
    }
    ret = client->GetRawGyro((DBusTypes::eAxis)i, valGyro);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      retParseIMUData = -1;
      break;
    }
    ret = client->GetEulerAngle((DBusTypes::eAxis)i, DBusTypes::eAngleUnit::eDegrees, valAngle);
    if (ret != IMUClient::eIMUError::eRET_OK)
    {
      retParseIMUData = -1;
      break;
    }

    /* Show Axis data */
    std::stringstream sAccel;
    sAccel << std::fixed << std::setprecision(IMU_PRECISION) << valAccel;
    std::stringstream sGyro;
    sGyro << std::fixed << std::setprecision(IMU_PRECISION) << valGyro;
    std::stringstream sAngle;
    sAngle << std::fixed << std::setprecision(IMU_PRECISION) << valAngle;

    sOut << mapAxis.at(i).c_str() << ",";
    sOut << sAccel.str() << ",";
    sOut << sGyro.str() << ",";
    sOut << sAngle.str() << ";";
  }

  data.assign(sOut.str());

  return retParseIMUData;
}

int main(int argc, char const *argv[])
{
  /**
   * PARSE ARGUMENTS
   */
  ArgParser::arguments args;
  auto retParse = ArgParser::iProcessArgs(argc, argv, args);
  if (retParse != 0) {
    LOGERROR("Error ProcessArgs [%d]", retParse);
    return retParse;
  }

  LOGDEBUG("ReturnParser: %d", retParse);
  LOGDEBUG("Arg daemon\t[%s]", args.daemon ? "Enable" : "Disable");
  LOGDEBUG("Arg port\t[%d]", args.port);

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
   * START IMU CLIENT
   */
  std::shared_ptr<IMUClient::IIMUClient> client;
  client = std::make_shared<IMUClient::DBusIMUClient>();
  auto ret = client->Init();
  if (ret != IMUClient::eIMUError::eRET_OK)
  {
    LOGERROR("IMU Client Init Error [%d]", ((int)ret));
    return (int)ret;
  }

  /**
   * START SERVER TCP
   */
  std::shared_ptr<SocketServer::ISocketServer> server;
  server = std::make_shared<SocketServer::SocketServerImpl>(args.port);
  auto retServer = server->Init();
  if (retServer != 0)
  {
    return retServer;
  }

  /**
   * CONFIG IMU CLIENT CALLBACK
   */
  auto func = [&server, &client]()
  {
    /* Sample Data from IMU */
    std::string sData;
    ParseIMUData(client, sData);

    /* Send Data to Clients */
    std::vector<uint8_t> vData(sData.begin(), sData.end());
    server->SendToClients(vData);
  };
  client->AddUpdateDataCallback(func);

  /**
   * NORMAL OPERATION
   */
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

  LOGDEBUG("Closing socket...");

  return ((int)ret);
}
