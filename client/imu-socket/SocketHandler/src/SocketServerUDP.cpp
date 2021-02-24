#include <cstdlib>
#include <string>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <LogInstance.h>

#include <SocketServerUDP.hpp>

using namespace SocketServer;

SocketServerUDP::SocketServerUDP(int port) :
  bThreadRecvDataUDP(false),
  serverHandler(-1),
  port(port)
{
}

int SocketServerUDP::Init(void)
{
  auto ret = 0;

  serverHandler = socket(AF_INET, SOCK_DGRAM, 0);
  if (serverHandler < 0)
  {
    LOGERROR("socket failed [%d][%s]", serverHandler, strerror(errno));
    return -1;
  }

  struct sockaddr_in server_sockaddr;
  server_sockaddr.sin_family = AF_INET;
  server_sockaddr.sin_port = htons(this->port);
  server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  ret = bind(serverHandler, (struct sockaddr* ) &server_sockaddr, sizeof(server_sockaddr));
  if(ret != 0)
  {
    LOGERROR("bind failed [%d][%s]", ret, strerror(errno));
    return -1;
  }

  if (!thRecvDataUDP.joinable())
  {
    auto funcServer = [this]()
    {
      while (this->bThreadRecvDataUDP)
      {
        this->RecvDataUDP();
      }
    };

    this->bThreadRecvDataUDP = true;
    thRecvDataUDP = std::thread(funcServer);
    if (!thRecvDataUDP.joinable())
    {
      /* Thread not initialized */
      ret = -1;
    }
  }

  return ret;
}

void SocketServerUDP::RecvDataUDP(void)
{
  uint8_t buf[128];
  struct sockaddr_in cliaddr;
  socklen_t len = sizeof(cliaddr);

  memset(&cliaddr, 0, sizeof(cliaddr));

  if (serverHandler < 0)
  {
    return;
  }

  auto n = recvfrom(serverHandler,
                    buf, sizeof(buf),
                    MSG_WAITALL,
                    (struct sockaddr *) &cliaddr,
                    &len);
  if (n >= 0)
  {
    std::lock_guard<std::mutex> lck(mtxVecClientAddr);
    vecClientAddr.push_back(cliaddr);
  }
}

int SocketServerUDP::SendToClients(const std::vector<uint8_t> &vec)
{
  int ret = 0;

  std::lock_guard<std::mutex> lck(mtxVecClientAddr);

  if (serverHandler < 0)
  {
    return -1;
  }

  for (auto &cliaddr : vecClientAddr)
  {
    auto len = sizeof(cliaddr);
    ret = sendto( serverHandler,
                  &vec[0], vec.size(),
                  MSG_CONFIRM,
                  (const struct sockaddr *) &cliaddr, len);
    if (ret < 0)
    {
      LOGWARN("send failed [%d][%s]", ret, strerror(errno));
    }
  }

  return ret;
}

void SocketServerUDP::CloseServer(void)
{
  /* Check valid socket */
  if (serverHandler == -1)
  {
    return;
  }

  std::lock_guard<std::mutex> lck(mtxVecClientAddr);

  /* Stop all data transfers */
  shutdown(serverHandler, SHUT_RDWR);

  /* Release Address */
  int val = 1;
  auto ret = setsockopt(serverHandler, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));
  if (ret != 0)
  {
    LOGERROR("setsockopt failed [%d][%s]", ret, strerror(errno));
  }

  /* Close socket */
  ret = close(serverHandler);
  if (ret != 0)
  {
    LOGERROR("close failed [%d][%s]", ret, strerror(errno));
  }

  /* Reset Socket */
  serverHandler = -1;
}

void SocketServerUDP::DeInit(void)
{
  this->CloseServer();

  /* Clear all client address */
  vecClientAddr.clear();

  /* Finish RecvDataUDP Thread */
  if (thRecvDataUDP.joinable())
  {
    bThreadRecvDataUDP = false;
    thRecvDataUDP.join();
  }
}

SocketServerUDP::~SocketServerUDP()
{
  this->DeInit();
}
