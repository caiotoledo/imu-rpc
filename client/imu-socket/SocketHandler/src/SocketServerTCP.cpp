#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <LogInstance.h>

#include <SocketServerTCP.hpp>

using namespace SocketServer;

constexpr auto QUEUE_LISTEN = 20;

SocketServerTCP::SocketServerTCP(int port) :
  bThreadServer(false),
  serverHandler(-1),
  port(port)
{
}

int SocketServerTCP::Init(void)
{
  auto ret = 0;

  serverHandler = socket(AF_INET, SOCK_STREAM, 0);
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

  ret = listen(serverHandler, QUEUE_LISTEN);
  if(ret != 0)
  {
    LOGERROR("listen failed [%d][%s]", ret, strerror(errno));
    return -1;
  }

  if (!thServer.joinable())
  {
    auto funcServer = [this]()
    {
      while (this->bThreadServer)
      {
        this->AcceptConnection();
      }
    };

    this->bThreadServer = true;
    thServer = std::thread(funcServer);
    if (!thServer.joinable())
    {
      /* Thread not initialized */
      ret = -1;
    }
  }

  return ret;
}

void SocketServerTCP::AcceptConnection(void)
{
  struct sockaddr_in client_addr;
  socklen_t length = sizeof(client_addr);

  int conn = accept(serverHandler, (struct sockaddr*)&client_addr, &length);
  if( conn >= 0 )
  {
    std::lock_guard<std::mutex> lck(mtxMapConnClient);
    auto connClient = std::make_shared<ConnectionClient>(conn);
    mapConnClient[conn] = connClient;
  }
}

int SocketServerTCP::SendToClients(const std::vector<uint8_t> &vec)
{
  std::lock_guard<std::mutex> lck(mtxMapConnClient);
  int ret = 0;
  std::vector<int> disconnectedClients;

  for (auto &val : mapConnClient)
  {
    auto connHandler = val.first;
    auto client = val.second;

    ret = client->Send(vec);
    if (ret != 0)
    {
      LOGWARN("Client [%d] disconnected", connHandler);
      disconnectedClients.push_back(connHandler);
      continue;
    }
  }

  for (auto &handler : disconnectedClients)
  {
    mapConnClient.erase(handler);
  }

  return ret;
}

void SocketServerTCP::CloseServer(void)
{
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

void SocketServerTCP::DeInit(void)
{
  /* Close Server */
  this->CloseServer();

  /* Close all client connections */
  mapConnClient.clear();

  /* Finish ConnectionAccpet Thread */
  if (thServer.joinable())
  {
    bThreadServer = false;
    thServer.join();
  }
}

SocketServerTCP::~SocketServerTCP()
{
  this->DeInit();
}
