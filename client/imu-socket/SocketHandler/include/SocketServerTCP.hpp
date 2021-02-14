#pragma once

#include <atomic>
#include <vector>
#include <mutex>
#include <map>
#include <memory>
#include <thread>

#include "ConnectionTCPClient.hpp"

#include "ISocketServer.hpp"

namespace SocketServer
{

  class SocketServerTCP : public ISocketServer
  {
  private:
    std::mutex mtxMapConnClient;
    std::map<int, std::shared_ptr<ConnectionTCPClient>> mapConnClient;

    std::thread thServer;
    std::atomic<bool> bThreadServer;

    int serverHandler;
    int port;

    /**
     * @brief Accept client connections, used in thread loop
     */
    void AcceptConnection(void);

    /**
     * @brief Close server socket and connections
     */
    void CloseServer(void);

  public:
    SocketServerTCP(int port);

    virtual int Init(void) override;
    virtual int SendToClients(const std::vector<uint8_t> &vec) override;
    virtual void DeInit(void) override;

    ~SocketServerTCP();
  };

} // namespace SocketServer

