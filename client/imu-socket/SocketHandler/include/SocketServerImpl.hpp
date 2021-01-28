#pragma once

#include <atomic>
#include <vector>
#include <mutex>
#include <map>
#include <memory>
#include <thread>

#include "ConnectionClient.hpp"

#include "ISocketServer.hpp"

namespace SocketServer
{

  class SocketServerImpl : public ISocketServer
  {
  private:
    std::mutex mtxMapConnClient;
    std::map<int, std::shared_ptr<ConnectionClient>> mapConnClient;

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
    SocketServerImpl(int port);

    virtual int Init(void) override;
    virtual int SendToClients(const std::vector<uint8_t> &vec) override;
    virtual void DeInit(void) override;

    ~SocketServerImpl();
  };

} // namespace SocketServer

