#pragma once

#include <atomic>
#include <vector>
#include <mutex>
#include <map>
#include <memory>
#include <thread>

#include "ISocketServer.hpp"

namespace SocketServer
{

  class SocketServerUDP : public ISocketServer
  {
  private:
    std::mutex mtxVecClientAddr;
    std::vector<struct sockaddr_in> vecClientAddr;

    std::thread thRecvDataUDP;
    std::atomic<bool> bThreadRecvDataUDP;

    int serverHandler;
    int port;

    /**
     * @brief Thread
     */
    void RecvDataUDP(void);

    /**
     * @brief Close server socket
     */
    void CloseServer(void);

  public:
    SocketServerUDP(int port);

    virtual int Init(void) override;
    virtual int SendToClients(const std::vector<uint8_t> &vec) override;
    virtual void DeInit(void) override;

    ~SocketServerUDP();
  };

} // namespace SocketServer

