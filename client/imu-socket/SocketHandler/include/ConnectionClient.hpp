#pragma once

#include <atomic>
#include <vector>
#include <thread>
#include <functional>

namespace SocketServer
{

  class ConnectionClient
  {
  private:
    std::thread thConnClient;
    std::atomic<bool> bThreadConnClient;

    int ConnHandler;

    std::vector<std::function<void(const std::vector<uint8_t> &)>> vecCbRecv;

    void CloseConnection(void);

  public:
    ConnectionClient(int handler);

    /**
     * @brief Send data to the client
     *
     * @param vec Byte vector to be send
     * @return int Returns 0 on success
     */
    virtual int Send(const std::vector<uint8_t> &vec);

    /**
     * @brief Add Callback notification when a new data is received
     *
     * @param cb Callback
     */
    virtual void AddCallbackRecv(std::function<void(const std::vector<uint8_t> &)> &&cb);

    virtual bool isConnected(void);

    ~ConnectionClient();
  };

} // namespace SocketServer
