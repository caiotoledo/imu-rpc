#pragma once

#include <atomic>
#include <vector>
#include <thread>
#include <functional>

namespace SocketServer
{

  class ConnectionTCPClient
  {
  private:
    std::thread thConnClient;
    std::atomic<bool> bThreadConnClient;

    int ConnHandler;

    std::vector<std::function<void(const std::vector<uint8_t> &)>> vecCbRecv;

    /**
     * @brief Close socket connection
     */
    void CloseConnection(void);

  public:
    ConnectionTCPClient(int handler);

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

    /**
     * @brief Check if client is connected
     *
     * @return true Connected
     * @return false Not Connected
     */
    virtual bool isConnected(void);

    ~ConnectionTCPClient();
  };

} // namespace SocketServer
