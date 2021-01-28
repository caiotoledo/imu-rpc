#pragma once

#include <vector>

namespace SocketServer
{

  class ISocketServer
  {
  public:
    ISocketServer() = default;

    /**
     * @brief Initialize Socket Server
     *
     * @return int Returns 0 on success, otherwise -1
     */
    virtual int Init(void) = 0;

    /**
     * @brief Send vector of bytes to all clients
     *
     * @param vec Byte vector
     * @return int Returns 0 on success
     */
    virtual int SendToClients(const std::vector<uint8_t> &vec) = 0;

    /**
     * @brief DeInitialize Socket Server
     */
    virtual void DeInit(void) = 0;

    ~ISocketServer() = default;
  };

} // namespace SocketServer

