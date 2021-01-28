#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <LogInstance.h>

#include "ConnectionClient.hpp"

using namespace SocketServer;

ConnectionClient::ConnectionClient(int handler) :
  ConnHandler(handler)
{
  auto funcRecv = [this]()
  {
    int retval;
    fd_set rfds;
    struct timeval tv;
    while (bThreadConnClient)
    {
      /* Prepare fd_set for select */
      FD_ZERO(&rfds);
      FD_SET(0, &rfds);
      FD_SET(ConnHandler, &rfds);

      /*Setting timeout time*/
      tv.tv_sec = 0UL;
      tv.tv_usec = 50UL*1000UL;

      /* Waiting data */
      retval = select(ConnHandler+1, &rfds, NULL, NULL, &tv);
      if (retval > 0)
      {
        /* Received Data! */
        if(FD_ISSET(ConnHandler,&rfds))
        {
          /* Get queued bytes available for read */
          int bytes_avail;
          auto ret = ioctl(ConnHandler, FIONREAD, &bytes_avail);
          if (ret != 0)
          {
            break;
          }

          /* Read from socket */
          char buf[bytes_avail];
          auto bytes_read = recv(ConnHandler, &buf[0], bytes_avail, 0);
          if (bytes_read < 0)
          {
            LOGWARN("recv failed [%d][%s]", bytes_read, strerror(errno));
            break;
          }

          /* Notify via callback */
          std::vector<uint8_t> vecBytes(buf, buf+bytes_read);
          for (auto &func : vecCbRecv)
          {
            func(vecBytes);
          }
        }
      }

      if (!this->isConnected())
      {
        bThreadConnClient = false;
      }
    }
    this->CloseConnection();
  };
  bThreadConnClient = true;
  thConnClient = std::thread(funcRecv);
}

bool ConnectionClient::isConnected(void)
{
  int error_code;
  socklen_t error_code_size = sizeof(error_code);

  getsockopt(ConnHandler, SOL_SOCKET, SO_ERROR, &error_code, &error_code_size);

  return (error_code == 0);
}

int ConnectionClient::Send(const std::vector<uint8_t> &vec)
{
  auto ret = 0;

  if (!this->isConnected())
  {
    return -1;
  }

  auto retSend = send(ConnHandler, &vec[0], vec.size(), 0);
  if (retSend < 0)
  {
    LOGWARN("send failed [%d][%s]", ret, strerror(errno));
    ret = -1;
  }

  return ret;
}

void ConnectionClient::AddCallbackRecv(std::function<void(const std::vector<uint8_t> &)> &&cb)
{
  vecCbRecv.push_back(cb);
}

void ConnectionClient::CloseConnection(void)
{
  shutdown(ConnHandler, SHUT_RDWR);

  close(ConnHandler);
}

ConnectionClient::~ConnectionClient()
{
  if (thConnClient.joinable())
  {
    bThreadConnClient = false;
    thConnClient.join();
  }

  this->CloseConnection();
}
