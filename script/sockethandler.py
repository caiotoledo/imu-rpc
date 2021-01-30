import logging
import coloredlogs
import multiprocessing
import threading
from concurrent.futures import ThreadPoolExecutor
import socket

class SocketHandlerClient():

  def __init__(self, cbRecv=None):
    # Initialize logging
    self.__Logger = logging.getLogger('SocketHandlerClient')
    coloredlogs.install(level='INFO',logger=self.__Logger)
    # Initialize ThreadPoolExecutor
    cpucount = multiprocessing.cpu_count()
    self.__Logger.debug('Max Workers: {}'.format(cpucount))
    self.__executor = ThreadPoolExecutor(max_workers=cpucount)
    self.__futRecvThread = None
    # Initialize socket
    self.__socket = None
    # Receive Callback Initialization
    self.__cbRecv = []
    if cbRecv is not None: self.__cbRecv.append(cbRecv)

  def Init(self, ip, port):
    ret = 0
    if self.__socket is None:
      try:
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM | socket.SOCK_NONBLOCK)
        self.__socket.settimeout(5)
        self.__socket.connect((ip, port))
        self.__socket.settimeout(0)
        self.__InitRecvThread()
      except OSError as msg:
        self.__Logger.critical(msg)
        self.__socket.close()
        self.__socket = None
        ret = -1
    return ret

  def AddCallbackRecv(self, cb):
    self.__cbRecv.append(cb)

  def DeInit(self):
    self.__DeInitRecvThread()
    if self.__socket is not None:
      self.__socket.close()
      self.__socket = None

  def __DeInitRecvThread(self):
    self.__bRecvThread = False
    if self.__futRecvThread is not None: self.__futRecvThread.result()

  def __InitRecvThread(self):
    self.__bRecvThread = True
    self.__futRecvThread = self.__executor.submit(self.__RecvThread)


  def __RecvThread(self):
    while self.__bRecvThread:
      try:
        data = self.__socket.recv(1024)
        for cb in self.__cbRecv: cb(data)
      except:
          pass
