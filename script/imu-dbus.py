import sys
import time
import argparse
import logging
import coloredlogs
import queue

import sockethandler
import imuplot

# Initialize Logger
__Logger = logging.getLogger('imu-dbus')

# Version variable:
__version__ = "v0.0.0"

# Arguments parser:
parser = argparse.ArgumentParser(description='Script to interact with imu-dbus server')
parser.add_argument("--Address",
                    "-a",
                    dest="Address",
                    help="IMU IP Address",
                    metavar="IP",
                    required=True)
parser.add_argument("--Port",
                    "-p",
                    dest="Port",
                    help="TCP IP Port",
                    metavar="PORT",
                    required=True)
parser.add_argument("--SampleTime",
                    "-t",
                    dest="SampleTime",
                    help="IMU Sample time in seconds",
                    metavar="SAMPLETIME")
parser.add_argument('--MathPlot',
                    '-m',
                    dest="MathPlot",
                    help="Enable Math Plotting",
                    action='store_true',
                    default=False)
parser.add_argument('--Debug',
                    '-d',
                    dest="Debug",
                    help="Enable Debug Log",
                    action='store_true',
                    default=False)
parser.add_argument('--version',
                    '-v',
                    action='version',
                    version=__version__)


imuQueue = queue.Queue()
def cbSockRecv(data):
  sData = data.decode("utf-8")
  __Logger.debug(sData)

  sData = sData.split(";")
  d = {}
  for axis in sData[0:3]:
    AxisName =  axis.split(",")[0]
    AxisAccel = float(axis.split(",")[1])
    AxisGyro =  float(axis.split(",")[2])
    AxisAngle = float(axis.split(",")[3])
    d[AxisName] = {
      "Accel" : AxisAccel,
      "Gyro" : AxisGyro,
      "Angle" : AxisAngle,
    }

  AccelObj = imuplot.Accel(d["X"]["Accel"], d["Y"]["Accel"], d["Z"]["Accel"])
  GyroObj =  imuplot.Gyro(d["X"]["Gyro"], d["Y"]["Gyro"], d["Z"]["Gyro"])
  AngleObj = imuplot.AngleEuler(d["X"]["Angle"], d["Y"]["Angle"], d["Z"]["Angle"])
  imu = imuplot.ImuData(accel={}, gyro={}, angle={})
  now = time.time()
  imu.addAccel(now, AccelObj)
  imu.addGyro(now, GyroObj)
  imu.addAngle(now, AngleObj)

  imuQueue.put(imu)



def main():
  # Default script runtime
  __defaultTime = 5

  # Parse command line arguments
  args = parser.parse_args()
  # Store variables
  ipAddress = args.Address
  ipPort = int(args.Port)
  sampleTime = float(args.SampleTime if args.SampleTime is not None else __defaultTime)
  mathPlot = args.MathPlot
  debug = args.Debug

  LoggerLevel = 'DEBUG' if debug is True else 'INFO'
  coloredlogs.install(level=LoggerLevel,logger=__Logger)

  # Initialize socket handler module
  sock = sockethandler.SocketHandlerClient()
  sock.Init(ip=ipAddress, port=ipPort)

  # Initialize imuplot module
  myplot = imuplot.ImuDataPlot()
  if mathPlot:
    myplot.Init()

  start_time = time.time()
  sock.AddCallbackRecv(cb=cbSockRecv)
  while ((time.time() - start_time) <= sampleTime) or (imuQueue.empty() is not True):
    try:
      # Get data from queue
      imu = imuQueue.get(timeout=sampleTime/1000)
      myplot.appendImuData(imu)
      if imuQueue.empty():
        myplot.showGraph("IMU Data")
    except queue.Empty:
      # No data available, keep waiting
      pass
    # Stop socket connection to avoid data flood
    if ((time.time() - start_time) > sampleTime):
      sock.DeInit()

  sock.DeInit()

  __Logger.info("Execution time {:.2f} seconds".format(time.time() - start_time))

  myplot.plotShow()


if __name__ == "__main__":
  main()
