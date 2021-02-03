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
__version__ = "v0.1.0"

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
  timepoint = float(sData[0])/1000
  for axis in sData[1:4]:
    AxisName =  axis.split(",")[0]
    AxisAccel = float(axis.split(",")[1])
    AxisGyro =  float(axis.split(",")[2])
    AxisAngle = float(axis.split(",")[3])
    AxisComplAngle = float(axis.split(",")[4])
    d[AxisName] = {
      "Accel" : AxisAccel,
      "Gyro" : AxisGyro,
      "Angle" : AxisAngle,
      "ComplAngle" : AxisComplAngle,
    }

  AccelObj = imuplot.Accel(d["X"]["Accel"], d["Y"]["Accel"], d["Z"]["Accel"])
  GyroObj =  imuplot.Gyro(d["X"]["Gyro"], d["Y"]["Gyro"], d["Z"]["Gyro"])
  AngleObj = imuplot.AngleEuler(d["X"]["Angle"], d["Y"]["Angle"], d["Z"]["Angle"])
  ComplAngleObj = imuplot.ComplAngle(d["X"]["ComplAngle"], d["Y"]["ComplAngle"], d["Z"]["ComplAngle"])
  imu = imuplot.ImuData(accel={}, gyro={}, angle={}, angleComplFilter={})
  imu.addAccel(timepoint, AccelObj)
  imu.addGyro(timepoint, GyroObj)
  imu.addAngle(timepoint, AngleObj)
  imu.addComplFilterAngle(timepoint, ComplAngleObj)

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
  retSock = sock.Init(ip=ipAddress, port=ipPort)
  if retSock != 0:
    __Logger.critical("Connection Failure with [{}:{}]".format(ipAddress, ipPort))
    sys.exit(retSock)

  # Initialize imuplot module
  myplot = imuplot.ImuDataPlot()
  if mathPlot:
    myplot.Init()

  start_time = time.time()
  sock.AddCallbackRecv(cb=cbSockRecv)
  while ((time.time() - start_time) <= sampleTime) or (imuQueue.empty() is not True):
    # Stop socket connection to avoid data flood
    if ((time.time() - start_time) > sampleTime):
      sock.DeInit()
    try:
      # Get data from queue
      imu = imuQueue.get(timeout=sampleTime/1000)
      myplot.appendImuData(imu)
      if imuQueue.empty():
        myplot.showGraph("IMU Data")
    except queue.Empty:
      # No data available, keep waiting
      pass

  sock.DeInit()

  __Logger.info("Execution time {:.2f} seconds".format(time.time() - start_time))

  myplot.plotShow()


if __name__ == "__main__":
  main()
