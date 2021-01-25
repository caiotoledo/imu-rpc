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
  start_time = time.time()

  # Parse command line arguments
  args = parser.parse_args()
  # Store variables
  ipAddress = args.Address
  ipPort = int(args.Port)
  sampleTime = int(args.SampleTime if args.SampleTime is not None else sys.maxsize)
  mathPlot = args.MathPlot
  debug = args.Debug

  LoggerLevel = 'DEBUG' if debug is True else 'INFO'
  coloredlogs.install(level=LoggerLevel,logger=__Logger)

  sock = sockethandler.SocketHandlerClient(cbRecv=cbSockRecv)
  sock.Init(ip=ipAddress, port=ipPort)

  runTime = sampleTime if sampleTime is not sys.maxsize else 5
  if mathPlot:
    myplot = imuplot.ImuDataPlot()
  while ( (time.time() - start_time) < runTime ):
    try:
      # Get data from queue
      imu = imuQueue.get(timeout=runTime/1000)
      if mathPlot:
        myplot.appendImuData(imu)
        if imuQueue.empty():
          myplot.showGraph("IMU Data")
    except queue.Empty:
      # No data available, keep waiting
      pass

  sock.DeInit()

  __Logger.info("Execution time {:.2f} seconds".format(time.time() - start_time))

  if mathPlot:
    myplot.plotShow()


if __name__ == "__main__":
  main()
