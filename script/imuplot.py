import matplotlib.pyplot as plt

class Generic3DPoint:
  def __init__(self, x, y, z):
    self.X = x
    self.Y = y
    self.Z = z

  def __str__(self):
    return 'X = {}; Y = {}; Z = {};'.format(self.X,self.Y,self.Z)


class Accel(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)


class Gyro(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)


class AngleEuler(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)

class ComplAngle(Generic3DPoint):
  def __init__(self, x, y, z):
    super().__init__(x,y,z)

class ImuData:
  def __init__(self, accel, gyro, angle, angleComplFilter):
    self.ArrAccel = accel
    self.ArrGyro = gyro
    self.ArrAngle = angle
    self.ArrComplFilterAngle = angleComplFilter

  def addAccel(self, t, accel):
    self.ArrAccel[t] = accel

  def addGyro(self, t, gyro):
    self.ArrGyro[t] = gyro

  def addAngle(self, t, angle):
    self.ArrAngle[t] = angle

  def addComplFilterAngle(self, t, angle):
    self.ArrComplFilterAngle[t] = angle

  def __add__(self, other):
    accel = {**self.ArrAccel, **other.ArrAccel}
    gyro = {**self.ArrGyro, **other.ArrGyro}
    angle = {**self.ArrAngle, **other.ArrAngle}
    angleComplFilter = {**self.ArrComplFilterAngle, **other.ArrComplFilterAngle}
    return ImuData(accel, gyro, angle, angleComplFilter)


class ImuDataPlot():
  def __init__(self, imudata=ImuData(accel={}, gyro={}, angle={}, angleComplFilter={})):
    self.Data = imudata
    self.__isInitalized = False

  def Init(self):
    self.__fig, (self.__axAccel, self.__axGyro, self.__axAngle, self.__axComplAngle) = plt.subplots(4, 1)

    self.__axAccel.set_title('Accelerometer')
    self.__axAccel.set(ylabel='Accel (mg)', xlabel='Time (ms)')

    self.__axGyro.set_title('Gyroscope')
    self.__axGyro.set(ylabel='Gyro (°/s)', xlabel='Time (ms)')

    self.__axAngle.set_title('Euler Angle')
    self.__axAngle.set(ylabel='Angle (°)', xlabel='Time (ms)')

    self.__axComplAngle.set_title('Angle Complementary Filter')
    self.__axComplAngle.set(ylabel='Angle (°)', xlabel='Time (ms)')

    self.__isInitalized = True

  def setImuData(self, imudata):
    self.Data = imudata

  def appendImuData(self, imudata):
    self.Data = self.Data + imudata

  def __getArrDataPlot(self):
    def __parseData(Arr3DPointIn):
      dictOut = {'T':[], 'X':[], 'Y':[], 'Z':[]}
      for key,value in Arr3DPointIn.items():
        dictOut['T'].append(key)
        dictOut['X'].append(value.X)
        dictOut['Y'].append(value.Y)
        dictOut['Z'].append(value.Z)
      return dictOut

    imuAccel = __parseData(self.Data.ArrAccel)
    imuGyro =  __parseData(self.Data.ArrGyro)
    imuAngle = __parseData(self.Data.ArrAngle)
    imuComplAngle = __parseData(self.Data.ArrComplFilterAngle)

    return imuAccel, imuGyro, imuAngle, imuComplAngle

  def __plotGraph(self, ax, plotData):
    # Normalize time data (Always start from 0 seconds)
    timeDataArr = [timeData - plotData['T'][0] for timeData in plotData['T']]
    # Plot IMU data
    ax.plot(timeDataArr, plotData['X'], 'r.')
    ax.plot(timeDataArr, plotData['Y'], 'g.')
    ax.plot(timeDataArr, plotData['Z'], 'b.')
    ax.legend(['X','Y','Z'])
    ax.grid(b=True)

  def showGraph(self, title='', pause_time=0.001):
    if self.__isInitalized:
      self.__fig.suptitle(title)
      plotAccel, plotGyro, plotAngle, plotComplAngle = self.__getArrDataPlot()

      self.__plotGraph(self.__axAccel, plotAccel)
      self.__plotGraph(self.__axGyro, plotGyro)
      self.__plotGraph(self.__axAngle, plotAngle)
      self.__plotGraph(self.__axComplAngle, plotComplAngle)

      plt.draw()
      plt.pause(pause_time)

  def plotShow(self):
    plt.show()
