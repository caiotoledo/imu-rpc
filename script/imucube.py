from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math as m

import numpy as np

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def Rotation(X=0,Y=0,Z=0):
  return Rz(Z)*Ry(Y)*Rx(X)

def MatrixRotation(Matrix=np.array([]),X=0,Y=0,Z=0,decimals=4):
  Rot = Rz(Z)*Ry(Y)*Rx(X)
  return np.round(np.dot(Rot,Matrix), decimals=decimals)

def MatrixTranslation(Matrix=np.array([]),X=0,Y=0,Z=0):
  npVector = np.array([[X],[Y],[Z]])
  return Matrix-npVector


class LineCubePlot():
  def __init__(self, delta=1):
    self.__delta = delta
    self.__CubeFaces = self.__generateCubeMatrix(delta=self.__delta)
    self.__CubeFacesRotate = []
    self.__isInitalized = False

  def Init(self):
    self.__fig = plt.figure()
    self.__ax = Axes3D(self.__fig)
    self.__ax.set_xlim3d(-self.__delta, self.__delta)
    self.__ax.set_ylim3d(-self.__delta, self.__delta)
    self.__ax.set_zlim3d(-self.__delta, self.__delta)
    self.__isInitalized = True

  def Draw(self, pause_time=0.001):
    if self.__isInitalized:
      kwargs = [
        {'alpha': 1, 'color': 'black'},
        {'alpha': 1, 'color': 'black'},
        {'alpha': 1, 'color': 'black'},
        {'alpha': 1, 'color': 'black'},
        {'alpha': 1, 'color': 'magenta'},
      ]
      self.__ax.clear()
      for faceMatrix, args in zip(self.__CubeFacesRotate, kwargs):
        self.__ax.plot3D(faceMatrix[0], faceMatrix[1], faceMatrix[2], **args)
      plt.title('My Cube')
      plt.draw()
      plt.pause(pause_time)

  def Rotate(self,X=0,Y=0,Z=0):
    self.__CubeFacesRotate[:] = [MatrixRotation(Matrix=face,X=X,Y=Y,Z=Z) for face in self.__CubeFaces]

  def __generateCubeMatrix(self, delta):
    # Vector points of a square
    v0 = [0,0,0]
    v1 = [0,1,0]
    v2 = [0,1,-1]
    v3 = [0,0,-1]
    # Crete np array of the square
    npSquareMatrixPath = np.array([v0,v1,v2,v3,v0]).transpose()*delta
    # Translate square
    npVector = np.array([[-0.5],[0.5],[-0.5]])*delta
    npSquareMatrixPath = npSquareMatrixPath-npVector

    # Create Cube Square Faces
    SquareFaces = []
    SquareFaces.append(npSquareMatrixPath)
    SquareFaces.append(MatrixRotation(Matrix=npSquareMatrixPath,Y=m.pi/2))
    SquareFaces.append(MatrixRotation(Matrix=npSquareMatrixPath,Z=m.pi/2))
    SquareFaces.append(MatrixRotation(Matrix=npSquareMatrixPath,Y=-m.pi/2))
    SquareFaces.append(MatrixTranslation(Matrix=npSquareMatrixPath,X=(1*delta)))

    return SquareFaces


def main():
  arr1 = [0,0,0]
  arr2 = [1,1,1]
  for val1,val2,index in zip(arr1,arr2,range(0,3)):
    print(val1)
    print(val2)
    print(index)
    print("OPA")
    arr1[index] = val2
  print(arr1)
  obj = CubePlot(delta=100)
  obj.Draw(pause_time=1)
  obj.Rotate(X=m.pi/4)
  obj.Draw(pause_time=1)
  obj.Rotate(Y=m.pi/4)
  obj.Draw(pause_time=1)
  obj.Rotate(Z=m.pi/4)
  obj.Draw(pause_time=1)
  obj.Rotate(X=m.pi/4)
  obj.Draw(pause_time=1)
  obj.Rotate(Y=m.pi/4)
  obj.Draw(pause_time=1)
  obj.Rotate(Z=m.pi/4)
  obj.Draw(pause_time=1)
  plt.show()


if __name__ == "__main__":
  main()
