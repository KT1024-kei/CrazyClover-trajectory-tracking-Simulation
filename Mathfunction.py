import numpy as np

class Mathfunction():

  def Vee(self, Vector):
    return np.array([[0, -Vector[2], Vector[1]], [Vector[2], 0, -Vector[0]], [-Vector[1], Vector[0], 0]])

  def Wedge(self, Mat):
    return np.array([Mat[2, 1], -Mat[2, 0], Mat[1, 0]])

  def saturation(self, param, UP_limit, LOW_limit):

    return max(min(param, UP_limit), LOW_limit)
  
  # Body angular velocity to Euler angle rate
  def BAV2EAR(self, Euler, Wb): 
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Euler_angle_rate = np.matmul(np.linalg.inv(np.array([[1.0, 0.0, -sinP],
                                        [0.0, cosR,  cosP * sinR],
                                        [0.0, -sinR, cosP * cosR]])) , Wb)
    # Euler_angle_rate = np.matmul(np.array([[cosR, sinR, 0.0],
    #                                       [-cosP*sinR, cosP*cosR, 0.0],
    #                                       [sinP*cosR, sinP*sinR, cosP]])/cosP, Wb)
  
    return Euler_angle_rate

  def EAR2BAV(self, Euler, Euler_rate):
    r = Euler[0]
    p = Euler[1]
    y = Euler[2]

    cosR = np.cos(r);sinR = np.sin(r)
    cosP = np.cos(p);sinP = np.sin(p)

    Wb = np.matmul((np.array([[1.0, 0.0, -sinP],
                                        [0.0, cosR,  cosP * sinR],
                                        [0.0, -sinR, cosP * cosR]])) , Euler_rate)
    
    return Wb
class Integration():
  
  def __init__(self, dt, param):
    self.dt = dt
    self.integral = param

  def integration(self, param):
    self.integral += param*self.dt    
    return self.integral