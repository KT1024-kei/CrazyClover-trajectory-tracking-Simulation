import sys
sys.path.append('../')

import numpy as np

from tools.Mathfunction import Mathfunction
from Drone.State import State
from Drone.Inner_controller import Controller_attituede_rate

class Drone(Mathfunction):
  def __init__(self, dt):
    print("Initial DRONE model")

    self.set_parametor(dt)
    self.inner_controller = Controller_attituede_rate(dt, self.mQ, self.I)
    
  def set_parametor(self, dt):
    # print("Set Simulation and Physical parametor")

    # Physical Parametor
    self.g = 9.8
    self.mQ = 0.67
    self.I = np.array([[10**(-2) , 0.0, 0.0],[0.0, 10**(-2), 0.0], [0.0, 0.0, 10**(-1)]])
    self.Arm_length = 0.15
    self.e3 = np.array([0, 0, 1.0])
    self.dt = dt

    self.CM_MP2FM  = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, -1.0, 1.0, 1.0], [-1.0, 1.0, 1.0, -1.0], [1.0, -1.0, 1.0, -1.0]])
    self.M = np.zeros(4)
 
  def set_initial_state(self, P, V, R, Euler, Wb, Euler_rate, dt):
    # print("set references")
    self.P = State(dt, P)
    self.V = State(dt, V)
    self.R = State(dt, R)
    self.Euler = State(dt, Euler)
    self.Wb = State(dt, Wb)
    self.Euler_rate = State(dt, Euler_rate)
  
  def update_state(self, acc, Omega_acc):
    # print("update phsycal state")
    self.V.integration(acc)
    self.Wb.integration(Omega_acc)
    self.Euler_rate.update((self.BAV2EAR(self.Euler.now, self.Wb.now)))
    self.P.integration(self.V.now)
    self.Euler_rate.pre[1] = -self.Euler_rate.pre[1]
    self.Euler.integration(self.Euler_rate.pre)
    self.R.integration(np.matmul(self.R.now, self.Vee(self.Wb.pre)))

    self.inner_controller.set_state(self.Wb.now, self.Euler_rate.now)
    # print(self.R.now)

    
  def MP2FM(self, Moter_Power):
    # print("Input Thrust[gF] and Euler rate[rad/s]")

    return np.matmul(self.CM_MP2FM, Moter_Power)

  def get_input_acc_and_Wb(self, acc, Wb):

    self.inner_controller.inner_controller2(acc, Wb)
    return self.inner_controller.MP_pwm

  def Power_destribution_stock(self, T, Eulerrate):
    # print("Destribute Power to each Moter")

    Moter_Power = np.zeros(4)
    r = Eulerrate[0]/2.0
    p = Eulerrate[1]/2.0
    y = Eulerrate[2]

    Moter_Power[0] = self.saturation(T - r + p + y, 35000.0, 0.0)
    Moter_Power[1] = self.saturation(T - r - p - y, 35000.0, 0.0)
    Moter_Power[2] = self.saturation(T + r - p + y, 35000.0, 0.0)
    Moter_Power[3] = self.saturation(T + r + p - y, 35000.0, 0.0)
    # print(Moter_Power)
    return Moter_Power

  def Power_destribution_stock2(self, IN_Moter_Power):
      # print("Destribute Power to each Moter")

      Moter_Power = np.zeros(4)


      Moter_Power[0] = self.saturation(IN_Moter_Power[0], 35000.0, 0.0)
      Moter_Power[1] = self.saturation(IN_Moter_Power[1], 35000.0, 0.0)
      Moter_Power[2] = self.saturation(IN_Moter_Power[2], 35000.0, 0.0)
      Moter_Power[3] = self.saturation(IN_Moter_Power[3], 35000.0, 0.0)
      # print(Moter_Power)
      return Moter_Power

  def Drone_Dynamics(self, F, M):
    # print("Calcurate Drone motion")
    M[0:2] = M[0:2] * self.Arm_length/(2.0*np.sqrt(2))
    # M[0] = 0
    # M[1] = 0
    # M[2] = 0

    acc = (self.g*F*np.matmul(self.R.now, self.e3)/(self.mQ*1000.0) - self.g*self.e3)
    Omega_acc = np.matmul(np.linalg.inv(self.I), (M - np.cross(self.Wb.now, np.matmul(self.I, self.Wb.now))))
    return acc, Omega_acc

  def MM_pwm2gf(self, moter):
    # print("Moter map: PWM -> gF ")
    In_m1 = moter[0]
    In_m2 = moter[1]
    In_m3 = moter[2]
    In_m4 = moter[3]

    Out_m = np.zeros(4)

    m1_map = np.array([2.077e-07, 0.0021, 0.0])
    m2_map = np.array([2.1910e-07, 0.0022, 0.0])
    m3_map = np.array([2.1161e-07, 0.0024, 0.0])
    m4_map = np.array([2.0210e-07, 0.0024, 0.0])
    
    sign_m1 = np.sign(In_m1)
    Out_m[0] = sign_m1 * np.dot(m1_map, np.array([In_m1**2, sign_m1*np.abs(In_m1), 0.0]))

    sign_m2 = np.sign(In_m2)
    Out_m[1] = sign_m2 * np.dot(m2_map, np.array([In_m2**2, sign_m2*np.abs(In_m2), 0.0]))

    sign_m3 = np.sign(In_m3)
    Out_m[2] = sign_m3 * np.dot(m3_map, np.array([In_m3**2, sign_m3*np.abs(In_m3), 0.0]))

    sign_m4 = np.sign(In_m4)
    Out_m[3] = sign_m4 * np.dot(m4_map, np.array([In_m4**2, sign_m4*np.abs(In_m4), 0.0]))

    return Out_m

  def main(self, acc, Wb):

    self.M = self.get_input_acc_and_Wb(acc, Wb)

    # M_pwm = self.Power_destribution_stock(Thrust, Euelr_rate)
    M_pwm = self.Power_destribution_stock2(self.M)
    # print('C', M_pwm)
    M_gf = self.MM_pwm2gf(M_pwm)
    # print(M_gf)
    self.F_and_M = self.MP2FM(M_gf)
    # print(self.F_and_M[1:])
    acc, Wb_acc = self.Drone_Dynamics(self.F_and_M[0], self.F_and_M[1:])
    self.update_state(acc,  Wb_acc)

    return acc, Wb_acc