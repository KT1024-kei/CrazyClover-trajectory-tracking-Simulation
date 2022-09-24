import sys
sys.path.append('../')

import numpy as np
from tools.pid import PID
# from Drone.Drone_model import Drone
from tools.Mathfunction import Mathfunction

class Controller_attituede_rate(Mathfunction):

  def __init__(self, dt, mQ):
    print("initialize Attitude rate controller")

    self.mQ = mQ
    self.g = 9.8
    # Euler angle rate PID
    self.R_rate_pid = PID(5.0, 2, 0.0, dt)
    self.P_rate_pid = PID(5.0, 2, 0.0, dt)
    self.Y_rate_pid = PID(15.0, 5, 0.0, dt)
    
    # Body angle velocity PID
    self.Wx_pid = PID(50.0, 20, 0.0, dt)
    self.Wy_pid = PID(50.0, 20, 0.0, dt)
    self.Wz_pid = PID(5, 1, 0.0, dt)
    
    self.M_gf = np.array([0.0, 0.0, 0.0])
    self.FM_pwm = np.array([0.0, 0.0, 0.0, 0.0])

    self.MP_gf = np.array([0.0, 0.0, 0.0, 0.0])
    self.MP_pwm = np.array([0.0, 0.0, 0.0, 0.0])

    self.rad2deg = 180.0/np.pi
    self.deg2rad = np.pi/180.0

    # Controll Matrix
    KT = self.mQ/(self.g*4)*1000.0
    KR = 1.0/4
    KP = 1.0/4
    KY = 1.0/4
    self.FM2MP_map  = np.array([[KT, KT, KT, KT], [-KR, -KR, KR, KR], [-KP, KP, KP, -KP], [-KY, KY, -KY, KY]]).T
    self.MP2FM_map = np.array([[1.0, 1.0, 1.0, 1.0], [-1.0, -1.0, 1.0, 1.0], [-1.0, 1.0, 1.0, -1.0], [-1.0, 1.0, -1.0, 1.0]])

  def set_state(self, Wb, Euler_rate):
      
    # print("set references")
    self.Wb = Wb
    self.Euler_rate = Euler_rate

    self.Wx_pid.state = Wb[0]
    self.Wy_pid.state = Wb[1]
    self.Wz_pid.state = Wb[2]

    self.R_rate_pid.state = Euler_rate[0] * self.rad2deg
    self.P_rate_pid.state = -Euler_rate[1] * self.rad2deg
    self.Y_rate_pid.state = Euler_rate[2] * self.rad2deg
  
  def inner_controller(self, F, Euler_rate_input):
    
    self.R_rate_pid.desired = Euler_rate_input[0]
    self.P_rate_pid.desired = Euler_rate_input[1]
    self.Y_rate_pid.desired = Euler_rate_input[2]

    self.controll_attitude_rate()
    self.FM2MP(F)
    self.MM_gf2pwm()
    self.MM_gf2pwm2(F)
    # print("C",self.MP_gf)
    # print(self.M_gf)

  def inner_controller2(self, F, Wb):
    
    # set desired angular velocity
    self.Wx_pid.desired = Wb[0]
    self.Wy_pid.desired = Wb[1]
    self.Wz_pid.desired = Wb[2]

    self.controll_attitude_rate2()
    # self.FM2MP(F)
    # self.MM_gf2pwm()
    self.MM_gf2pwm2(F)
    # print("C",self.MP_gf)
    # print(self.M_gf)

  def controll_attitude_rate(self):

    self.R_rate_pid.runpid()
    self.P_rate_pid.runpid()
    self.Y_rate_pid.runpid()
    
    self.M_gf[0] = self.R_rate_pid.output*self.deg2rad
    self.M_gf[1] = self.P_rate_pid.output*self.deg2rad
    self.M_gf[2] = self.Y_rate_pid.output*self.deg2rad

  def controll_attitude_rate2(self):

    # Wb = self.EAR2BAV(Euler, Euler_rate)

    self.Wx_pid.runpid()
    self.Wy_pid.runpid()
    self.Wz_pid.runpid()
    
    self.M_gf[0] = self.Wx_pid.output
    self.M_gf[1] = self.Wy_pid.output
    self.M_gf[2] = self.Wz_pid.output


  def FM2MP(self, F):

    self.MP_gf = np.matmul(self.FM2MP_map, np.array([F, self.M_gf[0], self.M_gf[1], -self.M_gf[2]]))



  def MM_gf2pwm(self):
    # print("Moter map: gF -> PWM ")
    In_m1 = self.MP_gf[0]
    In_m2 = self.MP_gf[1]
    In_m3 = self.MP_gf[2]
    In_m4 = self.MP_gf[3]
    

    sign_m1 = np.sign(In_m1)
    self.MP_pwm[0] = sign_m1 * (np.sqrt(sign_m1*In_m1*8.309953163553529e-7+4.231999208818486e-6)*2.406752433662037e+6-4.951128620134714e+3)
    # self.MP_pwm[0] = sign_m1 * (-1.0785e-5 * In_m1**4 + sign_m1* 0.0080 * In_m1**3 - 2.0815 * In_m1**2 +  306.7172*In_m1 + 1.7019e+03)

    sign_m2 = np.sign(In_m2)
    self.MP_pwm[1] = sign_m2 * (np.sqrt(sign_m2*In_m2*8.763965396891775e-7+4.838977432913171e-6)*2.282071995297151e+6-5.020028004430765e+3)
    # self.MP_pwm[1] = sign_m2 * (-8.3705e-06 * In_m2**4 + sign_m2* 0.0066 * In_m2**3 - 1.8432 * In_m2**2 +  288.9068*In_m2 + 1.8032e+03)

    sign_m3 = np.sign(In_m3)
    self.MP_pwm[2] = sign_m3 * (np.sqrt(sign_m3*In_m3*8.464410710967024e-7+5.931205410463058e-6)*2.362834305061159e+6-5.75446231128539e+3)
    # self.MP_pwm[2] = sign_m3 * (-8.6533e-06 * In_m3**4 + sign_m3*  0.0068 * In_m3**3 -1.8811 * In_m3**2 + 290.2357*In_m3 + 1.8179e+03)

    sign_m4 = np.sign(In_m4)
    self.MP_pwm[3] = sign_m4 * (np.sqrt(sign_m4*In_m4*8.083914057959226e-7+5.805476588712849e-6)*2.474049062942287e+6-5.961111523577613e+3)
    # self.MP_pwm[3] = sign_m4 * (-9.6405e-06 * In_m4**4 + sign_m4* 0.0073 * In_m4**3 -1.9698 * In_m4**2 +  298.3715*In_m4 + 1.6579e+03)

  def MM_gf2pwm2(self, F):
    # print("Moter map: gF -> PWM ")

    self.MP_gf = np.matmul(self.FM2MP_map, np.array([F, self.M_gf[0], self.M_gf[1], 0])) # Thrust, Roll, Pitch

    In_m1 = self.MP_gf[0]
    In_m2 = self.MP_gf[1]
    In_m3 = self.MP_gf[2]
    In_m4 = self.MP_gf[3]
    

    sign_m1 = np.sign(In_m1)
    self.MP_pwm[0] = sign_m1 * (np.sqrt(sign_m1*In_m1*8.309953163553529e-7+4.231999208818486e-6)*2.406752433662037e+6-4.951128620134714e+3)
    # self.MP_pwm[0] = sign_m1 * (-1.0785e-5 * In_m1**4 + sign_m1* 0.0080 * In_m1**3 - 2.0815 * In_m1**2 +  306.7172*In_m1 + 1.7019e+03)

    sign_m2 = np.sign(In_m2)
    self.MP_pwm[1] = sign_m2 * (np.sqrt(sign_m2*In_m2*8.763965396891775e-7+4.838977432913171e-6)*2.282071995297151e+6-5.020028004430765e+3)
    # self.MP_pwm[1] = sign_m2 * (-8.3705e-06 * In_m2**4 + sign_m2* 0.0066 * In_m2**3 - 1.8432 * In_m2**2 +  288.9068*In_m2 + 1.8032e+03)

    sign_m3 = np.sign(In_m3)
    self.MP_pwm[2] = sign_m3 * (np.sqrt(sign_m3*In_m3*8.464410710967024e-7+5.931205410463058e-6)*2.362834305061159e+6-5.75446231128539e+3)
    # self.MP_pwm[2] = sign_m3 * (-8.6533e-06 * In_m3**4 + sign_m3*  0.0068 * In_m3**3 -1.8811 * In_m3**2 + 290.2357*In_m3 + 1.8179e+03)

    sign_m4 = np.sign(In_m4)
    self.MP_pwm[3] = sign_m4 * (np.sqrt(sign_m4*In_m4*8.083914057959226e-7+5.805476588712849e-6)*2.474049062942287e+6-5.961111523577613e+3)
    # self.MP_pwm[3] = sign_m4 * (-9.6405e-06 * In_m4**4 + sign_m4* 0.0073 * In_m4**3 -1.9698 * In_m4**2 +  298.3715*In_m4 + 1.6579e+03)

    self.M_gf[2] = self.M_gf[2]*(10**(-4))
    In_m1_torque = -0.25*(-self.M_gf[2])
    In_m2_torque = 0.25*(-self.M_gf[2])
    In_m3_torque = -0.25*(-self.M_gf[2])
    In_m4_torque = 0.25*(-self.M_gf[2])

    sign_m1 = np.sign(In_m1_torque)
    self.MP_pwm[0] += sign_m1 * (np.sqrt(sign_m1 *In_m1_torque*1.006001536082039e-10+1.587299594622495e-13)*1.988068534953908e+10-7.920649662566478e+3)
    # self.MP_pwm[0] = sign_m1 * (-1.0785e-5 * In_m1**4 + sign_m1* 0.0080 * In_m1**3 - 2.0815 * In_m1**2 +  306.7172*In_m1 + 1.7019e+03)

    sign_m2 = np.sign(In_m2_torque)
    self.MP_pwm[1] += sign_m2 * (np.sqrt(sign_m2 *In_m2_torque*9.66189557532568e-11+1.932237720338033e-13)*2.069987182543716e+10-9.09908944642907e+3)
    # self.MP_pwm[1] = sign_m2 * (-8.3705e-06 * In_m2**4 + sign_m2* 0.0066 * In_m2**3 - 1.8432 * In_m2**2 +  288.9068*In_m2 + 1.8032e+03)

    sign_m3 = np.sign(In_m3_torque)
    self.MP_pwm[2] += sign_m3 * (np.sqrt(sign_m3 *In_m3_torque*1.12726263933792e-10+1.24935566032467e-13)*1.774209425741873e+10-6.271160652750821e+3)
    # self.MP_pwm[2] = sign_m3 * (-8.6533e-06 * In_m3**4 + sign_m3*  0.0068 * In_m3**3 -1.8811 * In_m3**2 + 290.2357*In_m3 + 1.8179e+03)

    sign_m4 = np.sign(In_m4_torque)
    self.MP_pwm[3] += sign_m4 * (np.sqrt(sign_m4 *In_m4_torque*8.969381778123772e-11+1.667909502882046e-13)*2.229808084296265e+10-9.106546870860521e+3)
    # self.MP_pwm[3] = sign_m4 * (-9.6405e-06 * In_m4**4 + sign_m4* 0.0073 * In_m4**3 -1.9698 * In_m4**2 +  298.3715*In_m4 + 1.6579e+03)
    print(sign_m1 * (np.sqrt(sign_m1 *In_m1_torque*1.006001536082039e-10+1.587299594622495e-13)*1.988068534953908e+10-7.920649662566478e+3))

    



    


