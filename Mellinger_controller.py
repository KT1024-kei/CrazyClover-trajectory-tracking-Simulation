import numpy as np
from Trajectory import Trajectory
from Mathfunction import Mathfunction
from Inner_controller import Controller_attituede_rate
from Log import Log_data

class Mellinger(Mathfunction):
  def __init__(self, dt):
    self.dt = dt
    self.inner_controller = Controller_attituede_rate(self.dt)
  def mellinger_init(self):
    print("Init Mellinger Controller")

    # init trajectory
    self.kp = np.array([1.0, 1.0, 1.0])
    self.kv = np.array([1.0, 1.0, 1.0])
    self.ka = np.array([0.0, 0.0, 0.0])
    self.kR = np.array([100.0, 100.0, 10.0])

    self.Euler_nom = np.array([0.0, 0.0, 0.0])
    self.Euler_rate_nom = np.array([0.0, 0.0, 0.0])
    self.traj_W = np.zeros(3)

  def set_reference(self, traj_plan):
    self.trajectory = Trajectory(traj_plan)
    self.rad2deg = 180.0/np.pi

  def set_state(self, P, V, R, Euler, Wb, Euler_rate):
      
    # print("set references")
    self.P = P
    self.V = V
    self.R = R
    self.Euler = Euler
    self.Wb = Wb
    self.Euler_rate = Euler_rate
    self.inner_controller.set_state(Wb, Euler_rate)
  
  def Position_controller(self):
    traj_pos = self.trajectory.traj_pos
    traj_vel = self.trajectory.traj_vel
    traj_acc = self.trajectory.traj_acc
    
    self.ref_acc = self.kp * (traj_pos - self.P) + self.kv*(traj_vel - self.V) + traj_acc
    # self.ref_acc = traj_acc
    # print(self.P, traj_pos, self.V, traj_vel)
    self.input_acc = np.linalg.norm(self.ref_acc)

  def Attitude_controller(self):
    
    traj_acc = self.trajectory.traj_acc
    traj_jer = self.trajectory.traj_jer
    traj_yaw = self.trajectory.traj_yaw
    traj_yaw_rate = self.trajectory.traj_yaw_rate

    # nominal Rotation matrics
    traj_R = np.zeros((3, 3))
    traj_Rxc = np.array([np.cos(traj_yaw), np.sin(traj_yaw), 0.0])
    traj_Ryc = np.array([-np.sin(traj_yaw), np.cos(traj_yaw), 0.0])
    traj_Rz = self.ref_acc/np.linalg.norm(self.ref_acc)
    traj_Rx = np.cross(traj_Ryc, traj_Rz)/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    traj_Ry = np.cross(traj_Rz, traj_Rx)
    # traj_Ry = np.cross(traj_Rz, traj_Rxc)
    # traj_Rx = np.cross(traj_Ry, traj_Rz)

    traj_R[:, 0] = traj_Rx
    traj_R[:, 1] = traj_Ry
    traj_R[:, 2] = traj_Rz

    print(traj_R)
    # nominal Angular velocity

    traj_wy =  np.dot(traj_Rx, traj_jer) / np.dot(traj_Rz, traj_acc)
    traj_wx = -np.dot(traj_Ry, traj_jer) / np.dot(traj_Rz, traj_acc)
    traj_wz = (traj_yaw_rate * np.dot(traj_Rxc, traj_Rx) + traj_wy * np.dot(traj_Ryc, traj_Rz))/np.linalg.norm(np.cross(traj_Ryc, traj_Rz))
    self.traj_W[0] = traj_wx
    self.traj_W[1] = traj_wy
    self.traj_W[2] = traj_wz
    
    # when multiply rad2deg
    self.input_W = self.traj_W + self.kR*self.Wedge(-(np.matmul(traj_R.T, self.R) - np.matmul(self.R.T, traj_R))/2.0)
    print(self.kR*self.Wedge(np.matmul(traj_R.T, self.R) - np.matmul(self.R.T, traj_R))/2.0)
    

    self.Euler_nom[1] =  np.arctan( ( traj_acc[0]*np.cos(traj_yaw) + traj_acc[1]*np.sin(traj_yaw) ) / (traj_acc[2]))                                                        
    self.Euler_nom[0] = np.arctan( ( traj_acc[0]*np.sin(traj_yaw) - traj_acc[1]*np.cos(traj_yaw) ) / np.sqrt( (traj_acc[2])**2 + ( traj_acc[0]*np.cos(traj_yaw) + traj_acc[2]*np.sin(traj_yaw) )**2));  
    self.Euler_nom[2] = traj_yaw

    self.input_Euler_rate = self.BAV2EAR(self.Euler_nom, self.input_W)
    # self.input_Euler_rate = self.input_W
    # self.inner_controller.R_rate_pid.desired = self.input_Euler_rate[0]*self.rad2deg
    # self.inner_controller.P_rate_pid.desired = self.input_Euler_rate[1]*self.rad2deg
    # self.inner_controller.Y_rate_pid.desired = self.input_Euler_rate[2]*self.rad2deg
    
    # self.inner_controller.Wx_pid.desired = self.input_W[0]
    # self.inner_controller.Wy_pid.desired = self.input_W[1]
    # self.inner_controller.Wz_pid.desired = self.input_W[2]

    self.Euler_rate_nom = self.BAV2EAR(self.Euler_nom, self.traj_W)

  def mellinger_ctrl(self, t):
    self.trajectory.set_clock(t)
    self.trajectory.set_traj()
    self.Position_controller()
    self.Attitude_controller()
    # print(self.ref_acc)



    # self.inner_controller.inner_controller(self.input_acc, self.Euler_rate)
    self.inner_controller.inner_controller2(self.input_acc, self.input_W)
    self.input_thrust_pwm = self.inner_controller.FM_pwm[0]
    self.input_M_pwm = self.inner_controller.FM_pwm[1:]

  def log_nom(self, log, t):
    log.write_nom(t=t, P=self.trajectory.traj_pos, V=self.trajectory.traj_vel, Euler=self.Euler_nom, Wb=self.traj_W, Euler_rate=self.Euler_rate_nom)

    











    




    